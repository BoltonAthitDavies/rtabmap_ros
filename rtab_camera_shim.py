#!/usr/bin/env python3
"""Bridge the sim bags into the topics RTAB-Map's stereo pipeline requires.

The recorded bags carry only /camN/image_raw/compressed -- no camera_info and no
TF. RTAB-Map refuses to start without both, so this node manufactures them from
the same calibration VINS reads, which keeps a single source of truth:

    vins_fusion_ros2/config/wil_sim/cam0_pinhole.yaml   intrinsics
    vins_fusion_ros2/config/wil_sim/cam1_pinhole.yaml
    vins_fusion_ros2/config/wil_sim/stereo_imu.yaml     body_T_cam0 / body_T_cam1

Published:
    <ns>/left/image_rect    bgr8   (colour, so the assembled map is not grey)
    <ns>/right/image_rect   mono8
    <ns>/left/camera_info
    <ns>/right/camera_info
    static TF  <body_frame> -> cam0_optical, cam1_optical

Two deliberate choices worth knowing about:

  * The images are published as "rect" WITHOUT any rectification step. A gz
    camera is an ideal pinhole -- cam0_pinhole.yaml has every distortion term at
    0.0 and both cameras share intrinsics -- so the raw frames already satisfy
    what rectification would produce. This is only true for the sim. The real
    rig's ~180 deg KANNALA_BRANDT fisheye would need a genuine remap first.

  * Both frames are stamped with the LEFT frame's time. cam0 and cam1 are
    written ~15 ms apart in the bag, and RTAB-Map's stereo path wants a
    synchronised pair. This matches what orbslam3_node.cpp already does
    (see its comment at onStereo) so the two systems stay comparable.
"""
import math
import os
import re
import sys

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import message_filters
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

WORKSPACE = os.path.expanduser('~/wil_project')
CONFIG_DIR = os.path.join(WORKSPACE, 'vins_fusion_ros2', 'config', 'wil_sim')


def read_pinhole(path):
    """fx, fy, cx, cy, width, height out of a VINS camera yaml (OpenCV format)."""
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        sys.exit(f'cannot open {path}')
    proj = fs.getNode('projection_parameters')
    intr = {k: proj.getNode(k).real() for k in ('fx', 'fy', 'cx', 'cy')}
    intr['width'] = int(fs.getNode('image_width').real())
    intr['height'] = int(fs.getNode('image_height').real())

    # Guard the assumption in the docstring: a non-zero distortion term means
    # these frames are NOT already rectified and publishing them as image_rect
    # would silently corrupt every triangulated point.
    dist = fs.getNode('distortion_parameters')
    if not dist.empty():
        bad = [k for k in ('k1', 'k2', 'p1', 'p2')
               if abs(dist.getNode(k).real()) > 1e-9]
        if bad:
            sys.exit(f'{path}: non-zero distortion {bad}; rectify before using this shim')
    fs.release()
    return intr


def _check_rigid(T, name):
    """A rotation block must be orthonormal with det +1. Fail loudly if not."""
    R = T[:3, :3]
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-6) or abs(np.linalg.det(R) - 1.0) > 1e-6:
        sys.exit(f'{name} is not a rigid transform (det={np.linalg.det(R):.3e}).\n{T}')
    return T


def read_extrinsics(path):
    """body_T_cam0, body_T_cam1 as 4x4 numpy arrays.

    Parsed from the text rather than through cv2.FileStorage. FileStorage
    SILENTLY MISREADS this file: the matrices are declared `dt: d` but their
    data lists write whole numbers bare (`0`, `1`, `-1`) instead of `0.0`, and
    OpenCV 4.11 stuffs those into the double array as raw ints. The result is
    that only the entries written with a decimal point survive -- so the
    TRANSLATION column reads back exactly right while the whole ROTATION block
    comes out as garbage on the order of 1e9 to 1e18.

    That failure is invisible without a check: the shim starts, prints a correct
    baseline, and publishes a static TF whose quaternion normalises to a camera
    pitched ~34 deg and yawed ~-27 deg from reality. Every point then lands in
    the wrong place, and the occupancy grid looks like smears rather than walls.

    Hence the parser below plus _check_rigid() on the way out.
    """
    txt = open(path).read()
    mats = []
    for key in ('body_T_cam0', 'body_T_cam1'):
        m = re.search(key + r'\s*:\s*!!opencv-matrix(.*?)\]', txt, re.S)
        if not m:
            sys.exit(f'{key} not found in {path}')
        block = m.group(1)
        rows = int(re.search(r'rows\s*:\s*(\d+)', block).group(1))
        cols = int(re.search(r'cols\s*:\s*(\d+)', block).group(1))
        data = re.search(r'data\s*:\s*\[(.*)', block, re.S).group(1)
        vals = [float(v) for v in re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', data)]
        if len(vals) != rows * cols:
            sys.exit(f'{key}: expected {rows * cols} values, parsed {len(vals)}')
        mats.append(_check_rigid(np.array(vals, float).reshape(rows, cols), key))
    return tuple(mats)


def quat_from_matrix(R):
    """Rotation matrix -> (x, y, z, w). Shepperd's method, branch on the largest term."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class CameraShim(Node):
    def __init__(self):
        super().__init__('rtab_camera_shim')

        self.declare_parameter('left_topic', '/cam0/image_raw/compressed')
        self.declare_parameter('right_topic', '/cam1/image_raw/compressed')
        self.declare_parameter('namespace', '/stereo')
        self.declare_parameter('body_frame', 'body')
        self.declare_parameter('config_dir', CONFIG_DIR)
        # RTAB-Map only runs a loop-closure check at Rtabmap/DetectionRate (1 Hz),
        # so republishing all 24 Hz of the bag just moves 66 MB/s of bgr8 through
        # DDS for nothing -- and in practice the transport, not the JPEG decode
        # (5 ms/frame), is what cannot keep up. 4 Hz leaves ample margin.
        self.declare_parameter('rate', 4.0)

        cfg = self.get_parameter('config_dir').value
        rate = float(self.get_parameter('rate').value)
        self.min_period = 1.0 / rate if rate > 0.0 else 0.0
        self.last_stamp = None
        ns = self.get_parameter('namespace').value
        self.body_frame = self.get_parameter('body_frame').value

        left = read_pinhole(os.path.join(cfg, 'cam0_pinhole.yaml'))
        right = read_pinhole(os.path.join(cfg, 'cam1_pinhole.yaml'))
        T0, T1 = read_extrinsics(os.path.join(cfg, 'stereo_imu.yaml'))

        # cam0 sits at body-y +0.0936, cam1 at 0.0, and script/check_stereo_side.py
        # confirms cam1 is physically to cam0's right on this bag. So cam0 is LEFT.
        baseline = float(np.linalg.norm(T0[:3, 3] - T1[:3, 3]))
        self.get_logger().info(
            f'left {left["width"]}x{left["height"]} fx={left["fx"]:.1f} '
            f'cx={left["cx"]:.1f} cy={left["cy"]:.1f}, baseline {baseline * 1000:.1f} mm')

        self.info_left = self._camera_info(left, 'cam0_optical', 0.0)
        # Tx = -fx * B is what tells RTAB-Map the pair's scale; get its sign wrong
        # and every depth comes out negative.
        self.info_right = self._camera_info(right, 'cam1_optical', -right['fx'] * baseline)

        self._publish_static_tf(T0, T1)

        self.pub_left = self.create_publisher(Image, f'{ns}/left/image_rect', 10)
        self.pub_right = self.create_publisher(Image, f'{ns}/right/image_rect', 10)
        self.pub_info_left = self.create_publisher(CameraInfo, f'{ns}/left/camera_info', 10)
        self.pub_info_right = self.create_publisher(CameraInfo, f'{ns}/right/camera_info', 10)

        sub_l = message_filters.Subscriber(
            self, CompressedImage, self.get_parameter('left_topic').value)
        sub_r = message_filters.Subscriber(
            self, CompressedImage, self.get_parameter('right_topic').value)
        self.sync = message_filters.ApproximateTimeSynchronizer([sub_l, sub_r], 30, 0.03)
        self.sync.registerCallback(self.on_pair)

        self.n = 0
        self.get_logger().info(
            f'publishing {ns}/left|right/image_rect + camera_info at {rate:g} Hz')

    def _camera_info(self, intr, frame, tx):
        m = CameraInfo()
        m.header.frame_id = frame
        m.width, m.height = intr['width'], intr['height']
        m.distortion_model = 'plumb_bob'
        m.d = [0.0] * 5
        m.k = [intr['fx'], 0.0, intr['cx'],
               0.0, intr['fy'], intr['cy'],
               0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [intr['fx'], 0.0, intr['cx'], tx,
               0.0, intr['fy'], intr['cy'], 0.0,
               0.0, 0.0, 1.0, 0.0]
        return m

    def _publish_static_tf(self, T0, T1):
        self.static_tf = StaticTransformBroadcaster(self)
        out = []
        for T, child in ((T0, 'cam0_optical'), (T1, 'cam1_optical')):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.body_frame
            t.child_frame_id = child
            t.transform.translation.x = float(T[0, 3])
            t.transform.translation.y = float(T[1, 3])
            t.transform.translation.z = float(T[2, 3])
            q = quat_from_matrix(T[:3, :3])
            (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w) = q
            out.append(t)
        self.static_tf.sendTransform(out)
        self.get_logger().info(f'static TF {self.body_frame} -> cam0_optical, cam1_optical')

    def on_pair(self, left_msg, right_msg):
        # Throttle on the message stamp, so the kept subset is identical whatever
        # --rate the bag is played at.
        t = left_msg.header.stamp.sec + left_msg.header.stamp.nanosec * 1e-9
        if self.min_period and self.last_stamp is not None:
            if t - self.last_stamp < self.min_period:
                return
        self.last_stamp = t

        l = cv2.imdecode(np.frombuffer(left_msg.data, np.uint8), cv2.IMREAD_COLOR)
        r = cv2.imdecode(np.frombuffer(right_msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
        if l is None or r is None:
            self.get_logger().warn('undecodable frame pair, skipped')
            return

        stamp = left_msg.header.stamp

        li = Image()
        li.header.stamp = stamp
        li.header.frame_id = 'cam0_optical'
        li.height, li.width = l.shape[:2]
        li.encoding = 'bgr8'
        li.step = li.width * 3
        li.data = l.tobytes()

        ri = Image()
        ri.header.stamp = stamp
        ri.header.frame_id = 'cam1_optical'
        ri.height, ri.width = r.shape[:2]
        ri.encoding = 'mono8'
        ri.step = ri.width
        ri.data = r.tobytes()

        self.info_left.header.stamp = stamp
        self.info_right.header.stamp = stamp

        self.pub_left.publish(li)
        self.pub_right.publish(ri)
        self.pub_info_left.publish(self.info_left)
        self.pub_info_right.publish(self.info_right)

        self.n += 1
        if self.n % 100 == 0:
            self.get_logger().info(f'{self.n} stereo pairs republished')


def main():
    rclpy.init()
    node = CameraShim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
