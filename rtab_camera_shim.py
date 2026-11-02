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

  * With camera_model:=pinhole (the sim) the images are published as "rect"
    WITHOUT any rectification step. A gz camera is an ideal pinhole --
    cam0_pinhole.yaml has every distortion term at 0.0 and both cameras share
    intrinsics -- so the raw frames already satisfy what rectification would
    produce. This is only true for the sim.
  * With camera_model:=fisheye (the real rig, ~180 deg KANNALA_BRANDT at
    1920x1080) every pair is remapped onto a virtual rectified pinhole pair of
    hfov_deg horizontal field of view before publishing. RTAB-Map's stereo path
    has no equidistant model, so this is the only way it can consume the rig.
    The rectification rotations come from cv2.fisheye.stereoRectify; the
    projection matrices are built by hand because OpenCV's own estimate
    degenerates for a lens whose corners map to infinity (fx came back 0.0 on
    dataset_realv5_000). The side (left_cam) is VALIDATED against the extrinsic
    and the published TF is checked to put the right rectified camera at
    (+baseline, 0, 0) of the left one. See MEMORY_CLAUDE/README.md 7d.

  * Both frames are stamped with the LEFT frame's time. cam0 and cam1 are
    written ~15 ms apart in the bag, and RTAB-Map's stereo path wants a
    synchronised pair. This matches what orbslam3_node.cpp already does
    (see its comment at onStereo) so the two systems stay comparable.
"""
import math
import os
import time
import re
import sys

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

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
    # Drop comments first. The real-rig config keeps every superseded
    # calibration as a commented-out block with the SAME key, and the first
    # `body_T_cam1:` in the file is one of those -- the regex would otherwise
    # return a retired extrinsic (found 2026-10-28: it handed back the
    # pre-remount cam1 at +0.0936 m, the sim-era geometry).
    txt = re.sub(r'#[^\n]*', '', open(path).read())
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


def read_fisheye(path):
    """K (3x3), D (4,), (width, height) out of a VINS KANNALA_BRANDT yaml.

    Parsed from the text for the same reason read_extrinsics() is: FileStorage
    cannot be trusted with these files. VINS's k2..k5 are OpenCV fisheye's
    k1..k4 in order (the file says so too).
    """
    txt = re.sub(r'#[^\n]*', '', open(path).read())
    if not re.search(r'model_type\s*:\s*KANNALA_BRANDT', txt):
        sys.exit(f'{path}: camera_model:=fisheye needs model_type KANNALA_BRANDT')

    def num(key):
        m = re.search(r'\b' + key + r'\s*:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', txt)
        if not m:
            sys.exit(f'{key} not found in {path}')
        return float(m.group(1))

    K = np.array([[num('mu'), 0.0, num('u0')],
                  [0.0, num('mv'), num('v0')],
                  [0.0, 0.0, 1.0]])
    D = np.array([num('k2'), num('k3'), num('k4'), num('k5')])
    size = (int(num('image_width')), int(num('image_height')))
    return K, D, size


def _rot4(R):
    T = np.eye(4)
    T[:3, :3] = R
    return T


def build_fisheye_rectification(cfg, left_cam, hfov_deg, scale=1.0):
    """Everything the fisheye path needs, computed once. Pure function so the
    offline epipolar check (script side) can exercise exactly this code.

    Returns dict with: maps_left/right (for cv2.remap), P_left/P_right (3x4),
    body_T_rect_left/right (4x4, the TF to publish), baseline, size, left/right
    camera indices, and the rectification rotations R_left/R_right.
    """
    if left_cam not in (0, 1):
        sys.exit(f'left_cam must be 0 or 1, got {left_cam!r}')
    right_cam = 1 - left_cam
    K = {}
    D = {}
    size = {}
    for i in (0, 1):
        K[i], D[i], size[i] = read_fisheye(os.path.join(cfg, f'cam{i}_fisheye.yaml'))
    if size[0] != size[1]:
        sys.exit(f'cam0/cam1 image sizes differ: {size[0]} vs {size[1]}')
    body_T = dict(zip((0, 1), read_extrinsics(os.path.join(cfg, 'stereo_imu.yaml'))))

    # Side check. In the LEFT camera's optical frame (+x right) the right camera
    # must sit at +x. This is the invariant the whole stereo path rests on and
    # it is the one that silently breaks when a rig is remounted
    # (realv3: cam1 moved to cam0's left; see memory realv3-camera-sides-swapped).
    camL_T_camR = np.linalg.inv(body_T[left_cam]) @ body_T[right_cam]
    tx = float(camL_T_camR[0, 3])
    if tx <= 0.0:
        sys.exit(f'left_cam={left_cam} is NOT the left camera: cam{right_cam} sits at '
                 f'x={tx:+.4f} m in its optical frame. Use left_cam:={right_cam}.')
    baseline = float(np.linalg.norm(camL_T_camR[:3, 3]))

    # cv2.fisheye.stereoRectify wants (R, T) taking LEFT points into the RIGHT
    # frame: X_R = R X_L + T.
    camR_T_camL = np.linalg.inv(camL_T_camR)
    R1, R2, _, _, _ = cv2.fisheye.stereoRectify(
        K[left_cam], D[left_cam], K[right_cam], D[right_cam], size[0],
        camR_T_camL[:3, :3], camR_T_camL[:3, 3],
        flags=cv2.CALIB_ZERO_DISPARITY, balance=0.0, fov_scale=1.0)

    W, H = (int(round(size[0][0] * scale)), int(round(size[0][1] * scale)))
    f = (W / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    P_left = np.array([[f, 0.0, W / 2.0, 0.0],
                       [0.0, f, H / 2.0, 0.0],
                       [0.0, 0.0, 1.0, 0.0]])
    P_right = P_left.copy()
    P_right[0, 3] = -f * baseline      # Tx = -fx * B, same convention as pinhole
    maps_left = cv2.fisheye.initUndistortRectifyMap(
        K[left_cam], D[left_cam], R1, P_left, (W, H), cv2.CV_16SC2)
    maps_right = cv2.fisheye.initUndistortRectifyMap(
        K[right_cam], D[right_cam], R2, P_right, (W, H), cv2.CV_16SC2)

    # X_rect = R1 X_cam, so cam_T_rect = R1^T and body_T_rect = body_T_cam R1^T.
    body_T_rect_left = _check_rigid(body_T[left_cam] @ _rot4(R1.T), 'body_T_rect_left')
    body_T_rect_right = _check_rigid(body_T[right_cam] @ _rot4(R2.T), 'body_T_rect_right')
    # The defining property of a rectified pair: the right camera lies on the
    # left one's +x axis with the same orientation. If this fails the maps and
    # the TF disagree and every triangulated point lands off its ray.
    rl_T_rr = np.linalg.inv(body_T_rect_left) @ body_T_rect_right
    if (not np.allclose(rl_T_rr[:3, 3], [baseline, 0.0, 0.0], atol=1e-6)
            or not np.allclose(rl_T_rr[:3, :3], np.eye(3), atol=1e-6)):
        sys.exit(f'rectified frames are not a canonical stereo pair:\n{rl_T_rr}')

    return dict(left_cam=left_cam, right_cam=right_cam, baseline=baseline,
                size=(W, H), f=f, hfov_deg=hfov_deg,
                R_left=R1, R_right=R2, P_left=P_left, P_right=P_right,
                maps_left=maps_left, maps_right=maps_right,
                body_T_rect_left=body_T_rect_left,
                body_T_rect_right=body_T_rect_right)


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
        # pinhole: the sim path above, untouched. fisheye: the real rig.
        self.declare_parameter('camera_model', 'pinhole')
        # fisheye only. Which camera index is physically LEFT; validated against
        # the extrinsic at startup. The topics are then /cam{i}/image_raw/compressed.
        self.declare_parameter('left_cam', 0)
        # fisheye only. Horizontal field of view of the virtual pinhole pair.
        # 110 deg at 1920 px gives fx=672 and fx*B=59.8 px*m with the 89 mm
        # baseline -- the same depth budget as the sim (59.9). Wider keeps more
        # of the fisheye but costs depth precision linearly.
        # dynamic_typing: `-p hfov_deg:=110` arrives as an INTEGER and rclpy
        # refuses to declare a DOUBLE with an integer override -- the shim then
        # dies at startup and rtabmap silently gets no data (2026-10-29: a whole
        # mapping run produced a 0-node database this way).
        self.declare_parameter('hfov_deg', 110.0,
                               ParameterDescriptor(dynamic_typing=True))
        # fisheye only. Output size as a fraction of the input. RTAB-Map runs
        # dense SGBM on every pair it keeps, and that cost scales with pixels x
        # disparity range, so 0.5 makes the back-end ~4-8x cheaper for the same
        # field of view. Depth precision per pixel halves with the focal length;
        # the grid is 5 cm cells so at 0.5 nothing visible is lost.
        self.declare_parameter('rect_scale', 1.0,
                               ParameterDescriptor(dynamic_typing=True))

        cfg = self.get_parameter('config_dir').value
        rate = float(self.get_parameter('rate').value)
        self.min_period = 1.0 / rate if rate > 0.0 else 0.0
        self.last_stamp = None
        ns = self.get_parameter('namespace').value
        self.body_frame = self.get_parameter('body_frame').value

        self.camera_model = self.get_parameter('camera_model').value
        self.rect = None
        if self.camera_model == 'pinhole':
            left = read_pinhole(os.path.join(cfg, 'cam0_pinhole.yaml'))
            right = read_pinhole(os.path.join(cfg, 'cam1_pinhole.yaml'))
            T0, T1 = read_extrinsics(os.path.join(cfg, 'stereo_imu.yaml'))

            # cam0 sits at body-y +0.0936, cam1 at 0.0, and script/check_stereo_side.py
            # confirms cam1 is physically to cam0's right on this bag. So cam0 is LEFT.
            baseline = float(np.linalg.norm(T0[:3, 3] - T1[:3, 3]))
            self.get_logger().info(
                f'left {left["width"]}x{left["height"]} fx={left["fx"]:.1f} '
                f'cx={left["cx"]:.1f} cy={left["cy"]:.1f}, baseline {baseline * 1000:.1f} mm')

            self.frame_left, self.frame_right = 'cam0_optical', 'cam1_optical'
            self.info_left = self._camera_info(left, self.frame_left, 0.0)
            # Tx = -fx * B is what tells RTAB-Map the pair's scale; get its sign wrong
            # and every depth comes out negative.
            self.info_right = self._camera_info(right, self.frame_right, -right['fx'] * baseline)
            self._publish_static_tf(((T0, self.frame_left), (T1, self.frame_right)))
            left_topic = self.get_parameter('left_topic').value
            right_topic = self.get_parameter('right_topic').value
        elif self.camera_model == 'fisheye':
            rect_scale = float(self.get_parameter('rect_scale').value)
            if not 0.1 <= rect_scale <= 1.0:
                sys.exit(f'rect_scale must be in [0.1, 1.0], got {rect_scale}')
            rect = build_fisheye_rectification(
                cfg, int(self.get_parameter('left_cam').value),
                float(self.get_parameter('hfov_deg').value), scale=rect_scale)
            self.rect = rect
            W, H = rect['size']
            self.get_logger().info(
                f'fisheye -> rectified pinhole {W}x{H}, hfov {rect["hfov_deg"]:g} deg, '
                f'fx={rect["f"]:.1f}, baseline {rect["baseline"] * 1000:.1f} mm, '
                f'fx*B={rect["f"] * rect["baseline"]:.1f} px*m; '
                f'left = cam{rect["left_cam"]}, right = cam{rect["right_cam"]}')
            self.frame_left = f'cam{rect["left_cam"]}_rect_optical'
            self.frame_right = f'cam{rect["right_cam"]}_rect_optical'
            intr = {'fx': rect['f'], 'fy': rect['f'], 'cx': W / 2.0, 'cy': H / 2.0,
                    'width': W, 'height': H}
            self.info_left = self._camera_info(intr, self.frame_left, 0.0)
            self.info_right = self._camera_info(intr, self.frame_right, rect['P_right'][0, 3])
            self._publish_static_tf(((rect['body_T_rect_left'], self.frame_left),
                                     (rect['body_T_rect_right'], self.frame_right)))
            left_topic = f'/cam{rect["left_cam"]}/image_raw/compressed'
            right_topic = f'/cam{rect["right_cam"]}/image_raw/compressed'
        else:
            sys.exit(f'camera_model must be pinhole or fisheye, got {self.camera_model!r}')

        self.pub_left = self.create_publisher(Image, f'{ns}/left/image_rect', 10)
        self.pub_right = self.create_publisher(Image, f'{ns}/right/image_rect', 10)
        self.pub_info_left = self.create_publisher(CameraInfo, f'{ns}/left/camera_info', 10)
        self.pub_info_right = self.create_publisher(CameraInfo, f'{ns}/right/camera_info', 10)

        sub_l = message_filters.Subscriber(self, CompressedImage, left_topic)
        sub_r = message_filters.Subscriber(self, CompressedImage, right_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer([sub_l, sub_r], 30, 0.03)
        self.sync.registerCallback(self.on_pair)

        self.n = 0
        self.proc_ms = []
        self.get_logger().info(
            f'{left_topic} + {right_topic} -> {ns}/left|right/image_rect + camera_info '
            f'at {rate:g} Hz')

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

    def _publish_static_tf(self, frames):
        self.static_tf = StaticTransformBroadcaster(self)
        out = []
        for T, child in frames:
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
        self.get_logger().info(
            f'static TF {self.body_frame} -> ' + ', '.join(c for _, c in frames))

    def on_pair(self, left_msg, right_msg):
        # Throttle on the message stamp, so the kept subset is identical whatever
        # --rate the bag is played at.
        t = left_msg.header.stamp.sec + left_msg.header.stamp.nanosec * 1e-9
        if self.min_period and self.last_stamp is not None:
            if t - self.last_stamp < self.min_period:
                return
        self.last_stamp = t
        t_wall = time.perf_counter()

        l = cv2.imdecode(np.frombuffer(left_msg.data, np.uint8), cv2.IMREAD_COLOR)
        r = cv2.imdecode(np.frombuffer(right_msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
        if l is None or r is None:
            self.get_logger().warn('undecodable frame pair, skipped')
            return
        if self.rect is not None:
            l = cv2.remap(l, *self.rect['maps_left'], cv2.INTER_LINEAR)
            r = cv2.remap(r, *self.rect['maps_right'], cv2.INTER_LINEAR)

        stamp = left_msg.header.stamp

        li = Image()
        li.header.stamp = stamp
        li.header.frame_id = self.frame_left
        li.height, li.width = l.shape[:2]
        li.encoding = 'bgr8'
        li.step = li.width * 3
        li.data = l.tobytes()

        ri = Image()
        ri.header.stamp = stamp
        ri.header.frame_id = self.frame_right
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
        self.proc_ms.append((time.perf_counter() - t_wall) * 1e3)
        if self.n % 20 == 0:
            # lag = how far behind sim time this pair's stamp is when it leaves
            # the shim. Anything beyond a few seconds means the input queue is
            # backing up, and past ~10 s rtabmap's TF buffer no longer covers
            # the stamp ("extrapolation into the past") and the pair is lost.
            now = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(
                f'{self.n} stereo pairs republished; lag {now - t:+.1f} s behind '
                f'sim time; {np.median(self.proc_ms[-20:]):.0f} ms/pair')


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
