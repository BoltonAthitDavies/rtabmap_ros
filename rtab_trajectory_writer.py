#!/usr/bin/env python3
"""Write RTAB-Map's OPTIMISED trajectory as a vio.csv the compare scripts can read.

Subscribes to /mapPath (nav_msgs/Path), which RTAB-Map republishes IN FULL every
time the pose graph is optimised -- so unlike an append-only odometry stream it
carries the retroactive corrections a loop closure makes to earlier poses. That
is the whole point of putting RTAB-Map behind the front-end, and it is what has
to be measured to show a loop-closure benefit.

Output matches script/extract_gt.py and both estimators' vio.csv exactly:

    t_ns, x, y, z, qw, qx, qy, qz, vx, vy, vz

RTAB-Map publishes /mapPath from TWO code paths and only one is usable here:

  * CoreWrapper.cpp:4879, the live per-iteration publish, stamps EVERY pose with
    the same message time. A CSV built from it has one timestamp repeated on all
    rows and cannot be matched against ground truth at all.
  * CoreWrapper.cpp:4266, reached only via the `publish_map` service, stamps each
    pose with its node's real time.

So this node calls /rtabmap/publish_map (graph_only) on a timer and again at
shutdown, and only writes a Path whose stamps are actually distinct. The file is
rewritten each time one arrives, so an interrupted run still leaves the latest
optimised graph on disk.

Two things to know before comparing the numbers:

  * These poses are one per GRAPH NODE, produced at Rtabmap/DetectionRate (1 Hz),
    not one per camera frame. A 93 s run yields ~90 rows where VINS writes ~1900.
    That is correct, not a bug -- RTAB-Map only ever estimates node poses -- but
    any metric that assumes matching row counts needs timestamp matching.

  * Velocities are finite differences between consecutive optimised poses, not a
    filter state. Comparable in magnitude, which is all plot_vio_vs_gt.py uses,
    but far coarser than VINS' own velocity output.
"""
import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Path
from rtabmap_msgs.srv import PublishMap


def resolve(output_path):
    """A directory gets /vio.csv appended; a path already ending .csv is used as is."""
    p = os.path.expanduser(output_path)
    if not p.lower().endswith('.csv'):
        p = os.path.join(p, 'vio.csv')
    os.makedirs(os.path.dirname(p), exist_ok=True)
    return p


class TrajectoryWriter(Node):
    def __init__(self):
        super().__init__('rtab_trajectory_writer')
        self.declare_parameter('output_path', '')
        self.declare_parameter('path_topic', '/mapPath')
        self.declare_parameter('publish_map_service', '/rtabmap/publish_map')
        self.declare_parameter('request_period', 3.0)

        out = self.get_parameter('output_path').value
        if not out:
            self.get_logger().error('output_path is empty; nothing would be written')
            sys.exit(1)
        self.out = resolve(out)
        topic = self.get_parameter('path_topic').value

        self.create_subscription(Path, topic, self.on_path, 10)
        self.n_written = 0
        self.last_path = None

        self.cli = self.create_client(
            PublishMap, self.get_parameter('publish_map_service').value)
        self.create_timer(float(self.get_parameter('request_period').value),
                          self.request_graph)
        self.get_logger().info(f'writing optimised trajectory from {topic} -> {self.out}')

    def request_graph(self):
        """Ask RTAB-Map to republish the graph through the properly-stamped path."""
        if not self.cli.service_is_ready():
            return
        req = PublishMap.Request()
        req.global_map = True
        req.optimized = True
        req.graph_only = True      # skip point clouds; only the graph is wanted
        self.cli.call_async(req)

    @staticmethod
    def _stamps_distinct(poses):
        first = poses[0].header.stamp
        return any(p.header.stamp.sec != first.sec or
                   p.header.stamp.nanosec != first.nanosec for p in poses)

    def on_path(self, msg):
        poses = msg.poses
        if not poses:
            return

        self.last_path = msg
        # A single-pose graph cannot be judged either way; anything longer with a
        # uniform stamp came from the live publish and is unusable.
        if len(poses) > 1 and not self._stamps_distinct(poses):
            return

        rows = []
        for i, ps in enumerate(poses):
            t = ps.header.stamp.sec * 10**9 + ps.header.stamp.nanosec
            p, q = ps.pose.position, ps.pose.orientation

            # Central difference where possible, one-sided at the ends. dt comes
            # from the node stamps, which are irregular (RTAB-Map adds a node when
            # it decides to, not on a clock), so it cannot be assumed constant.
            j0 = max(i - 1, 0)
            j1 = min(i + 1, len(poses) - 1)
            if j0 == j1:
                vx = vy = vz = 0.0
            else:
                s0, s1 = poses[j0].header.stamp, poses[j1].header.stamp
                dt = ((s1.sec - s0.sec) + (s1.nanosec - s0.nanosec) * 1e-9)
                if dt > 1e-9:
                    a, b = poses[j0].pose.position, poses[j1].pose.position
                    vx, vy, vz = (b.x - a.x) / dt, (b.y - a.y) / dt, (b.z - a.z) / dt
                else:
                    vx = vy = vz = 0.0

            rows.append('%d,' % t + ','.join(
                '%.5f' % v for v in (p.x, p.y, p.z, q.w, q.x, q.y, q.z, vx, vy, vz)))

        # Write via a temporary file and rename, so a reader never catches the
        # file half-rewritten mid-run.
        tmp = self.out + '.tmp'
        with open(tmp, 'w') as f:
            f.write('\n'.join(rows) + '\n')
        os.replace(tmp, self.out)

        self.n_written += 1
        if self.n_written == 1 or self.n_written % 10 == 0:
            self.get_logger().info(f'{len(rows)} optimised poses -> {self.out}')


def main():
    rclpy.init()
    node = TrajectoryWriter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # One last request, then pump the executor briefly so the properly-stamped
        # reply is received and written before the process goes away.
        try:
            if rclpy.ok():
                node.request_graph()
                for _ in range(40):
                    rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            pass

        if node.n_written:
            node.get_logger().info(f'{node.n_written} write(s), final: {node.out}')
        elif node.last_path is not None:
            # Never saw distinct stamps. Write it anyway rather than losing the
            # run, but say plainly that the timestamps are unusable.
            node.get_logger().error(
                'only ever received uniformly-stamped paths -- is '
                '/rtabmap/publish_map reachable? writing anyway; TIMESTAMPS ARE '
                'NOT USABLE for comparison')
            with open(node.out, 'w') as f:
                for ps in node.last_path.poses:
                    t = ps.header.stamp.sec * 10**9 + ps.header.stamp.nanosec
                    p, q = ps.pose.position, ps.pose.orientation
                    f.write('%d,' % t + ','.join('%.5f' % v for v in (
                        p.x, p.y, p.z, q.w, q.x, q.y, q.z, 0.0, 0.0, 0.0)) + '\n')
        else:
            node.get_logger().error('no /mapPath received; nothing written')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
