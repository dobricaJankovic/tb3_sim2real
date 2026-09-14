"""Drive an identical open-loop command sequence and record what the robot did.

The point of this repository is that three backends are interchangeable. That
claim is only worth anything if it is measured, and it can only be measured with
ONE instrument: a per-backend script would be a per-backend result. So this node
runs on real, gazebo and isaacsim without knowing which it is talking to — it
publishes /cmd_vel and reads /odom, both of which every backend is contracted to
provide.

    ros2 run tb3_bringup drive_test --ros-args -p label:=gazebo -p out:=/tmp/g.json

It is open loop on purpose. Nav2 would correct exactly the errors we are trying
to measure.
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# name, linear.x (m/s), angular.z (rad/s), seconds.
#
# Kept inside the burger's real limits (0.22 m/s, 2.84 rad/s) and well inside
# them for the arc, because the interesting differences are in acceleration and
# wheel-slip handling, not in saturation. A settle phase brackets the run so
# odometry drift at rest is separable from drift under motion.
SEQUENCE = [
    ('settle_pre', 0.00, 0.00, 2.0),
    ('straight', 0.15, 0.00, 5.0),
    ('stop_1', 0.00, 0.00, 2.0),
    ('rotate', 0.00, 0.50, 5.0),
    ('stop_2', 0.00, 0.00, 2.0),
    ('arc', 0.10, 0.30, 5.0),
    ('settle_post', 0.00, 0.00, 3.0),
]

RATE = 20.0  # Hz, for both publishing and sampling


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DriveTest(Node):

    def __init__(self):
        super().__init__('drive_test')
        self.declare_parameter('label', 'unknown')
        self.declare_parameter('out', '')
        self.label = self.get_parameter('label').value
        self.out = self.get_parameter('out').value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Odometry is published RELIABLE by all three backends; ask for the same
        # so a mismatch is an error rather than a silently empty recording.
        self.create_subscription(Odometry, 'odom', self._on_odom,
                                 QoSProfile(depth=50,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.odom = None
        self.samples = []

    def _on_odom(self, msg):
        self.odom = msg

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _pose(self):
        p = self.odom.pose.pose
        t = self.odom.twist.twist
        return dict(t=self._now(), x=p.position.x, y=p.position.y,
                    yaw=yaw_of(p.orientation), vx=t.linear.x, wz=t.angular.z)

    def wait_for_odom(self, timeout=120.0):
        deadline = time.time() + timeout
        while rclpy.ok() and self.odom is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom is None:
            raise SystemExit('drive_test: no /odom after {:.0f}s'.format(timeout))
        # The clock can still be at zero for a moment after the first message.
        while rclpy.ok() and self._now() <= 0.0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('odom up; starting sequence')

    def run(self):
        self.wait_for_odom()
        results = []
        for name, lin, ang, secs in SEQUENCE:
            msg = Twist()
            msg.linear.x, msg.angular.z = lin, ang
            start = self._pose()
            t0 = start['t']
            phase = [start]
            while rclpy.ok() and self._now() - t0 < secs:
                self.pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=1.0 / RATE)
                phase.append(self._pose())
            end = phase[-1]
            results.append(self._summarise(name, lin, ang, secs, phase))
            self.samples.extend({'phase': name, **s} for s in phase)
            self.get_logger().info(
                '{:12s} cmd=({:.2f},{:.2f}) -> d={:.4f}m dyaw={:.4f}rad'.format(
                    name, lin, ang, results[-1]['distance'], results[-1]['d_yaw']))

        self.pub.publish(Twist())
        report = {'label': self.label, 'sequence': results,
                  'final': self._pose(), 'samples': self.samples}
        if self.out:
            with open(self.out, 'w') as f:
                json.dump(report, f, indent=1)
            self.get_logger().info('wrote ' + self.out)
        return report

    @staticmethod
    def _summarise(name, lin, ang, secs, phase):
        a, b = phase[0], phase[-1]
        dt = b['t'] - a['t']
        dx, dy = b['x'] - a['x'], b['y'] - a['y']
        d = math.hypot(dx, dy)
        d_yaw = math.atan2(math.sin(b['yaw'] - a['yaw']), math.cos(b['yaw'] - a['yaw']))
        # Path length, not displacement: an arc's chord understates how far the
        # wheels actually turned, and for a pure rotation the displacement is
        # ~0 while any nonzero path length is slip or noise.
        path = sum(math.hypot(q['x'] - p['x'], q['y'] - p['y'])
                   for p, q in zip(phase, phase[1:]))
        moving = [s for s in phase if abs(s['vx']) > 1e-6 or abs(s['wz']) > 1e-6]
        return {
            'phase': name, 'cmd_v': lin, 'cmd_w': ang,
            'commanded_s': secs, 'elapsed_s': round(dt, 4),
            'distance': round(d, 5), 'path_len': round(path, 5),
            'd_yaw': round(d_yaw, 5),
            'expected_distance': round(abs(lin) * dt, 5),
            'expected_d_yaw': round(ang * dt, 5),
            # Mean of the backend's own reported twist while it was moving: this
            # is what the robot says it did, as against what the pose integrates.
            'mean_vx': round(sum(s['vx'] for s in moving) / len(moving), 5) if moving else 0.0,
            'mean_wz': round(sum(s['wz'] for s in moving) / len(moving), 5) if moving else 0.0,
            'peak_vx': round(max((abs(s['vx']) for s in phase), default=0.0), 5),
            'samples': len(phase),
        }


def main(args=None):
    rclpy.init(args=args)
    node = DriveTest()
    try:
        node.run()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
