"""Exit as soon as a simulator is actually publishing /clock.

Nav2 with use_sim_time:=true will wait for /clock on its own, but it does so
silently — which looks identical to a DDS domain mismatch or to a simulator
that never started. This node turns that into a readable line in the log, and
bringup.launch.py hangs the whole Nav2 stack off its exit, so on a simulated
backend Nav2 starts when sim time exists rather than two minutes before it.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock

# /clock is published best-effort by both Gazebo and Isaac Sim.
CLOCK_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class WaitForSim(Node):

    def __init__(self):
        super().__init__('wait_for_sim')
        self.declare_parameter('warn_after_sec', 10.0)
        self._warn_after = self.get_parameter('warn_after_sec').value
        self._elapsed = 0.0
        self._warned = False

        self.create_subscription(Clock, '/clock', self._on_clock, CLOCK_QOS)
        self._timer = self.create_timer(1.0, self._tick)
        self.get_logger().info('waiting for /clock ...')

    def _on_clock(self, msg):
        self.get_logger().info(
            f'simulator connected — /clock at {msg.clock.sec}.{msg.clock.nanosec:09d}')
        raise SystemExit

    def _tick(self):
        self._elapsed += 1.0
        if self._elapsed >= self._warn_after and not self._warned:
            self._warned = True
            self.get_logger().warn(
                'still no /clock. Isaac Sim on a cold shader cache takes two '
                'to three minutes to reach play(), and gzserver a few seconds; '
                'past that, check that the simulator is still alive and that it '
                'shares ROS_DOMAIN_ID with this shell.')


def main(args=None):
    rclpy.init(args=args)
    node = WaitForSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
