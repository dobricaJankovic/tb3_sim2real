"""Send the same navigation goals to every backend and record what happened.

The task-layer companion to drive_test, and ONE instrument for the same
reason: a per-backend navigation script would produce a per-backend result.
This node knows nothing about which backend is running. It talks to
NavigateToPose, tf and /scan, all of which every backend supplies through the
same Nav2 stack, on the same map, with the same `nav2_params.yaml` — Nav2 must
not be tuned per backend, or the sim-to-real gap is absorbed into the tuning
and stops being measurable.

    ros2 run tb3_bringup nav_test --ros-args \
        -p label:=gazebo -p world:=turtlebot3_world -p out:=/tmp/nav.json

Unlike drive_test this is CLOSED loop, and that is the point of running both:
the controller corrects the actuation error drive_test measures, so the pair
says both how big the plant error is and how much of it survives to the task.

Per goal it records success, time on the simulator's clock, path length against
the straight-line distance, final position and heading error, how many recovery
behaviours fired, and the closest it came to an obstacle. With `bag_dir` set a
rosbag is recorded alongside, so that a metric nobody thought of tonight can be
derived later without re-running the matrix -- the same recorder drive_test
uses, in `bagging.py`.
"""

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from tb3_bringup.bagging import NAV_TOPICS, start_bag, stop_bag

# x, y, yaw, in the map frame.
#
# Chosen per world rather than generated: a goal list that moves with the code
# is not a fixed experiment. Each list ends where it began, so a run is
# repeatable without respawning the robot, and each contains at least one goal
# that is mostly ROTATION -- the regime where Isaac Sim's actuation deficit is
# worst and where a closed loop is most likely to hide it.
GOALS = {
    'turtlebot3_world': [
        ('far_corner', 2.0, 0.5, 0.0),
        ('spin_in_place', 2.0, 0.5, math.pi),
        ('back_home', -2.0, -0.5, 0.0),
    ],
    'small_office': [
        ('across_room', 1.5, 1.5, 0.0),
        ('spin_in_place', 1.5, 1.5, math.pi),
        ('back_home', -1.5, -1.5, 0.0),
    ],
    'empty_stage': [
        ('out', 1.5, 0.0, 0.0),
        ('spin_in_place', 1.5, 0.0, math.pi),
        ('back_home', 0.0, 0.0, 0.0),
    ],
}

# Node names the Nav2 behaviour tree uses for recovery. Counted by name rather
# than by subtree position because the tree is data, not code, and a goal that
# succeeded only after three spins and a back-up is not the same result as one
# that drove straight there.
RECOVERIES = ('Spin', 'BackUp', 'Wait', 'ClearEntireCostmap',
              'ClearCostmapService', 'ClearingActions', 'RecoveryActions',
              'RoundRobin')

STATUS = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_of(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class NavTest(Node):

    def __init__(self):
        super().__init__('nav_test')
        self.declare_parameter('label', 'unknown')
        self.declare_parameter('world', '')
        self.declare_parameter('out', '')
        self.declare_parameter('bag_dir', '')
        self.declare_parameter('timeout', 120.0)
        self.label = self.get_parameter('label').value
        world = self.get_parameter('world').value
        self.out = self.get_parameter('out').value
        self.timeout = self.get_parameter('timeout').value
        if world not in GOALS:
            raise SystemExit('nav_test: world must be one of {}'.format(
                ', '.join(GOALS)))
        self.world = world
        self.goals = GOALS[world]

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, 'odom', self._on_odom,
                                 QoSProfile(depth=20,
                                            reliability=ReliabilityPolicy.RELIABLE))
        # /ground_truth/odom exists on BOTH simulators since 2026-09-18 --
        # Gazebo from a P3D plugin, Isaac Sim from the chassis-prim node that
        # used to feed its /odom -- and on no hardware, because there is no
        # ground truth on a real robot.
        self.create_subscription(Odometry, '/ground_truth/odom',
                                 self._on_truth, 10)
        self.create_subscription(
            LaserScan, 'scan', self._on_scan,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.odom = None
        self.truth = None
        self.path = 0.0
        self.min_range = float('inf')
        self.recoveries = 0
        self._last_xy = None

    # --- channels ---------------------------------------------------------
    def _on_odom(self, msg):
        self.odom = msg
        p = msg.pose.pose.position
        if self._last_xy is not None:
            self.path += math.hypot(p.x - self._last_xy[0], p.y - self._last_xy[1])
        self._last_xy = (p.x, p.y)

    def _on_truth(self, msg):
        self.truth = msg

    def _on_scan(self, msg):
        # The closest the robot came to anything, over the whole goal. inf and
        # nan are what a lidar reports for "no return", not a near miss.
        for r in msg.ranges:
            if msg.range_min < r < self.min_range and not math.isinf(r):
                self.min_range = r

    def _on_feedback(self, _fb):
        pass

    def _on_bt_log(self, msg):
        for event in msg.event_log:
            if event.node_name in RECOVERIES and event.current_status == 'RUNNING':
                self.recoveries += 1

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def adopt_sim_time(self, timeout=180.0):
        """Time goals on the simulator's clock; see drive_test for why."""
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.count_publishers('/clock'):
                self.set_parameters(
                    [Parameter('use_sim_time', Parameter.Type.BOOL, True)])
                self.get_logger().info('/clock found; timing on simulated time')
                return True
            if time.time() - (deadline - timeout) > 5.0:
                break
        self.get_logger().info('no /clock; timing on wall time')
        return False

    def robot_pose(self):
        """Where Nav2 thinks the robot is: map -> base_footprint, as tf."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except Exception:                                    # noqa: BLE001
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_of(t.transform.rotation))

    # --- the run ----------------------------------------------------------
    def wait_for_stack(self, timeout=300.0):
        if not self.client.wait_for_server(timeout_sec=timeout):
            raise SystemExit('nav_test: no navigate_to_pose action server')
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.robot_pose() is not None:
                self.get_logger().info('map -> base_footprint is up')
                return
        raise SystemExit('nav_test: no map -> base_footprint transform')

    def go(self, name, x, y, yaw):
        start = self.robot_pose()
        self.path, self.min_range, self.recoveries = 0.0, float('inf'), 0
        self._last_xy = None

        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = 'map'
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        z, w = quat_of(yaw)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = z, w

        t0 = self._now()
        send = self.client.send_goal_async(msg, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send, timeout_sec=30.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return {'goal': name, 'status': 'REJECTED'}

        result_future = handle.get_result_async()
        wall_stop = time.time() + self.timeout * 20.0 + 60.0
        timed_out = False
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._now() - t0 > self.timeout or time.time() > wall_stop:
                timed_out = True
                handle.cancel_goal_async()
                break
        elapsed = self._now() - t0
        status = 'TIMEOUT' if timed_out else STATUS.get(
            result_future.result().status if result_future.done() else 0,
            'UNKNOWN')

        # Let the robot come to rest before reading the final pose: the action
        # returns while the base is still settling, and a pose sampled mid-stop
        # reads as an error the controller had already removed.
        settle = self._now() + 1.5
        while rclpy.ok() and self._now() < settle:
            rclpy.spin_once(self, timeout_sec=0.05)

        end = self.robot_pose()
        row = {
            'goal': name, 'status': status,
            'target': [round(float(x), 4), round(float(y), 4), round(yaw, 4)],
            'elapsed_s': round(elapsed, 3),
            'recoveries': self.recoveries,
            'odom_path_m': round(self.path, 4),
            'min_obstacle_m': (round(self.min_range, 4)
                               if math.isfinite(self.min_range) else None),
        }
        if end:
            row['final'] = [round(v, 4) for v in end]
            row['goal_err_m'] = round(math.hypot(end[0] - x, end[1] - y), 4)
            row['yaw_err_rad'] = round(abs(angle_diff(end[2], yaw)), 4)
        if start and end:
            straight = math.hypot(end[0] - start[0], end[1] - start[1])
            row['straight_line_m'] = round(straight, 4)
            # How much further than necessary. 1.0 is a perfect straight run;
            # a pure rotation has no straight-line distance and no ratio.
            row['path_ratio'] = (round(self.path / straight, 4)
                                 if straight > 0.05 else None)
        if self.truth is not None:
            p = self.truth.pose.pose
            row['truth'] = [round(p.position.x, 4), round(p.position.y, 4),
                            round(yaw_of(p.orientation), 4)]
        self.get_logger().info(
            '{:14s} {:9s} {:6.2f}s err {} m  yaw {} rad  recoveries {}'.format(
                name, status, row['elapsed_s'], row.get('goal_err_m'),
                row.get('yaw_err_rad'), row['recoveries']))
        return row

    def run(self):
        self.sim_time = self.adopt_sim_time()
        # Subscribed after the clock is settled: the behaviour tree log is the
        # only channel here that is pure bookkeeping, and missing its first
        # messages would undercount recoveries on the first goal.
        try:
            from nav2_msgs.msg import BehaviorTreeLog
            self.create_subscription(BehaviorTreeLog, 'behavior_tree_log',
                                     self._on_bt_log, 10)
        except ImportError:
            self.get_logger().warn('no BehaviorTreeLog; recoveries not counted')
        self.wait_for_stack()
        rows = [self.go(*g) for g in self.goals]
        report = {'label': self.label, 'world': self.world,
                  'sim_time': self.sim_time, 'goals': rows}
        if self.out:
            with open(self.out, 'w') as f:
                json.dump(report, f, indent=1)
            self.get_logger().info('wrote ' + self.out)
        return report


def main(args=None):
    rclpy.init(args=args)
    node = NavTest()
    bag_dir = node.get_parameter('bag_dir').value
    bag = start_bag(bag_dir, NAV_TOPICS)
    try:
        node.run()
    except (KeyboardInterrupt, SystemExit) as e:
        node.get_logger().error('nav_test: {}'.format(e))
    finally:
        stop_bag(bag, bag_dir)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
