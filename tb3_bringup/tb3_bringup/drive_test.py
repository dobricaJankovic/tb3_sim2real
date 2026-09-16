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

It records the drive chain at three points, so that a discrepancy can be
attributed rather than just noticed:

    command -> wheel   analytic omega (below) vs /joint_states   the actuator
    wheel -> body      /joint_states integrated vs ground truth  slip, contact
    body -> odom       ground truth vs /odom                     the odometry

The analytic omega is the ground truth for the first layer and needs no robot:
the differential-drive kinematics are exact, so a joint that does not reach its
own commanded velocity is broken on any backend. Ground truth for the second
and third is the simulator's own body pose — /gazebo/model_states, and for
Isaac Sim the chassis prim that IsaacComputeOdometry already reads. It is
absent on the real robot, where those two layers collapse into one and are
reported as such rather than faked.
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

# name, linear.x (m/s), angular.z (rad/s), seconds.
#
# Kept inside the burger's real limits (0.22 m/s, 2.84 rad/s) and well inside
# them for the arc, because the interesting differences are in acceleration and
# wheel-slip handling, not in saturation. A settle phase brackets the run so
# odometry drift at rest is separable from drift under motion.
SEQUENCES = {
    'default': [
        ('settle_pre', 0.00, 0.00, 2.0),
        ('straight', 0.15, 0.00, 5.0),
        ('stop_1', 0.00, 0.00, 2.0),
        ('rotate', 0.00, 0.50, 5.0),
        ('stop_2', 0.00, 0.00, 2.0),
        ('arc', 0.10, 0.30, 5.0),
        ('settle_post', 0.00, 0.00, 3.0),
    ],
    # The acceptance matrix: each rate held long enough to reach steady state,
    # bracketed by a stop so one phase cannot bleed into the next. Rotation
    # first and at the LOW rates too -- the deficit was worst at wz = 0.2,
    # which is exactly Nav2's final-alignment regime.
    'sweep': [
        ('settle_pre', 0.00, 0.00, 2.0),
        ('rot_0.2', 0.00, 0.20, 5.0),
        ('stop_r1', 0.00, 0.00, 1.5),
        ('rot_0.5', 0.00, 0.50, 5.0),
        ('stop_r2', 0.00, 0.00, 1.5),
        ('rot_1.0', 0.00, 1.00, 5.0),
        ('stop_r3', 0.00, 0.00, 1.5),
        ('rot_1.5', 0.00, 1.50, 5.0),
        ('stop_r4', 0.00, 0.00, 1.5),
        ('lin_0.10', 0.10, 0.00, 5.0),
        ('stop_l1', 0.00, 0.00, 1.5),
        ('lin_0.15', 0.15, 0.00, 5.0),
        ('stop_l2', 0.00, 0.00, 1.5),
        ('lin_0.22', 0.22, 0.00, 5.0),
        ('settle_post', 0.00, 0.00, 2.0),
    ],
    # Drive into something and keep driving. The question is not whether the
    # robot stops -- both simulators will stop it -- but what the ODOMETRY does
    # once it has: wheels that keep turning against a wall integrate distance
    # the robot never travelled, and that error is unbounded and invisible to
    # Nav2, which is why this is worth measuring rather than assuming.
    'collide': [
        ('settle_pre', 0.00, 0.00, 2.0),
        ('approach', 0.15, 0.00, 25.0),
        ('rest', 0.00, 0.00, 3.0),
        ('reverse', -0.10, 0.00, 3.0),
        ('settle_post', 0.00, 0.00, 3.0),
    ],
}

# The burger, as turtlebot3_gazebo's model.sdf, the URDF and
# turtlebot3_isaacsim's WHEELS table all independently state it. Restated here
# rather than imported so that the instrument's notion of ground truth does not
# move when a backend's does -- if one of them ever disagrees, this test is
# what should report it.
WHEEL_SEPARATION = 0.160   # m
WHEEL_RADIUS = 0.033       # m
WHEEL_JOINTS = ('wheel_left_joint', 'wheel_right_joint')

RATE = 20.0  # Hz, for both publishing and sampling


def wheel_command(lin, ang):
    """Exact differential-drive inverse kinematics: (omega_left, omega_right).

    This is the contract every backend signs. Gazebo tracks it, the real
    OpenCR's encoder PID tracks it, and an actuator that does not is wrong
    independently of what any other backend does -- which is why this is the
    reference rather than a comparison between simulators.
    """
    return ((lin - ang * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS,
            (lin + ang * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DriveTest(Node):

    def __init__(self):
        super().__init__('drive_test')
        self.declare_parameter('label', 'unknown')
        self.declare_parameter('out', '')
        self.declare_parameter('sequence', 'default')
        self.label = self.get_parameter('label').value
        self.out = self.get_parameter('out').value
        name = self.get_parameter('sequence').value
        if name not in SEQUENCES:
            raise SystemExit('drive_test: sequence must be one of {}'.format(
                ', '.join(SEQUENCES)))
        self.sequence_name = name
        self.sequence = SEQUENCES[name]

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Odometry is published RELIABLE by all three backends; ask for the same
        # so a mismatch is an error rather than a silently empty recording.
        self.create_subscription(Odometry, 'odom', self._on_odom,
                                 QoSProfile(depth=50,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(JointState, 'joint_states', self._on_joints, 50)
        # Ground truth for the wheel->body layer, where there is any. The
        # Gazebo backend publishes it from a P3D plugin; Isaac Sim does not
        # need to, because IsaacComputeOdometry reads the chassis prim and its
        # /odom already IS the body pose. Subscribing unconditionally is
        # correct on all three: a backend that does not publish it simply
        # leaves the channel empty, and the report says which happened rather
        # than leaving a reader to assume.
        self.create_subscription(Odometry, '/ground_truth/odom',
                                 self._on_truth, 10)

        self.odom = None
        self.joints = None       # (omega_left, omega_right), rad/s
        self.turns = {}          # joint -> cumulative UNWRAPPED position, rad
        self._last_raw = {}
        self.truth = None        # (x, y, yaw) from the simulator, or None
        self.samples = []

    def _on_odom(self, msg):
        self.odom = msg

    def _on_joints(self, msg):
        vel, pos = {}, {}
        for i, name in enumerate(msg.name):
            if name not in WHEEL_JOINTS:
                continue
            if i < len(msg.velocity):
                vel[name] = msg.velocity[i]
            if i < len(msg.position):
                pos[name] = msg.position[i]
        if len(vel) == len(WHEEL_JOINTS):
            self.joints = tuple(vel[j] for j in WHEEL_JOINTS)
        # A continuous joint's reported angle may be unwrapped and accumulating
        # (Gazebo) or folded into [-pi, pi] (an articulation that stores a
        # quaternion). Differencing the folded one across a wrap reads as most
        # of a turn backwards, which would land in the wheel->body layer as a
        # spectacular fake slip. Accumulate here instead: at 20 Hz the fastest
        # wheel in the matrix moves 0.33 rad per sample, nowhere near the pi
        # that would make this ambiguous.
        for name, raw in pos.items():
            prev = self._last_raw.get(name)
            if prev is None:
                self.turns[name] = 0.0
            else:
                step = raw - prev
                self.turns[name] += math.atan2(math.sin(step), math.cos(step))
            self._last_raw[name] = raw

    def _on_truth(self, msg):
        p = msg.pose.pose
        self.truth = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _pose(self):
        p = self.odom.pose.pose
        t = self.odom.twist.twist
        s = dict(t=self._now(), x=p.position.x, y=p.position.y,
                 yaw=yaw_of(p.orientation), vx=t.linear.x, wz=t.angular.z)
        if self.joints is not None:
            s['wl'], s['wr'] = self.joints
            s['pl'] = self.turns.get(WHEEL_JOINTS[0])
            s['pr'] = self.turns.get(WHEEL_JOINTS[1])
        if self.truth is not None:
            s['gx'], s['gy'], s['gyaw'] = self.truth
        return s

    def adopt_sim_time(self, timeout=180.0):
        """Time phases on the SIMULATOR's clock when there is one.

        `ros2 run` does not set use_sim_time, so without this the phases are
        timed on the wall clock while the robot moves in sim time. Gazebo at a
        real-time factor near 1.0 hides that completely; Isaac Sim with RTX does
        not, and every phase comes out short by exactly the real-time factor —
        which reads as a large kinematic error and is nothing of the kind.

        Detected rather than declared, so the instrument stays one command on
        all three backends: the real robot publishes no /clock and keeps wall
        time, which is correct for it.
        """
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.count_publishers('/clock'):
                self.set_parameters(
                    [Parameter('use_sim_time', Parameter.Type.BOOL, True)])
                self.get_logger().info('/clock found; timing on simulated time')
                return True
            if time.time() > deadline - timeout + 5.0:
                break
        self.get_logger().info('no /clock; timing on wall time')
        return False

    def wait_for_odom(self, timeout=180.0):
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
        self.sim_time = self.adopt_sim_time()
        self.wait_for_odom()
        results = []
        for name, lin, ang, secs in self.sequence:
            msg = Twist()
            msg.linear.x, msg.angular.z = lin, ang
            start = self._pose()
            t0 = start['t']
            phase = [start]
            # Bounded by the wall clock too: a simulator that stalls its own
            # /clock would otherwise hang the run rather than report anything.
            wall_stop = time.time() + secs * 20.0 + 30.0
            while rclpy.ok() and self._now() - t0 < secs and time.time() < wall_stop:
                self.pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=1.0 / RATE)
                phase.append(self._pose())
            end = phase[-1]
            results.append(self._summarise(name, lin, ang, secs, phase))
            self.samples.extend({'phase': name, **s} for s in phase)
            r = results[-1]
            self.get_logger().info(
                '{:12s} cmd=({:.2f},{:.2f}) -> d={:.4f}m dyaw={:.4f}rad'
                '  wheel {:s}'.format(
                    name, lin, ang, r['distance'], r['d_yaw'],
                    'L {:.3f}/{:.3f} R {:.3f}/{:.3f}'.format(
                        r['wheel_wl'], r['cmd_wl'], r['wheel_wr'], r['cmd_wr'])
                    if 'wheel_wl' in r else 'n/a'))

        self.pub.publish(Twist())
        report = {'label': self.label, 'sequence_name': self.sequence_name,
                  'sim_time': self.sim_time,
                  # Which layers this run can actually speak to. A missing
                  # channel has to be visible in the file, or an absent
                  # /joint_states reads later as a backend that reported
                  # nothing wrong.
                  'has_joint_states': self.joints is not None,
                  'ground_truth': '/ground_truth/odom' if self.truth is not None
                                  else 'none (on isaacsim /odom IS the body pose)',
                  'wheel_separation': WHEEL_SEPARATION,
                  'wheel_radius': WHEEL_RADIUS,
                  'sequence': results,
                  'final': self._pose()}
        if self.out:
            with open(self.out, 'w') as f:
                json.dump(report, f, indent=1)
            self.get_logger().info('wrote ' + self.out)
            # The raw trace goes beside it, not in it. The summary is a few KB
            # and worth committing as evidence; the samples are ~600 KB a run
            # and are regenerated by re-running.
            raw = self.out.rsplit('.json', 1)[0] + '.samples.json'
            with open(raw, 'w') as f:
                json.dump({'label': self.label, 'samples': self.samples}, f)
            self.get_logger().info('wrote ' + raw)
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

        # Steady state only: the back half of the phase, so a ramp at the start
        # cannot be read as a tracking error. Gazebo ramps over ~0.1 s and
        # Isaac Sim steps in one sample, so any window that includes the start
        # would compare the two backends' transients and call it a gain.
        steady = phase[len(phase) // 2:] or phase
        cmd_wl, cmd_wr = wheel_command(lin, ang)
        out = {
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

        # --- layer 1: command -> wheel -------------------------------------
        # The analytic omega is the target. Reporting the ABSOLUTE error beside
        # the ratio is the point: a fixed resisting torque against a finite-gain
        # velocity drive gives a constant absolute error, and a units or
        # kinematics error gives a constant ratio. The two are only
        # distinguishable across a range of commanded rates, which is what the
        # 'sweep' sequence is for.
        out['cmd_wl'], out['cmd_wr'] = round(cmd_wl, 5), round(cmd_wr, 5)
        got = [s for s in steady if 'wl' in s]
        if got:
            wl = sum(s['wl'] for s in got) / len(got)
            wr = sum(s['wr'] for s in got) / len(got)
            out['wheel_wl'], out['wheel_wr'] = round(wl, 5), round(wr, 5)
            out['wheel_err_l'] = round(abs(cmd_wl) - abs(wl), 5)
            out['wheel_err_r'] = round(abs(cmd_wr) - abs(wr), 5)
            if abs(cmd_wl) > 1e-9:
                out['wheel_track_l'] = round(abs(wl) / abs(cmd_wl), 5)
            if abs(cmd_wr) > 1e-9:
                out['wheel_track_r'] = round(abs(wr) / abs(cmd_wr), 5)

        # --- layer 2: wheel -> body ----------------------------------------
        # What the wheels turned through, forward-integrated by the same exact
        # kinematics, against what the body actually did. The difference is
        # slip and the contact model, and it is the one layer a real robot
        # cannot report on itself.
        turned = [s for s in phase if s.get('pl') is not None]
        if len(turned) >= 2:
            dpl = turned[-1]['pl'] - turned[0]['pl']
            dpr = turned[-1]['pr'] - turned[0]['pr']
            out['wheel_d'] = round(WHEEL_RADIUS * (dpl + dpr) / 2.0, 5)
            out['wheel_d_yaw'] = round(
                WHEEL_RADIUS * (dpr - dpl) / WHEEL_SEPARATION, 5)

        # --- layer 3: body -> odom -----------------------------------------
        truth = [s for s in phase if 'gx' in s]
        if len(truth) >= 2:
            a2, b2 = truth[0], truth[-1]
            out['truth_d'] = round(math.hypot(b2['gx'] - a2['gx'],
                                              b2['gy'] - a2['gy']), 5)
            out['truth_d_yaw'] = round(
                math.atan2(math.sin(b2['gyaw'] - a2['gyaw']),
                           math.cos(b2['gyaw'] - a2['gyaw'])), 5)
        return out


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
