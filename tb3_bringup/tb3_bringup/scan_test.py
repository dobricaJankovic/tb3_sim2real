"""Compare a parked robot's /scan against the world manifest's own geometry.

The perception row of the four-layer table, and the one measurement in this
repository that is only possible BECAUSE the world registry exists: the
expected range down every beam is computed from `worlds/<name>/world.yaml`,
the same file both simulators were generated from. There is no reference scan
to compare against and no hand-measured room — the ground truth is the source
of truth, which is what makes a disagreement attributable to the sensor model
rather than to the environment.

    ros2 run tb3_bringup scan_test --ros-args \
        -p label:=gazebo -p world:=small_office -p out:=/tmp/scan.json

Static on purpose: the robot is parked at the manifest's spawn and never
commanded, so nothing here is confounded by the actuation deficit that the rest
of tonight is about.

The room is boxes and the props are meshes. Only the boxes are ray-cast, which
is not a limitation but the method: a prop can only ever OCCLUDE a wall, so the
box-only prediction is an upper bound on the true range, and each beam sorts
itself into one of three cases without anyone having to model a chair.

    measured ~= predicted      a wall return   -> the error statistic
    measured <  predicted      a prop return   -> counted, excluded
    measured >  predicted      a LEAK: the beam passed through a wall that the
                               manifest says is there. Always a defect.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from tb3_bringup import worlds

# base_footprint -> base_scan for the burger, from turtlebot3_description:
# base_joint (0, 0, 0.010) composed with scan_joint (-0.032, 0, 0.172).
SCAN_OFFSET = (-0.032, 0.0, 0.182)

# How far a beam may sit from the box prediction and still count as a wall.
# Generous next to the LDS-01's 0.015 m spec, because it only has to separate
# "hit the wall" from "hit a chair", and the nearest prop to the spawn is
# 1.07 m away from a wall that is 1.5 m away.
WALL_TOL = 0.05


def ray_box(ox, oy, dx, dy, cx, cy, sx, sy):
    """Distance from (ox,oy) along (dx,dy) to an axis-aligned box, or None."""
    lo, hi = 0.0, float('inf')
    for o, d, c, s in ((ox, dx, cx, sx), (oy, dy, cy, sy)):
        lo_e, hi_e = c - s / 2.0, c + s / 2.0
        if abs(d) < 1e-12:
            if o < lo_e or o > hi_e:
                return None
            continue
        t1, t2 = (lo_e - o) / d, (hi_e - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        lo, hi = max(lo, t1), min(hi, t2)
        if lo > hi:
            return None
    return lo if lo > 0.0 else None


class ScanTest(Node):

    def __init__(self):
        super().__init__('scan_test')
        self.declare_parameter('label', 'unknown')
        self.declare_parameter('world', '')
        self.declare_parameter('out', '')
        self.declare_parameter('scans', 20)
        self.label = self.get_parameter('label').value
        self.out = self.get_parameter('out').value
        self.n_scans = self.get_parameter('scans').value
        self.world = worlds.World.load(self.get_parameter('world').value)

        self.create_subscription(
            LaserScan, 'scan', self._on_scan,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.scans = []

        # Every box whose vertical extent straddles the scan plane. A body that
        # the beam passes over is not an obstacle to it, and saying so here is
        # cheaper than discovering it as a systematic over-read.
        z = SCAN_OFFSET[2]
        self.boxes = []
        for body in self.world.bodies:
            g = body.get('geometry', {})
            if g.get('type') != 'box':
                continue
            x, y, bz = body.get('xyz', [0, 0, 0])
            sx, sy, sz = g['size']
            if not (bz - sz / 2.0 <= z <= bz + sz / 2.0):
                continue
            rpy = body.get('rpy', [0, 0, 0])
            if abs(math.atan2(math.sin(rpy[2]), math.cos(rpy[2]))) > 1e-6:
                self.get_logger().warn(
                    '{} is a rotated box; not ray-cast'.format(body['name']))
                continue
            self.boxes.append((body['name'], x, y, sx, sy))
        self.get_logger().info('{} boxes cross the scan plane at z={}'.format(
            len(self.boxes), z))

    def _on_scan(self, msg):
        if len(self.scans) < self.n_scans:
            self.scans.append(msg)

    def predict(self, ox, oy, angle):
        dx, dy = math.cos(angle), math.sin(angle)
        best, who = float('inf'), None
        for name, cx, cy, sx, sy in self.boxes:
            t = ray_box(ox, oy, dx, dy, cx, cy, sx, sy)
            if t is not None and t < best:
                best, who = t, name
        return (best, who) if who else (None, None)

    def run(self):
        deadline = time.time() + 180.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.count_publishers('/clock'):
                self.set_parameters(
                    [Parameter('use_sim_time', Parameter.Type.BOOL, True)])
                break
        while rclpy.ok() and len(self.scans) < self.n_scans and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.scans:
            raise SystemExit('scan_test: no /scan')

        # Where the sensor is: the manifest's spawn, plus the URDF's
        # base_footprint -> base_scan offset, rotated by the spawn yaw. The
        # robot is parked, so this is exact rather than estimated, and it
        # deliberately does not go through tf or /odom -- both of which are
        # products of the backend being measured.
        sx, sy, _, yaw = self.world.spawn
        ox = sx + SCAN_OFFSET[0] * math.cos(yaw) - SCAN_OFFSET[1] * math.sin(yaw)
        oy = sy + SCAN_OFFSET[0] * math.sin(yaw) + SCAN_OFFSET[1] * math.cos(yaw)

        ref = self.scans[-1]
        n = len(ref.ranges)
        # Mean over N scans per beam, so that sensor noise is separated from a
        # systematic offset rather than folded into it.
        stacked = [[] for _ in range(n)]
        for msg in self.scans:
            for i, r in enumerate(msg.ranges[:n]):
                if msg.range_min <= r <= msg.range_max and math.isfinite(r):
                    stacked[i].append(r)

        beams, walls, props, leaks, invalid = [], [], [], [], 0
        for i in range(n):
            angle = ref.angle_min + i * ref.angle_increment + yaw
            pred, who = self.predict(ox, oy, angle)
            got = (sum(stacked[i]) / len(stacked[i])) if stacked[i] else None
            spread = (max(stacked[i]) - min(stacked[i])) if len(stacked[i]) > 1 else 0.0
            if got is None:
                invalid += 1
            row = {'i': i, 'angle': round(angle, 5),
                   'predicted': round(pred, 5) if pred else None,
                   'measured': round(got, 5) if got else None,
                   'hits': who, 'spread': round(spread, 5),
                   'returns': len(stacked[i])}
            if got is not None and pred is not None:
                err = got - pred
                row['error'] = round(err, 5)
                if abs(err) <= WALL_TOL:
                    walls.append(err)
                elif err < 0:
                    props.append(got)
                else:
                    leaks.append(row)
            beams.append(row)

        def stats(v):
            if not v:
                return None
            m = sum(v) / len(v)
            sd = math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
            return {'n': len(v), 'mean': round(m, 5), 'sd': round(sd, 5),
                    'max_abs': round(max(abs(x) for x in v), 5)}

        report = {
            'label': self.label, 'world': self.world.name,
            'scans_averaged': len(self.scans), 'beams': n,
            'angle_min': round(ref.angle_min, 5),
            'angle_increment': round(ref.angle_increment, 6),
            'range_min': ref.range_min, 'range_max': ref.range_max,
            'sensor_xy': [round(ox, 4), round(oy, 4)],
            'valid_return_fraction': round(1.0 - invalid / float(n), 4),
            'wall_error_m': stats(walls),
            'prop_returns': len(props),
            'leaks': len(leaks),
            'leak_examples': leaks[:5],
            'noise_spread_mean': round(
                sum(b['spread'] for b in beams) / float(n), 5),
        }
        self.get_logger().info(json.dumps(
            {k: report[k] for k in ('valid_return_fraction', 'wall_error_m',
                                    'prop_returns', 'leaks')}))
        if self.out:
            with open(self.out, 'w') as f:
                json.dump(report, f, indent=1)
            with open(self.out.rsplit('.json', 1)[0] + '.beams.json', 'w') as f:
                json.dump({'label': self.label, 'beams': beams}, f)
            self.get_logger().info('wrote ' + self.out)
        return report


def main(args=None):
    rclpy.init(args=args)
    node = ScanTest()
    try:
        node.run()
    except (KeyboardInterrupt, SystemExit) as e:
        node.get_logger().error(str(e))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
