"""Launch Isaac Sim with the TurtleBot3 scene loaded, playing, and on ROS 2.

This is the Isaac Sim equivalent of what `gazebo.launch.py` does with gzserver
and gzclient: it *is* the simulator launcher. `SimulationApp` boots Kit, the
stage is opened, the ROS 2 interface is built, and `play()` is called from here.
Nothing has to be clicked in the GUI, and `bringup.launch.py backend:=isaacsim`
does not start any of it — that side only attaches to what this publishes.

    # host, once per X session
    ./docker/x11-auth.sh

    # simulator (this script) — leave it running
    docker compose run --rm --entrypoint /isaac-sim/python.sh isaacsim \
        /scripts/tb3_sim.py

    # ROS side, second terminal
    docker compose run --rm tb3_ros
    ros2 launch tb3_bringup bringup.launch.py backend:=isaacsim

Publishes the interface contract in docs/architecture.md: /clock, /odom,
/joint_states, /scan, tf odom->base_footprint, and subscribes /cmd_vel.
Everything below base_footprint comes from robot_state_publisher on the ROS
side, which is why only a *raw* odom->base_footprint transform is published
here — a full ROS2PublishTransformTree would fight it.

Node types and attribute names below are taken from Isaac Sim 6.0's own
reference graph in
`source/extensions/isaacsim.ros2.nodes/python/tests/test_differential_base.py`
(`add_differential_drive`), not guessed.
"""

import argparse
import math
import os
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument('--stage', default='/scenes/tb3_world.usd')
# The environment, by registry name (worlds/<name>/). This is the isaacsim
# counterpart of `bringup.launch.py world:=` — Isaac Sim runs in its own
# container, so the world is chosen here rather than by the ROS-side launch,
# which only attaches over DDS. Defaults to $WORLD, set in docker-compose.yml.
parser.add_argument('--world', default=os.environ.get('WORLD', 'turtlebot3_world'),
                    help='environment from the world registry; "" or --no-world '
                         'for the bare ground plane')
parser.add_argument('--no-world', action='store_true',
                    help='Skip the environment. Useful to tell an environment '
                         'problem apart from a robot or graph problem.')
parser.add_argument('--headless', action='store_true')
parser.add_argument('--no-lidar', action='store_true',
                    help='Skip the RTX lidar so a lidar failure can be isolated '
                         'from a graph failure. /scan will be missing.')
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({'headless': args.headless})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.usd
import usdrt.Sdf
from isaacsim.core.simulation_manager import SimulationManager

ROBOT_PRIM = '/World/turtlebot3'
# The environment is referenced in here rather than baked into tb3_world.usd, so
# changing world never means regenerating the robot stage.
WORLD_PRIM = '/World/env'
WORLDS_ROOT = os.environ.get('TB3_WORLDS_DIR', '/worlds')
# The importer puts PhysicsArticulationRootAPI on base_footprint, not on the
# reference prim above it. IsaacComputeOdometry, IsaacArticulationController and
# ROS2PublishJointState all want the articulation root, so they get this.
ARTICULATION_ROOT = f'{ROBOT_PRIM}/Geometry/base_footprint'
LIDAR_PRIM = f'{ARTICULATION_ROOT}/lidar'
GRAPH_PATH = '/World/ROS2Interface'

WHEEL_JOINTS = ['wheel_left_joint', 'wheel_right_joint']
# Wheel joints sit at y = +/-0.080 in the burger URDF, so separation is 0.160.
WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.160

# base_footprint -> base_scan, composed from the burger URDF's base_joint
# (0, 0, 0.010) and scan_joint (-0.032, 0, 0.172). It must match the URDF and
# not turtlebot3_gazebo's model.sdf (which says 0.171): robot_state_publisher
# owns this transform in every backend, and it reads the URDF.
SCAN_OFFSET = (-0.032, 0.0, 0.182)
# turtlebot3_gazebo's hls_lfcd_lds publishes 360 samples over 360 deg at 5 Hz
# across 0.12-3.5 m. Only the ranges are enforced here — see attach_lidar() for
# why the rates are left to the sensor config.
SCAN_RANGE_MIN = 0.12
SCAN_RANGE_MAX = 3.5


def build_graph() -> None:
    keys = og.Controller.Keys
    og.Controller.edit(
        {'graph_path': GRAPH_PATH, 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: [
                # OnPlaybackTick lives in omni.graph.action, NOT
                # isaacsim.core.nodes.
                ('OnTick', 'omni.graph.action.OnPlaybackTick'),
                ('SimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('PubClock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
                ('ComputeOdom', 'isaacsim.core.nodes.IsaacComputeOdometry'),
                ('PubOdom', 'isaacsim.ros2.bridge.ROS2PublishOdometry'),
                ('PubRawTF', 'isaacsim.ros2.bridge.ROS2PublishRawTransformTree'),
                ('PubJointState', 'isaacsim.ros2.bridge.ROS2PublishJointState'),
                ('SubTwist', 'isaacsim.ros2.bridge.ROS2SubscribeTwist'),
                # ROS2SubscribeTwist emits vectord[3] but DifferentialController
                # takes scalar doubles, so the two cannot be wired directly.
                ('BreakLinVel', 'omni.graph.nodes.BreakVector3'),
                ('BreakAngVel', 'omni.graph.nodes.BreakVector3'),
                ('DiffController', 'isaacsim.robot.wheeled_robots.DifferentialController'),
                ('ArticController', 'isaacsim.core.nodes.IsaacArticulationController'),
            ],
            keys.CONNECT: [
                ('OnTick.outputs:tick', 'PubClock.inputs:execIn'),
                ('SimTime.outputs:simulationTime', 'PubClock.inputs:timeStamp'),

                ('OnTick.outputs:tick', 'ComputeOdom.inputs:execIn'),
                ('OnTick.outputs:tick', 'PubOdom.inputs:execIn'),
                ('OnTick.outputs:tick', 'PubRawTF.inputs:execIn'),
                ('SimTime.outputs:simulationTime', 'PubOdom.inputs:timeStamp'),
                ('SimTime.outputs:simulationTime', 'PubRawTF.inputs:timeStamp'),
                ('ComputeOdom.outputs:position', 'PubOdom.inputs:position'),
                ('ComputeOdom.outputs:orientation', 'PubOdom.inputs:orientation'),
                ('ComputeOdom.outputs:linearVelocity', 'PubOdom.inputs:linearVelocity'),
                ('ComputeOdom.outputs:angularVelocity', 'PubOdom.inputs:angularVelocity'),
                ('ComputeOdom.outputs:position', 'PubRawTF.inputs:translation'),
                ('ComputeOdom.outputs:orientation', 'PubRawTF.inputs:rotation'),

                ('OnTick.outputs:tick', 'PubJointState.inputs:execIn'),
                ('SimTime.outputs:simulationTime', 'PubJointState.inputs:timeStamp'),

                ('OnTick.outputs:tick', 'SubTwist.inputs:execIn'),
                ('OnTick.outputs:tick', 'ArticController.inputs:execIn'),
                ('SubTwist.outputs:execOut', 'DiffController.inputs:execIn'),
                ('SubTwist.outputs:linearVelocity', 'BreakLinVel.inputs:tuple'),
                ('BreakLinVel.outputs:x', 'DiffController.inputs:linearVelocity'),
                ('SubTwist.outputs:angularVelocity', 'BreakAngVel.inputs:tuple'),
                ('BreakAngVel.outputs:z', 'DiffController.inputs:angularVelocity'),
                ('DiffController.outputs:velocityCommand', 'ArticController.inputs:velocityCommand'),
            ],
            keys.SET_VALUES: [
                ('PubClock.inputs:topicName', '/clock'),

                ('ComputeOdom.inputs:chassisPrim', [usdrt.Sdf.Path(ARTICULATION_ROOT)]),
                ('PubOdom.inputs:topicName', '/odom'),
                ('PubOdom.inputs:odomFrameId', 'odom'),
                ('PubOdom.inputs:chassisFrameId', 'base_footprint'),

                ('PubRawTF.inputs:topicName', '/tf'),
                ('PubRawTF.inputs:parentFrameId', 'odom'),
                ('PubRawTF.inputs:childFrameId', 'base_footprint'),

                ('PubJointState.inputs:topicName', '/joint_states'),
                ('PubJointState.inputs:targetPrim', [usdrt.Sdf.Path(ARTICULATION_ROOT)]),

                ('SubTwist.inputs:topicName', '/cmd_vel'),
                ('DiffController.inputs:wheelRadius', WHEEL_RADIUS),
                ('DiffController.inputs:wheelDistance', WHEEL_BASE),
                ('ArticController.inputs:targetPrim', [usdrt.Sdf.Path(ARTICULATION_ROOT)]),
                ('ArticController.inputs:jointNames', WHEEL_JOINTS),
            ],
        },
    )


def attach_lidar():
    """RTX 2D lidar -> /scan on frame base_scan. Returns the sensor.

    The caller must keep the returned LidarSensor alive: it owns the render
    product the writer renders from, and letting it go out of scope tears that
    down. The symptom is silent — the writer attaches without complaint and
    /scan simply never appears.

    Uses the LaserScan writer rather than PointCloud2 on purpose: a point cloud
    would force a pointcloud_to_laserscan node into the isaacsim backend that
    the other two backends do not have.
    """
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

    # Example_Rotary_2D is a 200 m survey lidar out of the box, and a 1.0 m near
    # range would blind the robot to everything a TurtleBot3 navigates around,
    # so the ranges must be overridden. The *rates* deliberately are not.
    #
    # Forcing scanRateBaseHz to the LDS's 5 Hz (with patternFiringRateHz to
    # match, for 360 samples/rev) does produce the right advertised geometry,
    # but the scan pattern baked into the config still assumes its own 30 Hz /
    # 32000 Hz. The plugin then warns "Multi-tick is enabled but motion BVH is
    # not active. This is not supported." and /scan publishes erratically —
    # a burst, then nothing — instead of steadily. Leaving the rates alone and
    # letting tick_rate default to the asset's own value keeps the sensor
    # self-consistent.
    #
    # Cost: ~0.34 deg/sample instead of the LDS's 1.0, i.e. a denser scan than
    # the real robot. Harmless for Nav2, but it is a real sim2real gap — closing
    # it means authoring an LDS scan-pattern config rather than re-rating this
    # one. Tracked in docs/status.md.
    lidar = Lidar.create(
        path=LIDAR_PRIM,
        config='Example_Rotary_2D',
        translations=[list(SCAN_OFFSET)],
        attributes={
            'omni:sensor:Core:nearRangeM': SCAN_RANGE_MIN,
            'omni:sensor:Core:farRangeM': SCAN_RANGE_MAX,
        },
    )

    prim = prim_utils.get_prim_at_path(lidar.paths[0])
    rotation_rate = float(prim.GetAttribute('omni:sensor:Core:scanRateBaseHz').Get() or 0)
    firing_rate = int(prim.GetAttribute('omni:sensor:Core:patternFiringRateHz').Get() or 0)
    near = float(prim.GetAttribute('omni:sensor:Core:nearRangeM').Get() or 0)
    far = float(prim.GetAttribute('omni:sensor:Core:farRangeM').Get() or 0)
    if rotation_rate <= 0 or firing_rate <= 0:
        raise RuntimeError('lidar prim has a zero scan or firing rate')

    # The LaserScan writer does not read these off the prim itself the way the
    # OgnROS2RtxLidarHelper node does, so they have to be passed explicitly.
    # Reading them back from the prim rather than reusing the constants above is
    # deliberate: if an override above did not take, the numbers printed here
    # say so instead of quietly disagreeing with the published scan.
    sensor = LidarSensor(lidar, annotators=[])
    sensor.attach_writer(
        'RtxLidarROS2PublishLaserScan',
        topicName='scan',
        frameId='base_scan',
        horizontalFov=360.0,
        horizontalResolution=360.0 * rotation_rate / firing_rate,
        depthRange=[near, far],
        rotationRate=rotation_rate,
        azimuthRange=[-180.0, 180.0],
    )
    print(f'lidar: {rotation_rate} Hz, {firing_rate} Hz firing -> '
          f'{360.0 * rotation_rate / firing_rate:.3f} deg/sample, '
          f'range {near}-{far} m', flush=True)
    return sensor


def set_pose(prim_path: str, xyz, yaw: float) -> None:
    """Place a prim, coping with xformOps a reference already authored.

    XformCommonAPI cannot author a rotateXYZ over an `orient` op — and it says
    so by returning False, not by raising, so an unchecked call leaves the robot
    at the origin and the run merely looks odd. The robot prim is a reference,
    so it does carry the asset's own ops; fall back to writing them directly.
    """
    from pxr import Gf, UsdGeom

    prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
    xf = UsdGeom.XformCommonAPI(prim)
    deg = math.degrees(yaw)
    if (xf.SetTranslate(Gf.Vec3d(*[float(v) for v in xyz]))
            and xf.SetRotate(Gf.Vec3f(0.0, 0.0, deg),
                             UsdGeom.XformCommonAPI.RotationOrderXYZ)):
        return

    ops = {op.GetOpName(): op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}
    translate = ops.get('xformOp:translate')
    if translate is None:
        raise RuntimeError(f'cannot place {prim_path}: no translate op and '
                           f'XformCommonAPI refused ({list(ops)})')
    translate.Set(Gf.Vec3d(*[float(v) for v in xyz]))
    orient = ops.get('xformOp:orient')
    if orient is not None:
        half = yaw / 2.0
        q = Gf.Quatd(math.cos(half), Gf.Vec3d(0.0, 0.0, math.sin(half)))
        orient.Set(Gf.Quatf(q) if orient.GetTypeName() == 'quatf' else q)
    elif yaw:
        raise RuntimeError(f'cannot rotate {prim_path}: no orient op ({list(ops)})')


def load_world(name: str) -> None:
    """Reference the registry's generated stage in, and spawn the robot in it.

    The environment is a separate USD referenced at WORLD_PRIM rather than baked
    into tb3_world.usd, so switching worlds never regenerates the robot stage.
    Both come from worlds/<name>/, the same directory Gazebo reads.
    """
    import yaml
    from isaacsim.core.utils.stage import add_reference_to_stage

    world_dir = os.path.join(WORLDS_ROOT, name)
    usd = os.path.join(world_dir, 'isaac', f'{name}.usd')
    manifest_path = os.path.join(world_dir, 'world.yaml')

    if not os.path.isfile(manifest_path):
        avail = sorted(n for n in os.listdir(WORLDS_ROOT)
                       if os.path.isfile(os.path.join(WORLDS_ROOT, n, 'world.yaml'))
                       ) if os.path.isdir(WORLDS_ROOT) else []
        raise RuntimeError(
            f"unknown world '{name}': no {manifest_path}\n"
            f'  registry: {WORLDS_ROOT} (available: {", ".join(avail) or "(none)"})\n'
            f'  If that is empty, the worlds volume is not mounted.')
    if not os.path.isfile(usd):
        raise RuntimeError(
            f"world '{name}' has no generated stage at {usd}\n"
            f'  Build it first, from the host:  scripts/build_world_usd.sh {name}\n'
            f'  (The .usd is gitignored, so a fresh clone never has one.)')

    add_reference_to_stage(usd_path=usd, prim_path=WORLD_PRIM)
    simulation_app.update()

    with open(manifest_path) as f:
        manifest = yaml.safe_load(f)
    spawn = manifest.get('spawn') or {}
    xyz = list(spawn.get('xyz', [0.0, 0.0, 0.0]))
    xyz += [0.0] * (3 - len(xyz))
    yaw = float(spawn.get('yaw', 0.0))

    # Same spawn pose Gazebo uses, from the same manifest. Without this the
    # robot starts at the origin, which in turtlebot3_world is inside a pillar.
    set_pose(ROBOT_PRIM, xyz, yaw)
    print(f'world: {name} at {WORLD_PRIM}, robot spawned at '
          f'{[round(v, 3) for v in xyz]} yaw {yaw:g}', flush=True)


def main() -> None:
    app_utils.enable_extension('isaacsim.ros2.bridge')
    simulation_app.update()

    omni.usd.get_context().open_stage(args.stage)
    simulation_app.update()
    while stage_utils.is_stage_loading():
        simulation_app.update()

    if not prim_utils.get_prim_at_path(ARTICULATION_ROOT).IsValid():
        raise RuntimeError(f'{ARTICULATION_ROOT} missing from {args.stage} — '
                           'run import_tb3.py, then verify_asset.py')

    if args.world and not args.no_world:
        load_world(args.world)
    else:
        print('world: none (bare ground plane)', flush=True)

    build_graph()
    # Bound to a name for its lifetime, not discarded — see attach_lidar().
    lidar_sensor = None
    if not args.no_lidar:
        lidar_sensor = attach_lidar()

    SimulationManager.setup_simulation(dt=1.0 / 60.0, device='cpu')
    simulation_app.update()

    # The bridge publishes nothing while stopped, so this is the line that makes
    # the whole thing live. bringup.launch.py's wait_for_sim is waiting on it.
    app_utils.play()
    simulation_app.update()

    print(f'PLAYING. /clock /odom /joint_states /scan up, /cmd_vel subscribed. '
          f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "unset")}', flush=True)

    while simulation_app.is_running():
        simulation_app.update()

    app_utils.stop()


if __name__ == '__main__':
    try:
        main()
    finally:
        sys.stdout.flush()
        # Same TaskGroup teardown race as import_tb3.py — a graceful
        # simulation_app.close() aborts the process on the way out.
        os._exit(0)
