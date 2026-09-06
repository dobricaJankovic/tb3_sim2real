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
import os
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument('--stage', default='/scenes/tb3_world.usd')
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
# Matches turtlebot3_gazebo's hls_lfcd_lds sensor exactly, so /scan geometry is
# identical across backends: 360 samples over 360 deg at 5 Hz, 0.12-3.5 m.
SCAN_HZ = 5.0
SCAN_SAMPLES = 360
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

    # Example_Rotary_2D is a 200 m survey lidar out of the box — near range 1.0 m
    # would make the robot blind to everything a TurtleBot3 actually navigates
    # around. Override the scan geometry to the LDS.
    lidar = Lidar.create(
        path=LIDAR_PRIM,
        config='Example_Rotary_2D',
        # Must equal scanRateBaseHz, or the sensor ticks out of step with its
        # own scan and returns partial sweeps.
        tick_rate=SCAN_HZ,
        translations=[list(SCAN_OFFSET)],
        attributes={
            'omni:sensor:Core:nearRangeM': SCAN_RANGE_MIN,
            'omni:sensor:Core:farRangeM': SCAN_RANGE_MAX,
            'omni:sensor:Core:scanRateBaseHz': SCAN_HZ,
            'omni:sensor:Core:patternFiringRateHz': int(SCAN_SAMPLES * SCAN_HZ),
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
