"""Build the TurtleBot3 ROS 2 interface inside an Isaac Sim stage.

Scripted rather than clicked together in the GUI, so the interface contract is
reproducible and reviewable in git.

Run inside the Isaac Sim container:
    /isaac-sim/python.sh /scripts/build_scene.py --stage /scenes/tb3_world.usd

STATUS: skeleton. Node types and attribute names below follow the Isaac Sim 6.0
`isaacsim.ros2.bridge` / `isaacsim.core.nodes` namespaces but have NOT been run
yet — expect to fix names against the actual extension on first execution.

The contract this must satisfy (must match the real robot exactly):

    /clock                       ROS2PublishClock
    /scan                        ROS2PublishLaserScan     frame_id: base_scan
    /odom                        ROS2PublishOdometry      frame odom, child base_footprint
    /joint_states                ROS2PublishJointState
    tf odom -> base_footprint    ROS2PublishRawTransformTree   (raw only!)
    /cmd_vel                     ROS2SubscribeTwist -> DifferentialController

Everything below base_footprint comes from robot_state_publisher on the ROS
side, so do NOT add a full ROS2PublishTransformTree here — it would fight it.
"""

import argparse

# ---- TODO: fill these in once the TB3 USD exists -----------------------------
# There is no TurtleBot3 asset shipped with Isaac Sim; import the URDF with the
# URDF importer first (see skills/urdf-mjcf-to-usd-conversion) and save it under
# isaac/scenes/. Then set these to the real prim paths.
ROBOT_PRIM = '/World/turtlebot3'
CHASSIS_PRIM = f'{ROBOT_PRIM}/base_footprint'
LIDAR_PRIM = f'{ROBOT_PRIM}/base_scan/lidar'
WHEEL_JOINTS = ['wheel_left_joint', 'wheel_right_joint']

# Frame set the other two backends produce, from turtlebot3_description
# expanded with an empty namespace. Isaac Sim must match these exactly —
# a single wrong character leaves Nav2's costmap silently empty.
#   base_footprint, base_link, base_scan, caster_back_link,
#   imu_link, wheel_left_link, wheel_right_link

# Verified against the burger URDF: wheel joints sit at y = +/-0.080,
# so separation is 0.160. Radius 0.033 is the stock burger wheel.
WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.160

GRAPH_PATH = '/World/ROS2Interface'


def build_graph():
    import omni.graph.core as og

    keys = og.Controller.Keys
    og.Controller.edit(
        {'graph_path': GRAPH_PATH, 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: [
                ('OnTick', 'isaacsim.core.nodes.OnPlaybackTick'),
                ('SimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),

                ('PubClock', 'isaacsim.ros2.bridge.ROS2PublishClock'),

                ('ComputeOdom', 'isaacsim.core.nodes.IsaacComputeOdometry'),
                ('PubOdom', 'isaacsim.ros2.bridge.ROS2PublishOdometry'),
                ('PubRawTF', 'isaacsim.ros2.bridge.ROS2PublishRawTransformTree'),

                ('PubJointState', 'isaacsim.ros2.bridge.ROS2PublishJointState'),

                ('SubTwist', 'isaacsim.ros2.bridge.ROS2SubscribeTwist'),
                ('DiffController', 'isaacsim.robot.wheeled_robots.DifferentialController'),
                ('ArticController', 'isaacsim.core.nodes.IsaacArticulationController'),
            ],

            keys.CONNECT: [
                ('OnTick.outputs:tick', 'PubClock.inputs:execIn'),
                ('SimTime.outputs:simulationTime', 'PubClock.inputs:timeStamp'),

                ('OnTick.outputs:tick', 'ComputeOdom.inputs:execIn'),
                ('ComputeOdom.outputs:execOut', 'PubOdom.inputs:execIn'),
                ('ComputeOdom.outputs:linearVelocity', 'PubOdom.inputs:linearVelocity'),
                ('ComputeOdom.outputs:angularVelocity', 'PubOdom.inputs:angularVelocity'),
                ('ComputeOdom.outputs:position', 'PubOdom.inputs:position'),
                ('ComputeOdom.outputs:orientation', 'PubOdom.inputs:orientation'),
                ('SimTime.outputs:simulationTime', 'PubOdom.inputs:timeStamp'),

                ('ComputeOdom.outputs:execOut', 'PubRawTF.inputs:execIn'),
                ('ComputeOdom.outputs:position', 'PubRawTF.inputs:translation'),
                ('ComputeOdom.outputs:orientation', 'PubRawTF.inputs:rotation'),
                ('SimTime.outputs:simulationTime', 'PubRawTF.inputs:timeStamp'),

                ('OnTick.outputs:tick', 'PubJointState.inputs:execIn'),
                ('SimTime.outputs:simulationTime', 'PubJointState.inputs:timeStamp'),

                ('OnTick.outputs:tick', 'SubTwist.inputs:execIn'),
                ('SubTwist.outputs:execOut', 'DiffController.inputs:execIn'),
                ('SubTwist.outputs:linearVelocity', 'DiffController.inputs:linearVelocity'),
                ('SubTwist.outputs:angularVelocity', 'DiffController.inputs:angularVelocity'),
                ('DiffController.outputs:velocityCommand', 'ArticController.inputs:velocityCommand'),
                ('SubTwist.outputs:execOut', 'ArticController.inputs:execIn'),
            ],

            keys.SET_VALUES: [
                ('PubClock.inputs:topicName', '/clock'),

                ('ComputeOdom.inputs:chassisPrim', [CHASSIS_PRIM]),
                ('PubOdom.inputs:topicName', '/odom'),
                ('PubOdom.inputs:odomFrameId', 'odom'),
                ('PubOdom.inputs:chassisFrameId', 'base_footprint'),

                # Raw odom->base_footprint ONLY. robot_state_publisher owns the rest.
                ('PubRawTF.inputs:parentFrameId', 'odom'),
                ('PubRawTF.inputs:childFrameId', 'base_footprint'),
                ('PubRawTF.inputs:topicName', '/tf'),

                ('PubJointState.inputs:topicName', '/joint_states'),
                ('PubJointState.inputs:targetPrim', [ROBOT_PRIM]),

                ('SubTwist.inputs:topicName', '/cmd_vel'),
                ('DiffController.inputs:wheelRadius', WHEEL_RADIUS),
                ('DiffController.inputs:wheelDistance', WHEEL_BASE),
                ('ArticController.inputs:targetPrim', [ROBOT_PRIM]),
                ('ArticController.inputs:jointNames', WHEEL_JOINTS),
            ],
        },
    )


def build_lidar():
    """RTX lidar -> /scan.

    TODO: use the LaserScan writer, not PointCloud2. A point cloud would force
    a pointcloud_to_laserscan node into the isaacsim backend that the other two
    backends do not have, and the scan geometry would not match the LDS-02.
    Match angle_min/angle_max/range_max/sample count to the physical unit.
    """
    raise NotImplementedError('attach RTX lidar + LaserScan writer at ' + LIDAR_PRIM)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage', required=True, help='USD stage to open')
    parser.add_argument('--save', default='', help='write the modified stage back here')
    args = parser.parse_args()

    from isaacsim import SimulationApp
    sim_app = SimulationApp({'headless': False})

    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension('isaacsim.ros2.bridge')
    sim_app.update()

    omni.usd.get_context().open_stage(args.stage)
    sim_app.update()

    build_graph()
    # build_lidar()

    if args.save:
        omni.usd.get_context().save_as_stage(args.save)

    print('ROS 2 interface graph built. Press PLAY — the bridge publishes nothing until then.')
    while sim_app.is_running():
        sim_app.update()
    sim_app.close()


if __name__ == '__main__':
    main()
