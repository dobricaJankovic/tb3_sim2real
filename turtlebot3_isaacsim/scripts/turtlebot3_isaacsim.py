#!/usr/bin/env python3
#
# Copyright 2026 dobricaJankovic
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The simulator itself. Isaac Sim's answer to gzserver + model.sdf's plugins.

Started by `launch/isaacsim.launch.py` through NVIDIA's isaacsim_bringup, never
by hand in the normal workflow:

    ros2 launch turtlebot3_isaacsim turtlebot3_world.launch.py

Runs on **Kit's own Python 3.12**, with the system ROS 2 stripped off
PYTHONPATH/LD_LIBRARY_PATH before the process starts. Two consequences shape
this whole file:

  * `import rclpy`, `ament_index_python`, `yaml` and friends are NOT available.
    Every path this script needs arrives as an absolute command-line argument
    from the launch file, which does have ament. This is the same split NVIDIA
    uses for isaacsim_bringup's own scripts/open_isaacsim_stage.py.
  * Defaults below are resolved relative to __file__ instead, so the script is
    still runnable straight out of the package share for debugging.

What it publishes is the interface contract the real robot presents, so that
turtlebot3_navigation2 and turtlebot3_cartographer cannot tell the difference:

    /clock          ROS2PublishClock            <- gazebo_ros_init
    /odom           ROS2PublishOdometry         <- gazebo_ros_diff_drive
    tf odom->base_footprint  ROS2PublishRawTransformTree   <- ditto
    /joint_states   ROS2PublishJointState       <- gazebo_ros_joint_state_publisher
    /scan           RtxLidarROS2PublishLaserScan<- gazebo_ros_ray_sensor
    /cmd_vel        ROS2SubscribeTwist          <- gazebo_ros_diff_drive

Only a *raw* odom->base_footprint transform is published. Everything below
base_footprint is robot_state_publisher's, exactly as on the real robot and in
Gazebo -- publishing a full transform tree from here would fight it.

Node types and attribute names are taken from Isaac Sim 6.0's own reference
graph in isaacsim.ros2.nodes' tests (test_differential_base.py,
`add_differential_drive`), not from prose documentation.
"""

import argparse
import json
import math
import os
import sys
import traceback

SHARE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Wheel geometry per model. Same numbers, and the same duplication, as
# turtlebot3_gazebo's models/turtlebot3_<model>/model.sdf <wheel_separation> and
# <wheel_diameter>. The URDF remains the source of truth (burger wheel joints
# sit at y = +/-0.080, hence 0.160); this table is the simulator's copy of it,
# just as the SDF is Gazebo's.
WHEELS = {
    'burger':    {'separation': 0.160, 'radius': 0.033},
    'waffle':    {'separation': 0.287, 'radius': 0.033},
    'waffle_pi': {'separation': 0.287, 'radius': 0.033},
}

# base_footprint -> base_scan, composed from the URDF's base_joint (0, 0, 0.010)
# and scan_joint. Must match turtlebot3_description and NOT turtlebot3_gazebo's
# model.sdf, which says 0.171 where the URDF says 0.172: robot_state_publisher
# owns this transform in every backend and it reads the URDF.
SCAN_OFFSET = {
    'burger':    (-0.032, 0.0, 0.182),
    'waffle':    (-0.064, 0.0, 0.122),
    'waffle_pi': (-0.064, 0.0, 0.122),
}


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    p.add_argument('--model', default=model, choices=sorted(WHEELS))
    p.add_argument('--robot', default='',
                   help='robot asset .usd (default: models/turtlebot3_<model>/'
                        'turtlebot3_<model>.usd in this package)')
    p.add_argument('--world', default='',
                   help='environment .usd. Empty means a bare ground plane, '
                        'which is what empty_world.launch.py wants.')
    p.add_argument('--x-pose', type=float, default=0.0)
    p.add_argument('--y-pose', type=float, default=0.0)
    p.add_argument('--z-pose', type=float, default=0.01,
                   help='matches spawn_turtlebot3.launch.py -z 0.01')
    p.add_argument('--yaw', type=float, default=0.0)
    p.add_argument('--headless', action='store_true')
    p.add_argument('--no-lidar', action='store_true',
                   help='skip /scan, to tell a lidar problem apart from a '
                        'graph problem')
    p.add_argument('--lidar-config', default='turtlebot3_lds',
                   help='RTX lidar profile name, resolved out of '
                        'models/lidar_configs/ in this package')
    p.add_argument('--physics-hz', type=float, default=60.0)
    p.add_argument('--namespace', default='',
                   help='topic/frame prefix, for multi-robot. Empty for one '
                        'robot, as in turtlebot3_gazebo.')
    args, _ = p.parse_known_args()

    if not args.robot:
        args.robot = os.path.join(
            SHARE, 'models', f'turtlebot3_{args.model}',
            f'turtlebot3_{args.model}.usd')
    return args


args = parse_args()

# Everything above this line must not import Kit. SimulationApp boots the
# runtime and only then do isaacsim.* imports resolve.
from isaacsim import SimulationApp                                   # noqa: E402

simulation_app = SimulationApp({'headless': args.headless})

import carb                                                          # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils             # noqa: E402
import isaacsim.core.experimental.utils.prim as prim_utils           # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils         # noqa: E402
import omni.graph.core as og                                         # noqa: E402
import omni.usd                                                      # noqa: E402
import usdrt.Sdf                                                     # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager       # noqa: E402

ROBOT_PRIM = '/World/turtlebot3'
WORLD_PRIM = '/World/env'
GRAPH_PATH = '/World/ROS2Interface'
WHEEL_JOINTS = ['wheel_left_joint', 'wheel_right_joint']


def ns(name):
    """Prefix a topic or frame with the namespace, if there is one."""
    if not args.namespace:
        return name
    prefix = args.namespace.strip('/')
    return f'{prefix}/{name.lstrip("/")}' if name.startswith('/') else \
        f'{prefix}/{name}'


def find_articulation_root(root_path):
    """The prim carrying PhysicsArticulationRootAPI under `root_path`.

    Searched rather than hard-coded. The URDF importer has moved this between
    releases -- it currently lands on `<robot>/Geometry/base_footprint` rather
    than on the reference prim above it -- and IsaacComputeOdometry,
    IsaacArticulationController and ROS2PublishJointState all need the real one.
    A hard-coded path that goes stale produces a stage that loads and plays with
    an inert robot and no error, so resolve it and fail loudly instead.
    """
    from pxr import Usd, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        raise RuntimeError(f'{root_path} is not on the stage')
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    raise RuntimeError(
        f'no articulation root under {root_path}.\n'
        f'  The robot asset imported without an articulation, so nothing can '
        f'drive its joints.\n'
        f'  Rebuild it:  isaacsim-python '
        f'{os.path.join(SHARE, "scripts", "import_turtlebot3.py")} '
        f'--model {args.model}')


def build_graph(articulation_root):
    """The OmniGraph that is this package's model.sdf <plugin> block.

    One graph, ticked by OnPlaybackTick, holding every ROS 2 node. Nothing here
    publishes while the timeline is stopped, which is why main() calls play().
    """
    wheels = WHEELS[args.model]
    keys = og.Controller.Keys
    og.Controller.edit(
        {'graph_path': GRAPH_PATH, 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: [
                # OnPlaybackTick lives in omni.graph.action, NOT in
                # isaacsim.core.nodes.
                ('OnTick', 'omni.graph.action.OnPlaybackTick'),
                ('SimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('PubClock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
                ('ComputeOdom', 'isaacsim.core.nodes.IsaacComputeOdometry'),
                ('PubOdom', 'isaacsim.ros2.bridge.ROS2PublishOdometry'),
                ('PubRawTF', 'isaacsim.ros2.bridge.ROS2PublishRawTransformTree'),
                ('PubJointState', 'isaacsim.ros2.bridge.ROS2PublishJointState'),
                ('SubTwist', 'isaacsim.ros2.bridge.ROS2SubscribeTwist'),
                # ROS2SubscribeTwist emits vectord[3]; DifferentialController
                # takes scalar doubles. The two cannot be wired directly.
                ('BreakLinVel', 'omni.graph.nodes.BreakVector3'),
                ('BreakAngVel', 'omni.graph.nodes.BreakVector3'),
                ('DiffController',
                 'isaacsim.robot.wheeled_robots.DifferentialController'),
                ('ArticController',
                 'isaacsim.core.nodes.IsaacArticulationController'),
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
                ('ComputeOdom.outputs:linearVelocity',
                 'PubOdom.inputs:linearVelocity'),
                ('ComputeOdom.outputs:angularVelocity',
                 'PubOdom.inputs:angularVelocity'),
                ('ComputeOdom.outputs:position', 'PubRawTF.inputs:translation'),
                ('ComputeOdom.outputs:orientation', 'PubRawTF.inputs:rotation'),

                ('OnTick.outputs:tick', 'PubJointState.inputs:execIn'),
                ('SimTime.outputs:simulationTime',
                 'PubJointState.inputs:timeStamp'),

                ('OnTick.outputs:tick', 'SubTwist.inputs:execIn'),
                ('OnTick.outputs:tick', 'ArticController.inputs:execIn'),
                ('SubTwist.outputs:execOut', 'DiffController.inputs:execIn'),
                ('SubTwist.outputs:linearVelocity', 'BreakLinVel.inputs:tuple'),
                ('BreakLinVel.outputs:x', 'DiffController.inputs:linearVelocity'),
                ('SubTwist.outputs:angularVelocity', 'BreakAngVel.inputs:tuple'),
                ('BreakAngVel.outputs:z', 'DiffController.inputs:angularVelocity'),
                ('DiffController.outputs:velocityCommand',
                 'ArticController.inputs:velocityCommand'),
            ],
            keys.SET_VALUES: [
                ('PubClock.inputs:topicName', '/clock'),

                ('ComputeOdom.inputs:chassisPrim',
                 [usdrt.Sdf.Path(articulation_root)]),
                ('PubOdom.inputs:topicName', ns('/odom')),
                ('PubOdom.inputs:odomFrameId', ns('odom')),
                ('PubOdom.inputs:chassisFrameId', ns('base_footprint')),

                ('PubRawTF.inputs:topicName', '/tf'),
                ('PubRawTF.inputs:parentFrameId', ns('odom')),
                ('PubRawTF.inputs:childFrameId', ns('base_footprint')),

                ('PubJointState.inputs:topicName', ns('/joint_states')),
                ('PubJointState.inputs:targetPrim',
                 [usdrt.Sdf.Path(articulation_root)]),

                ('SubTwist.inputs:topicName', ns('/cmd_vel')),
                ('DiffController.inputs:wheelRadius', wheels['radius']),
                ('DiffController.inputs:wheelDistance', wheels['separation']),
                ('ArticController.inputs:targetPrim',
                 [usdrt.Sdf.Path(articulation_root)]),
                ('ArticController.inputs:jointNames', WHEEL_JOINTS),
            ],
        },
    )
    print(f'graph: {GRAPH_PATH} on {articulation_root} '
          f'(wheel r={wheels["radius"]} d={wheels["separation"]})', flush=True)


def register_lidar_configs():
    """Put this package's lidar_configs/ on the RTX profile search path.

    `app.sensors.nv.lidar.profileBaseFolder` is a settings *list* the renderer
    walks to resolve a profile by name, seeded by isaacsim.sensors.rtx with its
    own vendor folders. Appending ours is what lets the package ship its own
    sensor model -- the counterpart of turtlebot3_gazebo owning its <ray> block
    rather than borrowing a vendor sensor.
    """
    folder = os.path.join(SHARE, 'models', 'lidar_configs') + os.sep
    settings = carb.settings.get_settings()
    key = 'app/sensors/nv/lidar/profileBaseFolder'
    existing = list(settings.get(key) or [])
    if folder not in existing:
        settings.set(key, existing + [folder])
    return folder


def attach_lidar():
    """RTX 2D lidar -> /scan on frame base_scan. Returns the sensor.

    The caller MUST keep the returned LidarSensor alive: it owns the render
    product the writer renders from, and letting it fall out of scope tears that
    down. The symptom is silent -- the writer attaches without complaint and
    /scan simply never appears.

    The LaserScan writer is used rather than a point cloud on purpose. A point
    cloud would force a pointcloud_to_laserscan node into every launch file that
    uses this package, which neither the real robot nor Gazebo needs.
    """
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

    folder = register_lidar_configs()
    config_path = os.path.join(folder, f'{args.lidar_config}.json')
    if not os.path.isfile(config_path):
        raise RuntimeError(
            f'no lidar profile {args.lidar_config!r} in {folder}\n'
            f'  available: '
            f'{", ".join(sorted(f[:-5] for f in os.listdir(folder) if f.endswith(".json"))) or "(none)"}')

    lidar = Lidar.create(
        path=f'{find_articulation_root(ROBOT_PRIM)}/lidar',
        config=args.lidar_config,
        translations=[list(SCAN_OFFSET[args.model])],
    )

    # Read the rates back off the prim rather than trusting the JSON: if the
    # profile failed to resolve, the renderer falls back to a default and these
    # numbers say so instead of quietly disagreeing with the published scan.
    prim = prim_utils.get_prim_at_path(lidar.paths[0])
    rotation_rate = float(prim.GetAttribute('omni:sensor:Core:scanRateBaseHz').Get() or 0)
    firing_rate = int(prim.GetAttribute('omni:sensor:Core:patternFiringRateHz').Get() or 0)
    near = float(prim.GetAttribute('omni:sensor:Core:nearRangeM').Get() or 0)
    far = float(prim.GetAttribute('omni:sensor:Core:farRangeM').Get() or 0)
    if rotation_rate <= 0 or firing_rate <= 0:
        raise RuntimeError('lidar prim has a zero scan or firing rate')

    with open(config_path) as f:
        expected = json.load(f)['profile']
    if abs(rotation_rate - float(expected['scanRateBaseHz'])) > 1e-6:
        raise RuntimeError(
            f'lidar resolved to a different profile than {args.lidar_config}: '
            f'{rotation_rate} Hz on the prim vs '
            f'{expected["scanRateBaseHz"]} Hz in {config_path}')

    # The writer does not read these off the prim the way OgnROS2RtxLidarHelper
    # does, so they have to be passed explicitly.
    sensor = LidarSensor(lidar, annotators=[])
    sensor.attach_writer(
        'RtxLidarROS2PublishLaserScan',
        topicName=ns('scan'),
        frameId=ns('base_scan'),
        horizontalFov=360.0,
        horizontalResolution=360.0 * rotation_rate / firing_rate,
        depthRange=[near, far],
        rotationRate=rotation_rate,
        azimuthRange=[-180.0, 180.0],
    )
    print(f'lidar: {args.lidar_config} -- {rotation_rate} Hz, {firing_rate} Hz '
          f'firing -> {360.0 * rotation_rate / firing_rate:.3f} deg/sample, '
          f'range {near}-{far} m', flush=True)
    return sensor


def set_pose(prim_path, xyz, yaw):
    """Place a prim, coping with xformOps a reference already authored.

    XformCommonAPI cannot author a rotateXYZ over an `orient` op -- and it says
    so by returning False rather than raising, so an unchecked call leaves the
    robot at the origin and the run merely looks odd. The robot prim is a
    reference, so it does carry the asset's own ops; fall back to writing them.
    """
    from pxr import Gf, UsdGeom

    prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
    xf = UsdGeom.XformCommonAPI(prim)
    if (xf.SetTranslate(Gf.Vec3d(*[float(v) for v in xyz]))
            and xf.SetRotate(Gf.Vec3f(0.0, 0.0, math.degrees(yaw)),
                             UsdGeom.XformCommonAPI.RotationOrderXYZ)):
        return

    ops = {op.GetOpName(): op
           for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}
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


def build_stage():
    """Compose world + robot, the way gzserver composes a .world with a spawn.

    The environment is *referenced*, not baked into the robot stage, so
    switching worlds never means regenerating the robot asset -- and the robot
    asset stays the single reusable thing this package ships, exactly as
    models/turtlebot3_<model>/model.sdf is for Gazebo.
    """
    from isaacsim.core.api.objects import GroundPlane
    from isaacsim.core.api.materials.physics_material import PhysicsMaterial
    from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage
    from pxr import PhysxSchema, UsdGeom, UsdLux, UsdShade

    create_new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, '/World')

    # Handed a floor material explicitly. Left to itself GroundPlane authors one
    # at restitution 0.8 -- and PhysX combines restitution as an *average* by
    # default, so every wheel-on-floor contact came out at 0.4: a bouncy-ball
    # floor under a 0.94 kg robot, which made it rock on its caster skid and
    # creep with nothing commanding it. Nothing is logged when that happens.
    floor = PhysicsMaterial(prim_path='/World/PhysicsMaterials/floor',
                            static_friction=1.0, dynamic_friction=1.0,
                            restitution=0.0)
    physx = PhysxSchema.PhysxMaterialAPI.Apply(floor.prim)
    physx.CreateRestitutionCombineModeAttr().Set('min')
    GroundPlane(prim_path='/World/GroundPlane', physics_material=floor)
    # GroundPlane binds what it is given to its mesh collider and leaves the
    # infinite `collisionPlane` beside it on PhysX's fallback material. Binding
    # the parent too is what makes both resolve to the same floor.
    UsdShade.MaterialBindingAPI.Apply(
        stage.GetPrimAtPath('/World/GroundPlane')).Bind(
            floor.material, UsdShade.Tokens.weakerThanDescendants, 'physics')

    light = UsdLux.DistantLight.Define(stage, '/World/DistantLight')
    light.CreateIntensityAttr(1000)

    if args.world:
        if not os.path.isfile(args.world) and not os.path.isdir(args.world):
            raise RuntimeError(
                f'no world stage at {args.world}\n'
                f'  Worlds are generated, not committed. Build it first:\n'
                f'    isaacsim-python '
                f'{os.path.join(SHARE, "scripts", "import_turtlebot3.py")} '
                f'--world <name>')
        add_reference_to_stage(usd_path=args.world, prim_path=WORLD_PRIM)
        simulation_app.update()
        print(f'world: {args.world} at {WORLD_PRIM}', flush=True)
    else:
        print('world: none (bare ground plane)', flush=True)

    if not os.path.isfile(args.robot) and not os.path.isdir(args.robot):
        raise RuntimeError(
            f'no robot asset at {args.robot}\n'
            f'  The USD is generated from turtlebot3_description, not '
            f'committed. Build it first:\n'
            f'    isaacsim-python '
            f'{os.path.join(SHARE, "scripts", "import_turtlebot3.py")} '
            f'--model {args.model}')
    add_reference_to_stage(usd_path=args.robot, prim_path=ROBOT_PRIM)
    simulation_app.update()
    while stage_utils.is_stage_loading():
        simulation_app.update()

    xyz = (args.x_pose, args.y_pose, args.z_pose)
    set_pose(ROBOT_PRIM, xyz, args.yaw)
    print(f'robot: {args.robot} at {ROBOT_PRIM}, spawned at '
          f'{[round(v, 3) for v in xyz]} yaw {args.yaw:g}', flush=True)


def check_surfaces():
    """Fail on a robot whose colliders carry no surface properties.

    A robot asset built before import_turtlebot3.py learned to author physics
    materials references in, plays and publishes correctly -- it just never
    settles, rocking on its caster skid and drifting with no /cmd_vel at all.
    Nothing is logged, which is what makes it read as a physics-tuning problem
    rather than a stale asset. Say so here instead.
    """
    from pxr import Usd, UsdPhysics, UsdShade

    stage = omni.usd.get_context().get_stage()
    bad = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM)):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(
            'physics')
        if not material:
            bad.append(f'{prim.GetPath()}: no physics material bound')
            continue
        attr = UsdPhysics.MaterialAPI(material.GetPrim()).GetRestitutionAttr()
        restitution = attr.Get() if attr else None
        if restitution != 0.0:
            bad.append(f'{prim.GetPath()}: restitution {restitution} '
                       f'(from {material.GetPath()})')
    if bad:
        raise RuntimeError(
            f'{args.robot} has bouncy or unset surfaces:\n  '
            + '\n  '.join(bad)
            + '\n  The robot will rock on its caster and drift with no command '
              'given.\n  Rebuild it:  isaacsim-python '
            + os.path.join(SHARE, 'scripts', 'import_turtlebot3.py')
            + f' --model {args.model}')


def main():
    app_utils.enable_extension('isaacsim.ros2.bridge')
    simulation_app.update()

    build_stage()
    check_surfaces()

    articulation_root = find_articulation_root(ROBOT_PRIM)
    build_graph(articulation_root)

    # Bound to a name for its lifetime, not discarded -- see attach_lidar().
    lidar_sensor = None
    if not args.no_lidar:
        lidar_sensor = attach_lidar()

    SimulationManager.setup_simulation(dt=1.0 / args.physics_hz, device='cpu')
    simulation_app.update()

    # The bridge publishes nothing while the timeline is stopped, so this is the
    # line that makes the whole thing live.
    app_utils.play()
    simulation_app.update()

    # Sentinel. isaacsim_bringup runs this script as a grandchild process whose
    # stdout it inherits, so a launch file can gate on this line with
    # OnProcessIO -- which is how NVIDIA's own carter_navigation_isaacsim.launch.py
    # gates, except that theirs matches a string only the GUI path prints.
    print(f'Stage loaded and simulation is playing. '
          f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "unset")} '
          f'RMW={os.environ.get("RMW_IMPLEMENTATION", "default")}', flush=True)

    while simulation_app.is_running():
        simulation_app.update()

    app_utils.stop()
    del lidar_sensor


if __name__ == '__main__':
    # The traceback is printed HERE, not left to the interpreter: os._exit()
    # below terminates the process immediately, before Python gets to report an
    # exception on its way out. Without this, every failure in main() -- a
    # missing asset, a bad prim path, check_surfaces() -- looks identical to a
    # clean shutdown: no message, exit status 0.
    status = 0
    try:
        main()
    except BaseException:
        traceback.print_exc()
        status = 1
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        # A graceful simulation_app.close() races a background task-pool
        # teardown ("Destroying busy TaskGroup!") and aborts the process on the
        # way out. Skip it rather than chase the race.
        os._exit(status)
