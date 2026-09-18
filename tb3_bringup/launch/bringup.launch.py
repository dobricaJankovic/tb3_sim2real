"""Single entry point for all three backends.

    ros2 launch tb3_bringup bringup.launch.py backend:=gazebo   world:=turtlebot3_world
    ros2 launch tb3_bringup bringup.launch.py backend:=isaacsim world:=empty_stage
    ros2 launch tb3_bringup bringup.launch.py backend:=real     world:=lab_room
    ros2 launch tb3_bringup bringup.launch.py backend:=gazebo   world:=/path/to/my_office

`backend:=` and `world:=` are both required and have no default: `backend`
names the machine and `world` names the room, and guessing either produces a
run that looks fine and means nothing. `backend:=` picks who supplies the
robot layer (drivers, odometry, `robot_state_publisher`) and derives
`use_sim_time` so it is never set by hand; `world:=` picks the environment,
resolved through `tb3_bringup.worlds` so every backend agrees on the start
pose and the map. The two-layer split, why `backend:=real` contributes no
local processes here, and why `world:=` means the same thing on every backend
are written up in `docs/architecture.md`; this file only implements them.

The workstation layer is three modes:

    (neither)    robot + RViz. Drive it, run drive_test, attach your own stack.
    nav:=true    map_server + AMCL + Nav2, on the world's saved map.
                 No map -> error naming the fix: run slam:=true, then
                 scripts/save_map.py <world>
    slam:=true   slam_toolbox + Nav2. Navigate while building the map.

`nav:=true` navigates on a saved map; `slam:=true` navigates while making one;
neither gives you a bare robot. `slam:=true` already runs Nav2, so `slam:=true
nav:=true` is accepted rather than rejected — it means the same thing. To
drive manually instead of running Nav2, use a second terminal: `ros2 run
turtlebot3_teleop teleop_keyboard` or `ros2 run tb3_bringup drive_test`.

There is no `map:=` — a map lives in `worlds/<name>/map/` or it has not been
made yet, and `slam:=true` is how it gets made. There is no `initial_pose:=`
either: under `nav:=true` AMCL starts at the manifest's `spawn:` on every
backend, so no run begins with a human clicking in RViz (see
`pin_initial_pose`).

OpaqueFunction rather than IfCondition, so the backend argument is a plain
Python string we can branch on. That also lets us derive use_sim_time from the
backend instead of making the operator remember it.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tb3_bringup import worlds


def pin_initial_pose(params, world):
    """Write a params file with AMCL's initial pose set from the manifest.

    Without this, every Nav2 run needs a human to click "2D Pose Estimate" in
    RViz — a few centimetres and a few degrees of INDEPENDENT RANDOM ERROR per
    run, landing directly in goal_err_m, path_ratio and elapsed_s, which are
    the numbers the experiments compare. It cannot be separated from the
    sim-to-real gap afterwards. docs/roadmap.md §1 has the alternatives and
    why global localization is not the experimental default here.

    AMCL's own parameters, not a `/initialpose` publish: the publish races
    AMCL's activation, and that race is what swallowed the pose during the
    2026-09-16 hardware test.

    A temporary COPY, so `config/nav2_params.yaml` stays verbatim upstream —
    the same move, for the same reason, as `with_ground_truth()` in
    backends/gazebo.launch.py. Written here rather than with nav2_common's
    `RewrittenYaml`, which cannot do it: on Humble `substitute_params` walks
    the paths the source file ALREADY HAS (`pathify`) and rewrites only those,
    so a dotted path to an absent key is silently a no-op. All four of these
    are absent from upstream's file. The audit that put RewrittenYaml in
    docs/experiment-plan.md B5 got this wrong, and the failure it would have
    produced is the invisible kind: the launch succeeds, the file is written,
    and AMCL waits for a mouse anyway.

    Nothing downstream changes: localization_launch.py wraps whatever path it
    is handed in its own RewrittenYaml for use_sim_time.

    On backend:=real this is only as true as the floor marker: the robot has to
    START at the manifest's spawn, or AMCL begins confidently wrong. The marker
    is part of the protocol, not a convenience (docs/experiment-plan.md B5).
    """
    with open(params) as f:
        data = yaml.safe_load(f)
    x, y, _z, yaw = world.spawn
    amcl = data.setdefault('amcl', {}).setdefault('ros__parameters', {})
    amcl['set_initial_pose'] = True
    # z and the other two rotations are not offered: the manifest's spawn z is
    # a spawn HEIGHT for a simulator, and AMCL localises in the plane.
    amcl['initial_pose'] = {'x': float(x), 'y': float(y), 'z': 0.0,
                            'yaw': float(yaw)}
    out = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='tb3_nav2_params_', delete=False)
    yaml.safe_dump(data, out)
    out.close()
    return out.name


def setup(context, *args, **kwargs):
    pkg = get_package_share_directory('tb3_bringup')
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    backend = LaunchConfiguration('backend').perform(context)
    if backend not in worlds.BACKENDS:
        raise RuntimeError(
            f"backend must be one of {worlds.BACKENDS}, got '{backend}'")

    # The one thing that should never be set by hand again.
    use_sim_time = 'false' if backend == 'real' else 'true'
    simulated = backend != 'real'

    world = worlds.World.load(LaunchConfiguration('world').perform(context))

    def inc(path, **extra):
        # scoped + forwarding=False, and both halves are load-bearing.
        #
        # IncludeLaunchDescription on its own does NOT isolate launch
        # configurations: everything declared here leaks into the included file,
        # where it takes precedence over that file's own DeclareLaunchArgument
        # default (a declaration only fills in a value that is not already set).
        # So `world` — a registry name here — would arrive at the backend as a
        # name where it expects a resolved path, and `headless` would collide
        # with isaacsim_bringup's own differently-typed argument of that name.
        # Upstream's files care too: `slam` leaking into navigation_launch.py
        # would be read as its own unrelated argument.
        return GroupAction(
            [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(path),
                launch_arguments={'use_sim_time': use_sim_time, **extra}.items(),
            )],
            scoped=True,
            forwarding=False,
        )

    def ours(rel):
        return os.path.join(pkg, 'launch', rel)

    def nav2(rel):
        # nav2_bringup's own launch files, included directly rather than through
        # bringup_launch.py: `slam` and `nav` are independent questions here, so
        # that wrapper's whole job — an IfCondition switching between the two —
        # has nothing left to do. See docs/architecture.md.
        return os.path.join(nav2_launch_dir, rel)

    x, y, z, yaw = world.spawn
    backend_args = {}
    if simulated:
        backend_args = {
            'x_pose': f'{x:g}', 'y_pose': f'{y:g}', 'z_pose': f'{z:g}',
            'yaw': f'{yaw:g}',
            'headless': LaunchConfiguration('headless').perform(context),
        }
        backend_args['world'] = (world.gazebo_world() if backend == 'gazebo'
                                 else world.isaac_usd())
        if backend == 'isaacsim':
            backend_args['world_z'] = f'{world.isaac_world_z():g}'
            # Empty means "whatever the backend defaults to". The number and
            # the reasoning for it live in backends/isaacsim.launch.py; a
            # default repeated here would be a second copy of it, free to
            # drift. Forwarding '' would override that default with nothing.
            physics_hz = LaunchConfiguration('physics_hz').perform(context)
            if physics_hz:
                backend_args['physics_hz'] = physics_hz

    # The robot layer. Same URDF drives the kinematic tree in both simulated
    # worlds; each backend only has to supply odom->base_footprint plus sensor
    # topics on top of it. backend:=real supplies neither from here — the robot
    # runs its own turtlebot3_bringup already; see docs/architecture.md and
    # docs/roadmap.md for the tethered alternative this deliberately is not.
    actions = [] if backend == 'real' else [
        inc(ours('common/state_publisher.launch.py')),
        inc(ours(f'backends/{backend}.launch.py'), **backend_args),
    ]

    if LaunchConfiguration('rviz').perform(context) == 'true':
        actions.append(inc(ours('common/rviz.launch.py')))

    # The workstation layer: three modes, documented at the top of this file.
    slam = LaunchConfiguration('slam').perform(context) == 'true'
    nav = LaunchConfiguration('nav').perform(context) == 'true'
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    stack = []
    if slam:
        # slam_toolbox serves /map and publishes map->odom, so map_server and
        # AMCL must not also run. Upstream's slam_launch.py adds a
        # lifecycle-managed map_saver_server alongside it.
        #
        # slam_toolbox on Humble is a plain rclcpp::Node, NOT a lifecycle node:
        # slam_launch.py's lifecycle manager governs `map_saver` and nothing
        # else. Do not wait for a transition that never arrives.
        stack.append(inc(nav2('slam_launch.py'), params_file=params))
    elif nav:
        map_yaml = world.map()
        if not map_yaml:
            raise RuntimeError(
                f"nav:=true needs a map, and world '{world.name}' has none.\n"
                f'  Make one: run with slam:=true, drive the robot around, then '
                f'scripts/save_map.py {world.name}\n'
                f'  (There is no map:= argument. A map lives in its world\'s '
                f'directory or it does not exist — see worlds/README.md.)')
        stack.append(inc(nav2('localization_launch.py'),
                         map=map_yaml, params_file=pin_initial_pose(params, world)))

    if nav or slam:
        # Under slam:=true this runs alongside slam_toolbox rather than
        # map_server + AMCL, which is why nav:=true adds nothing new when
        # slam:=true is already set.
        stack.append(inc(nav2('navigation_launch.py'), params_file=params))

    if not stack:
        return actions

    if not simulated:
        return actions + stack

    # Nav2 with use_sim_time:=true waits for /clock silently, which looks
    # exactly like a DDS domain mismatch or a simulator that never started —
    # and Kit can take two minutes on a cold shader cache. Start it only once
    # something is actually publishing sim time, and say so in the log
    # meanwhile. wait_for_sim exits as soon as the first /clock arrives.
    waiter = Node(package='tb3_bringup', executable='wait_for_sim',
                  name='wait_for_sim', output='screen')
    return actions + [
        waiter,
        # handle_once, or the whole stack is brought up twice: the handler
        # stays registered otherwise and a second matching exit runs every
        # include again. The symptom is a wall of `Transition is not
        # registered` and `Node '/local_costmap/local_costmap' has already been
        # added to an executor`.
        RegisterEventHandler(OnProcessExit(target_action=waiter,
                                           on_exit=stack,
                                           handle_once=True)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'backend',
            description='Which robot to bring up: real | gazebo | isaacsim'),
        DeclareLaunchArgument(
            # Required, deliberately: no default. See the module docstring and
            # docs/architecture.md for why guessing either argument is worse
            # than an error.
            'world',
            description='Environment: a registry name (worlds/<name>/) or a '
                        'path to a world directory. backend:=real uses it for '
                        'the map only.'),
        DeclareLaunchArgument(
            'nav', default_value='false',
            description='Navigate on the world\'s saved map: map_server + '
                        'AMCL + Nav2. Ignored if slam:=true, which already '
                        'includes Nav2.'),
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='Navigate while building the map: slam_toolbox + '
                        'Nav2 in place of map_server + AMCL. Save the result '
                        'with scripts/save_map.py <world>.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Also start RViz'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run the simulator with no window (ignored by '
                        'backend:=real)'),
        DeclareLaunchArgument(
            # Not a tuning knob: below the backend's default the wheels
            # chatter against the ground rather than tracking their command,
            # which is measured in docs/worknotes/2026-09-17-lane-a-physics.md.
            # It is an argument at all so that cost can be traded against
            # fidelity in a measurement, and so the rate is recorded with one.
            'physics_hz', default_value='',
            description='PhysX sub-steps per second, backend:=isaacsim only. '
                        'Empty uses the backend default.'),
        OpaqueFunction(function=setup),
    ])
