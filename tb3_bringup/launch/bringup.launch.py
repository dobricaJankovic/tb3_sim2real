"""Single entry point for all three backends.

    ros2 launch tb3_bringup bringup.launch.py backend:=gazebo   world:=turtlebot3_world
    ros2 launch tb3_bringup bringup.launch.py backend:=isaacsim world:=empty_stage
    ros2 launch tb3_bringup bringup.launch.py backend:=real     world:=lab_room
    ros2 launch tb3_bringup bringup.launch.py backend:=gazebo   world:=/path/to/my_office

Both arguments are required. Neither has a sensible default: `backend` names
the machine and `world` names the room, and guessing either produces a run that
looks fine and means nothing.

A run is two layers, the way ROS already splits them:

  robot layer        drivers, odometry, robot_state_publisher. Supplied by a
                     simulator, or by the hardware itself.
  workstation layer  map server, localization, Nav2, RViz. Identical
                     everywhere; it does not know which robot it is driving.

`backend:=` answers exactly one question — who provides the robot layer.
`gazebo` and `isaacsim` start it here; `real` starts nothing here, because the
robot is a second computer already running its own turtlebot3_bringup. That is
why this file contributes no local processes for `backend:=real`: correct, not
broken.

`world:=` names the ENVIRONMENT, not a file, and means the same thing
everywhere: the backend picks up whichever representation of it applies —
gzserver loads its .world, Kit opens its .usd, and the real robot loads nothing
because the environment is already around it. All three take the robot's start
pose and the Nav2 map from the same manifest, which is what makes a run on one
backend comparable with a run on another.

The workstation layer is two independent questions, not one three-way choice:

                 nav:=false                  nav:=true
  slam:=false    robot only; teleop          map_server + AMCL + Nav2
  slam:=true     slam_toolbox + teleop       Nav2 while mapping

`slam` answers *where does map->odom come from* — slam_toolbox builds the map
and publishes the transform, or map_server and AMCL use one built earlier.
`nav` answers *does the navigation stack run*. Neither excludes the other, so
no combination has to be rejected, and `slam:=true nav:=true` is a real mode:
Nav2 plans over a map that slam_toolbox is still drawing.

Both default false. `world:=` is required and `map:=` does not exist — the map
is `worlds/<name>/map.yaml` or it has not been made yet, and `slam:=true` is
how it gets made. A map passed on the command line is a map that drifts away
from the world it describes, which is the failure the registry exists to
prevent.

OpaqueFunction rather than IfCondition, so the backend argument is a plain
Python string we can branch on. That also lets us derive use_sim_time from the
backend instead of making the operator remember it.
"""

import os

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


def params_file(pkg, backend):
    """config/nav2_<backend>.yaml if it exists, else the shared one.

    The backends need identical Nav2 structure and, eventually, different
    tuning; the delta between them is a measurement of the sim-to-real gap. A
    per-backend file therefore wins as soon as someone writes one, and until
    then there is one copy rather than three that drift.
    """
    specific = os.path.join(pkg, 'config', f'nav2_{backend}.yaml')
    return specific if os.path.isfile(specific) else os.path.join(
        pkg, 'config', 'nav2_params.yaml')


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
        # nav2_bringup's own launch files, included directly rather than
        # through its bringup_launch.py. That wrapper exists to make `slam`
        # choose between slam_launch.py and localization_launch.py with an
        # `IfCondition(['not ', slam])`; here the two questions are
        # independent, so the wrapper has nothing left to do.
        #
        # One consequence: localization_launch.py and navigation_launch.py
        # default `use_composition` to False, where bringup_launch.py defaults
        # it True and creates the shared `nav2_container` itself. So Nav2's
        # nodes now run as separate processes. Slightly more overhead, and it
        # retires the failure mode in docs/network.md where the container
        # starts and no composable node is ever loaded into it.
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

    # The robot layer. Same URDF drives the kinematic tree in both simulated
    # worlds; each backend only has to supply odom->base_footprint plus sensor
    # topics on top of it.
    #
    # backend:=real supplies neither from here. The robot runs its own
    # turtlebot3_bringup and already publishes /scan, /odom, the URDF and tf; a
    # robot_state_publisher started here would fight the robot's over
    # /tf_static and /robot_description, and drivers started here would go
    # looking for USB devices attached to the other machine. If the two are on
    # different subnets, discovery needs help as well: docs/network.md.
    #
    # The tethered alternative — OpenCR and lidar on this machine's USB, so the
    # robot layer runs here too — is deliberately not an argument, because a
    # robot that drives cannot be tethered. It comes back the day an x86 SBC or
    # a Jetson goes on the robot, and the way back is to include upstream's
    # turtlebot3_bringup/launch/robot.launch.py rather than to re-derive it.
    # One thing that costs an hour if it is re-derived: turtlebot3_node
    # declares `namespace` as a statically typed parameter with no default, so
    # it must be passed as a PARAMETER and not merely as a frame prefix, or the
    # process aborts before it opens the serial port with
    # `Statically typed parameter 'namespace' must be initialized`.
    actions = [] if backend == 'real' else [
        inc(ours('common/state_publisher.launch.py')),
        inc(ours(f'backends/{backend}.launch.py'), **backend_args),
    ]

    if LaunchConfiguration('rviz').perform(context) == 'true':
        actions.append(inc(ours('common/rviz.launch.py')))

    # The workstation layer. Two independent questions; see the grid at the top
    # of this file. The dispatch is deliberately flat — `slam` picks the source
    # of map->odom, `nav` switches the navigation stack on, and neither is
    # phrased as the absence of the other.
    slam = LaunchConfiguration('slam').perform(context) == 'true'
    nav = LaunchConfiguration('nav').perform(context) == 'true'
    params = params_file(pkg, backend)

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
                         map=map_yaml, params_file=params))

    if nav:
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
            # Required, deliberately: no default. A world is what makes two
            # runs comparable, so which one this is must be stated rather than
            # inherited. A default silently attributes every unqualified run to
            # turtlebot3_world — including a real-robot run in a room that is
            # not turtlebot3_world, where the map is simply wrong and Nav2
            # localises into fiction rather than failing.
            'world',
            description='Environment: a registry name (worlds/<name>/) or a '
                        'path to a world directory. backend:=real uses it for '
                        'the map only.'),
        DeclareLaunchArgument(
            'nav', default_value='false',
            description='Run the navigation stack: planner, controller, '
                        'behaviours, bt_navigator. Without slam:=true it also '
                        "runs map_server and AMCL on the world's map."),
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='Build the map as you go: slam_toolbox supplies /map '
                        'and map->odom in place of map_server and AMCL. '
                        'Combines with nav:=true. Save the result with '
                        'scripts/save_map.py <world>.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Also start RViz'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run the simulator with no window (ignored by '
                        'backend:=real)'),
        OpaqueFunction(function=setup),
    ])
