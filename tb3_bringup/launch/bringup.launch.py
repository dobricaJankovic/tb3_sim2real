"""Single entry point for all three backends.

    ros2 launch tb3_bringup bringup.launch.py backend:=gazebo
    ros2 launch tb3_bringup bringup.launch.py backend:=isaacsim world:=empty_stage
    ros2 launch tb3_bringup bringup.launch.py backend:=real
    ros2 launch tb3_bringup bringup.launch.py backend:=gazebo world:=/path/to/my_office

`backend:=` is the only thing that changes between a simulated run and a real
one. `world:=` names the ENVIRONMENT, not a file, and means the same thing
everywhere: the backend picks up whichever representation of it applies —
gzserver loads its .world, Kit opens its .usd, and the real robot loads nothing
because the environment is already around it. All three take the robot's start
pose and the Nav2 map from the same manifest, which is what makes a run on one
backend comparable with a run on another.

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

    backend = LaunchConfiguration('backend').perform(context)
    if backend not in worlds.BACKENDS:
        raise RuntimeError(
            f"backend must be one of {worlds.BACKENDS}, got '{backend}'")

    # The one thing that should never be set by hand again.
    use_sim_time = 'false' if backend == 'real' else 'true'
    simulated = backend != 'real'

    world = worlds.World.load(LaunchConfiguration('world').perform(context))

    def inc(rel, **extra):
        # scoped + forwarding=False, and both halves are load-bearing.
        #
        # IncludeLaunchDescription on its own does NOT isolate launch
        # configurations: everything declared here leaks into the included file,
        # where it takes precedence over that file's own DeclareLaunchArgument
        # default (a declaration only fills in a value that is not already set).
        # So `world` — a registry name here — would arrive at the backend as a
        # name where it expects a resolved path, and `headless` would collide
        # with isaacsim_bringup's own differently-typed argument of that name.
        return GroupAction(
            [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', rel)),
                launch_arguments={'use_sim_time': use_sim_time, **extra}.items(),
            )],
            scoped=True,
            forwarding=False,
        )

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

    actions = [
        # Same URDF drives the kinematic tree in all three worlds. Each backend
        # only has to supply odom->base_footprint plus sensor topics.
        inc('common/state_publisher.launch.py'),
        inc(f'backends/{backend}.launch.py', **backend_args),
    ]

    if LaunchConfiguration('nav').perform(context) != 'true':
        return actions

    map_yaml = LaunchConfiguration('map').perform(context) or world.map()
    if not map_yaml:
        raise RuntimeError(
            f"nav:=true needs a map, and world '{world.name}' declares none.\n"
            f'  Either pass map:=/path/to/map.yaml, or run with nav:=false, or '
            f'give the world a map: key (see worlds/README.md).')

    nav2 = inc('common/nav2.launch.py',
               params_file=params_file(pkg, backend),
               map=map_yaml,
               rviz=LaunchConfiguration('rviz').perform(context))

    if not simulated:
        return actions + [nav2]

    # Nav2 with use_sim_time:=true waits for /clock silently, which looks
    # exactly like a DDS domain mismatch or a simulator that never started —
    # and Kit can take two minutes on a cold shader cache. Start it only once
    # something is actually publishing sim time, and say so in the log
    # meanwhile. wait_for_sim exits as soon as the first /clock arrives.
    waiter = Node(package='tb3_bringup', executable='wait_for_sim',
                  name='wait_for_sim', output='screen')
    return actions + [
        waiter,
        RegisterEventHandler(OnProcessExit(target_action=waiter,
                                           on_exit=[nav2])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'backend',
            description='Which robot to bring up: real | gazebo | isaacsim'),
        DeclareLaunchArgument(
            'world', default_value='turtlebot3_world',
            description='Environment: a registry name (worlds/<name>/) or a '
                        'path to a world directory. backend:=real uses it for '
                        'the map only.'),
        DeclareLaunchArgument(
            'nav', default_value='true',
            description='Also start the Nav2 stack'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Also start RViz'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run the simulator with no window (ignored by '
                        'backend:=real)'),
        DeclareLaunchArgument(
            'map', default_value='',
            description="Occupancy map for Nav2; empty means the world's own"),
        OpaqueFunction(function=setup),
    ])
