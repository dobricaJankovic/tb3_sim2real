"""Isaac Sim backend.

Starts the simulator, rather than waiting for someone to have started it. It
does that by including turtlebot3_isaacsim, a peer of turtlebot3_gazebo
developed in its own repository and imported here by scripts/workspace.sh —
which in turn includes NVIDIA's isaacsim_bringup/run_isaacsim.launch.py, the
supported way to put Kit behind a `ros2 launch`.

That package owns the robot asset, the OmniGraph that publishes the interface
contract, the RTX lidar profile and the physics materials. This file owns
nothing but the translation from a world manifest into its arguments, which is
the whole reason `backend:=isaacsim world:=X` can now mean what
`backend:=gazebo world:=X` means.

Supplies: /clock, /scan, /odom, /joint_states, tf odom->base_footprint,
/cmd_vel sink. Deliberately does not include robot_state_publisher — that is in
common/, shared with the other two backends — so turtlebot3_isaacsim's own
robot_state_publisher.launch.py is bypassed by including its isaacsim.launch.py
rather than one of its per-world launch files.

`world` arrives already resolved by bringup.launch.py. An EMPTY string is
meaningful and is what `empty_stage` produces: no environment reference, and
the simulator authors its own ground plane and light.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

#: Forwarded verbatim; each has a default in turtlebot3_isaacsim's own file.
PASS_THROUGH = ('world', 'world_z', 'x_pose', 'y_pose', 'z_pose', 'yaw',
                'headless')


def include_simulator(context, pkg):
    """Include turtlebot3_isaacsim, forwarding physics_hz only if it was set.

    physics_hz is deliberately declared empty here rather than with a number.
    The rate and the reasoning for it belong to turtlebot3_isaacsim, which owns
    the asset and the physics; a default repeated here would be a second copy
    free to drift from it -- and did, for a day, pinning 480 after the package
    had moved to 240. Forwarding an empty string is not an option either: it
    reaches the simulator as `--physics-hz ''` and argparse rejects it.
    """
    arguments = {k: LaunchConfiguration(k).perform(context)
                 for k in PASS_THROUGH}
    physics_hz = LaunchConfiguration('physics_hz').perform(context)
    if physics_hz:
        arguments['physics_hz'] = physics_hz
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'isaacsim.launch.py')),
        launch_arguments=arguments.items(),
    )]


def generate_launch_description():
    try:
        pkg = get_package_share_directory('turtlebot3_isaacsim')
    except Exception as e:                       # PackageNotFoundError
        raise RuntimeError(
            'backend:=isaacsim needs the turtlebot3_isaacsim package, which is '
            'a source dependency of this repository rather than part of it.\n'
            '  scripts/workspace.sh              # fetch it into src/\n'
            '  colcon build --symlink-install\n'
            f'  ({e})')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world', default_value='',
            description='Path to the environment .usd; empty for a ground plane'),
        DeclareLaunchArgument(
            'world_z', default_value='0.0',
            description='Metres to raise the stage by so its floor meets z = 0'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.01'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'physics_hz', default_value='',
            description='PhysX sub-steps per simulated second. Empty uses '
                        "turtlebot3_isaacsim's own default, which is where the "
                        'rate and the reasoning for it live. Set it to compare '
                        'two rates without editing a default in an imported '
                        'package and remembering to put it back.'),

        OpaqueFunction(function=include_simulator, kwargs={'pkg': pkg}),
    ])
