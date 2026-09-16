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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
            'physics_hz', default_value='480.0',
            description='PhysX sub-steps per simulated second. 480 because at '
                        'the old 60 the wheels chatter instead of tracking '
                        'their commanded velocity; see '
                        'docs/worknotes/2026-09-17-lane-a-physics.md. Exposed '
                        'here so that comparing two rates does not mean editing '
                        'a default in an imported package and remembering to '
                        'put it back.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'isaacsim.launch.py')),
            launch_arguments={
                k: LaunchConfiguration(k) for k in (
                    'world', 'world_z', 'x_pose', 'y_pose', 'z_pose', 'yaw',
                    'headless', 'physics_hz')
            }.items(),
        ),
    ])
