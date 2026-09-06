"""Gazebo Classic backend.

Supplies: /clock, /scan, /odom, /joint_states, tf odom->base_footprint,
/cmd_vel sink — all from gazebo_ros plugins baked into the turtlebot3_gazebo URDF.

Deliberately does not include robot_state_publisher: that is in common/ and is
shared with the other two backends.

The world comes from the repo's registry (worlds/<name>/), not from
turtlebot3_gazebo's installed worlds, so that the same environment definition
drives Isaac Sim too. `world:=` takes a registry NAME; see tb3_bringup.worlds.
The spawn pose is read from the world's manifest rather than defaulted here, so
a Gazebo run and an Isaac Sim run of the same world start from the same pose.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from tb3_bringup import worlds


def setup(context, *args, **kwargs):
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    world_arg = LaunchConfiguration('world').perform(context)
    world_path = worlds.resolve(world_arg)

    # x_pose/y_pose default to the manifest's spawn, but stay overridable so a
    # single run can be started somewhere else without editing the world.
    x, y, _z, _yaw = worlds.spawn(os.path.basename(os.path.dirname(world_path)))
    x_pose = LaunchConfiguration('x_pose').perform(context) or f'{x:g}'
    y_pose = LaunchConfiguration('y_pose').perform(context) or f'{y:g}'

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world_path}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world', default_value='turtlebot3_world',
            description='World from the registry (worlds/<name>/), or a path to '
                        'a .world file'),
        # Empty means "use the world manifest's spawn pose".
        DeclareLaunchArgument('x_pose', default_value=''),
        DeclareLaunchArgument('y_pose', default_value=''),
        OpaqueFunction(function=setup),
    ])
