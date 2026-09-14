"""Gazebo Classic backend.

Supplies: /clock, /scan, /odom, /joint_states, tf odom->base_footprint,
/cmd_vel sink — all from gazebo_ros plugins baked into the turtlebot3_gazebo
model SDF.

Deliberately does not include robot_state_publisher: that is in common/ and is
shared with the other two backends.

`world` arrives already resolved to a .world path — bringup.launch.py does the
registry lookup once, for whichever backend is running, so that `world:=` means
the same environment everywhere. The same goes for the spawn pose.

The robot is spawned with spawn_entity.py directly rather than through
turtlebot3_gazebo's spawn_turtlebot3.launch.py, which takes only x_pose and
y_pose and pins z to 0.01. The manifest's spawn carries a yaw, and Isaac Sim
honours it; a Gazebo backend that silently dropped it would put the two
simulators at different headings from the same manifest, which is exactly the
class of divergence this repository is meant to make impossible.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_ros = get_package_share_directory('gazebo_ros')
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    robot_sdf = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models', f'turtlebot3_{model}', 'model.sdf')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            description='Path to the .world gzserver should load'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.01'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Skip gzclient; gzserver still runs'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=UnlessCondition(LaunchConfiguration('headless')),
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', model,
                '-file', robot_sdf,
                '-x', LaunchConfiguration('x_pose'),
                '-y', LaunchConfiguration('y_pose'),
                '-z', LaunchConfiguration('z_pose'),
                '-Y', LaunchConfiguration('yaw'),
            ],
        ),
    ])
