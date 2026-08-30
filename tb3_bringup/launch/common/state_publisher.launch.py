"""robot_state_publisher — identical for all three backends.

This is the whole point of the design: the URDF is the single source of truth
for the kinematic tree, so real / gazebo / isaacsim cannot disagree about frame
geometry. Each backend supplies only odom -> base_footprint (a *raw* transform)
plus sensor topics; everything below base_footprint comes from here.

Uses turtlebot3_description (no simulator plugins). The Gazebo backend spawns
from turtlebot3_gazebo's URDF, which carries the plugins — TODO: confirm the
link/joint geometry in the two files actually matches, they are separate files.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context, *args, **kwargs):
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        f'turtlebot3_{model}.urdf',
    )
    with open(urdf, 'r') as f:
        robot_description = f.read()

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true',
            'robot_description': robot_description,
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=setup),
    ])
