"""robot_state_publisher — identical for all three backends.

This is the load-bearing decision of the design: the URDF is the single source
of truth for the kinematic tree, so real / gazebo / isaacsim cannot disagree
about frame geometry. Each backend supplies only a *raw* odom -> base_footprint
transform plus sensor topics; everything below base_footprint comes from here.

Uses turtlebot3_description, which carries no simulator plugins. Verified
against turtlebot3_gazebo's URDF: links, joints and joint origins are
IDENTICAL, the gazebo copy just adds <gazebo> plugin blocks. So the Gazebo
backend spawning from its own URDF does not conflict with this.

turtlebot3_description's URDF is a xacro TEMPLATE with a ${namespace} arg — it
must be expanded with xacro, not read as plain text, or every frame comes out
literally named "${namespace}base_footprint".

Frames produced (namespace empty):
    base_footprint, base_link, base_scan, caster_back_link,
    imu_link, wheel_left_link, wheel_right_link
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        f'turtlebot3_{model}.urdf',
    )

    namespace = LaunchConfiguration('namespace')

    # ParameterValue(value_type=str) stops the expanded XML being reinterpreted
    # as YAML — a classic silent robot_state_publisher failure.
    robot_description = ParameterValue(
        Command([
            'xacro ', urdf,
            ' namespace:=',
            PythonExpression(['"', namespace, '" + "/" if "', namespace, '" != "" else ""']),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Frame prefix, for multi-robot. Empty for a single robot.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
