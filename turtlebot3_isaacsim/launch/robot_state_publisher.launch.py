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

"""robot_state_publisher, the one node that is identical on every backend.

It holds the URDF and subscribes /joint_states, and publishes:

    /tf         the two continuous wheel joints, driven by /joint_states
    /tf_static  every fixed joint: base_footprint->base_link, ->base_scan,
                ->imu_link, ->caster_back_link
    /robot_description  (latched) for RViz and Nav2

It never publishes odom->base_footprint. That belongs to the simulator, exactly
as it belongs to turtlebot3_node on the real robot, which is what lets the same
URDF serve all three. Three owners, no overlap:

    map --(amcl)--> odom --(this package)--> base_footprint --(rsp)--> the rest

`use_sim_time` matters here specifically because this node *stamps* tf. Stamping
with wall clock while everything else is on sim clock produces tf extrapolation
errors even when the geometry is perfect.

Deliberately NOT a copy of turtlebot3_gazebo's file of the same name, which has
two problems worth not inheriting:

  * it reads the URDF with open(). turtlebot3_description's URDF is a xacro
    TEMPLATE carrying a ${namespace} argument; read as plain text, every frame
    comes out literally named "${namespace}base_footprint".
  * its `frame_prefix` default of '' goes through
    PythonExpression(["'", frame_prefix, "/'"]), which evaluates to the string
    "/" -- not to an empty prefix.

Both are avoided by expanding the template with xacro, the way the real robot's
turtlebot3_bringup/turtlebot3_state_publisher.launch.py does.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    print('urdf_file_name : {}'.format(urdf_file_name))

    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)

    namespace = LaunchConfiguration('namespace')

    # ParameterValue(value_type=str) stops the expanded XML being reinterpreted
    # as YAML -- a classic silent robot_state_publisher failure.
    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' namespace:=',
            PythonExpression(
                ['"', namespace, '" + "/" if "', namespace, '" != "" else ""']),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Isaac Sim) clock if true'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Frame prefix, for multi-robot. Empty for one robot.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
