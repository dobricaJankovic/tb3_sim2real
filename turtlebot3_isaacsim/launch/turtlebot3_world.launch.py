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

"""TurtleBot3 in the turtlebot3_world environment, on Isaac Sim.

    ros2 launch turtlebot3_isaacsim turtlebot3_world.launch.py

Then, in another terminal, exactly what you would run against Gazebo or the
real robot -- only use_sim_time changes:

    ros2 launch turtlebot3_navigation2 navigation2.launch.py \\
        use_sim_time:=true map:=$HOME/map.yaml

turtlebot3_gazebo's file of this name has four includes. This has two, and the
two that are missing are missing for reasons rather than by omission:

  * **gzclient** has no counterpart. Gazebo splits server and GUI into two
    processes; Kit is one process that either opens a window or does not, so
    the GUI is `headless:=` on the include below rather than a launch file.

  * **spawn_turtlebot3** has no counterpart either, and this is the one real
    structural difference between the two packages. gazebo_ros ships
    libgazebo_ros_factory.so, which serves a `spawn_entity` ROS service, so
    Gazebo can have a robot pushed into an already-running server by a separate
    node. Isaac Sim ships no equivalent: NVIDIA's service surface
    (isaac_ros2_messages) is GetPrims, Get/SetPrimAttribute and IsaacPose --
    inspection, attribute setting and pose teleport, but nothing that *creates*
    a prim. So the robot is referenced into the stage by the simulator script
    itself at startup, and `x_pose`/`y_pose` below are forwarded to it.
    The argument names are kept identical to turtlebot3_gazebo's on purpose.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_isaacsim'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    headless = LaunchConfiguration('headless', default='false')

    world = os.path.join(
        get_package_share_directory('turtlebot3_isaacsim'),
        'worlds',
        'turtlebot3_world.usd'
    )

    isaacsim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'isaacsim.launch.py')
        ),
        launch_arguments={
            'world': world,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'headless': headless,
        }.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(isaacsim_cmd)
    ld.add_action(robot_state_publisher_cmd)

    return ld
