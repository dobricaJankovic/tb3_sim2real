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

"""TurtleBot3 on a bare ground plane, on Isaac Sim.

    ros2 launch turtlebot3_isaacsim empty_world.launch.py

The counterpart of turtlebot3_gazebo's empty_world.launch.py, and the right
first thing to run: it exercises the whole chain -- Kit start, robot asset,
OmniGraph, lidar, DDS -- with no environment to go wrong. If /scan is empty
here that is correct, because there is nothing to see.

`world` is left unset rather than pointing at an empty_world.usd. Gazebo needs
a .world file even for an empty world because gzserver has to be told what to
load; the simulator script builds its own ground plane and light, so an empty
environment is the absence of a reference, not a file.
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
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    headless = LaunchConfiguration('headless', default='false')

    isaacsim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'isaacsim.launch.py')
        ),
        launch_arguments={
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

    ld.add_action(isaacsim_cmd)
    ld.add_action(robot_state_publisher_cmd)

    return ld
