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

"""Start Isaac Sim. This package's gzserver.launch.py.

Wraps NVIDIA's isaacsim_bringup, which is to Isaac Sim what gazebo_ros is to
Gazebo: the vendor's own "run the simulator from a launch file". It is included
here rather than from the world launch files because this is the single place
that knows isaacsim_bringup exists, and because `run_isaacsim.launch.py` has
four sharp edges that are easier to pin down once than to rediscover.

Read from isaacsim_bringup/isaacsim_bringup/run_isaacsim.py, not from its docs:

1. On the `standalone:=` path, `headless`, `gui`, `custom_args` and
   `play_sim_on_start` are ALL ignored -- the branch is a bare
   `Popen(f"{python.sh} {standalone}")`. So `headless:=` here is appended to the
   standalone script's own argv instead, where our argparse handles it.
2. `standalone` is pasted into a `shell=True` command string, which is how the
   arguments below reach the script at all. It also means a path with a space
   in it would break, hence no quoting games: keep asset paths space-free.
3. `use_internal_libs:=true` strips paths that start with /opt/ros/<distro> and
   paths containing the other distro's name. It does NOT strip a colcon
   *overlay* -- your own workspace survives on PYTHONPATH/LD_LIBRARY_PATH and
   gets inherited by Kit's Python 3.12, which is exactly the pollution the
   stripping exists to prevent (Humble's rclpy is a cpython-310 extension).
   `exclude_install_path` is the vendor's mechanism for that, and it defaults
   below to whatever colcon has sourced.
4. `install_path` must be given whenever Isaac Sim is not at the version-derived
   default under $HOME. In a container it is usually /isaac-sim.

Shutdown: run_isaacsim kills the Kit process group with SIGKILL, not SIGTERM.
Kit gets no chance to clean up, and since it is a *grandchild* of ros2 launch,
launch's own process events fire for the node, never for Kit.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _default_exclude_install_path():
    """The colcon overlays currently sourced, as a comma-separated list.

    COLCON_PREFIX_PATH is exactly "the install prefixes on this environment",
    which is exactly what must not follow Kit into Python 3.12. Computing it
    beats hard-coding /ws/install and beats leaving it empty, which is silently
    wrong rather than loudly wrong.
    """
    prefixes = [p for p in os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep) if p]
    return ','.join(prefixes)


def setup(context, *args, **kwargs):
    pkg = get_package_share_directory('turtlebot3_isaacsim')
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    robot = cfg('robot') or os.path.join(
        pkg, 'models', f'turtlebot3_{model}', f'turtlebot3_{model}.usd')

    # See note 1 above: every one of these is the standalone script's argument,
    # not isaacsim_bringup's.
    standalone = [
        os.path.join(pkg, 'scripts', 'turtlebot3_isaacsim.py'),
        '--model', model,
        '--robot', robot,
        '--x-pose', cfg('x_pose'),
        '--y-pose', cfg('y_pose'),
        '--z-pose', cfg('z_pose'),
        '--yaw', cfg('yaw'),
        '--physics-hz', cfg('physics_hz'),
        '--lidar-config', cfg('lidar_config'),
    ]
    if cfg('world'):
        standalone += ['--world', cfg('world')]
    if cfg('namespace'):
        standalone += ['--namespace', cfg('namespace')]
    if cfg('headless').lower() in ('true', '1'):
        standalone.append('--headless')
    if cfg('lidar').lower() in ('false', '0'):
        standalone.append('--no-lidar')

    # See note 2: run_isaacsim pastes this straight into a shell command with
    # no quoting, so a space anywhere in it silently becomes two arguments.
    spaced = [part for part in standalone if ' ' in part]
    if spaced:
        raise RuntimeError(
            f'these arguments contain spaces, which run_isaacsim would split '
            f'when it pastes them into its shell command: {spaced}')
    joined = ' '.join(standalone)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('isaacsim_bringup'),
                'launch', 'run_isaacsim.launch.py')),
            launch_arguments={
                'standalone': joined,
                'install_path': cfg('isaac_install_path'),
                'version': cfg('isaac_version'),
                'ros_distro': cfg('ros_distro'),
                'use_internal_libs': cfg('use_internal_libs'),
                'exclude_install_path': cfg('exclude_install_path'),
                'dds_type': cfg('dds_type'),
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        # --- what to simulate -------------------------------------------------
        DeclareLaunchArgument(
            'world', default_value='',
            description='Path to an environment .usd. Empty means a bare '
                        'ground plane.'),
        DeclareLaunchArgument(
            'robot', default_value='',
            description='Path to the robot .usd. Empty means this package\'s '
                        'models/turtlebot3_$TURTLEBOT3_MODEL/.'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        # -z 0.01, the same drop turtlebot3_gazebo's spawn_turtlebot3 uses.
        DeclareLaunchArgument('z_pose', default_value='0.01'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Topic and frame prefix, for multi-robot.'),

        # --- how to simulate --------------------------------------------------
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Kit with no window. Gazebo\'s gzclient is a '
                        'separate process; Isaac Sim\'s GUI is the same '
                        'process, so this is a flag rather than a launch file.'),
        DeclareLaunchArgument(
            'lidar', default_value='true',
            description='Publish /scan. Set false to tell a lidar problem '
                        'apart from a graph problem.'),
        DeclareLaunchArgument(
            'lidar_config', default_value='turtlebot3_lds',
            description='RTX lidar profile from models/lidar_configs/.'),
        DeclareLaunchArgument('physics_hz', default_value='60.0'),

        # --- where Isaac Sim is ----------------------------------------------
        DeclareLaunchArgument(
            'isaac_install_path', default_value='/isaac-sim',
            description='Isaac Sim install root. isaacsim_bringup otherwise '
                        'guesses a version-derived path under $HOME.'),
        DeclareLaunchArgument('isaac_version', default_value='6.0.1'),
        DeclareLaunchArgument('ros_distro', default_value=os.environ.get('ROS_DISTRO', 'humble')),
        DeclareLaunchArgument(
            'use_internal_libs', default_value='true',
            description='Use the ROS 2 libraries bundled with Isaac Sim. Kit '
                        'runs Python 3.12 and Humble\'s rclpy is a cpython-310 '
                        'extension module, so this stays true.'),
        DeclareLaunchArgument(
            'exclude_install_path', default_value=_default_exclude_install_path(),
            description='Colcon overlays to keep off Kit\'s search paths. '
                        'Defaults to $COLCON_PREFIX_PATH -- see note 3 in the '
                        'module docstring.'),
        DeclareLaunchArgument(
            'dds_type', default_value='',
            description='Empty keeps the surrounding RMW_IMPLEMENTATION.'),

        OpaqueFunction(function=setup),
    ])
