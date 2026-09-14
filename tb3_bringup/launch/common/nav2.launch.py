"""Nav2 + RViz — identical for all three backends.

Everything that varies is passed in by bringup.launch.py: the params file
(config/nav2_<backend>.yaml where one exists, else config/nav2_params.yaml) and
the map (the world manifest's, unless map:= overrides it). Structure is the
same for every backend; tuning is not, once it diverges, and that difference is
precisely the sim-to-real gap we want to be able to measure.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('tb3_bringup')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('map', description='Occupancy map for Nav2'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'rviz_launch.py')),
            condition=IfCondition(LaunchConfiguration('rviz')),
            launch_arguments={
                'namespace': '',
                'use_namespace': 'False',
                'rviz_config': os.path.join(pkg, 'rviz', 'tb3.rviz'),
            }.items(),
        ),
    ])
