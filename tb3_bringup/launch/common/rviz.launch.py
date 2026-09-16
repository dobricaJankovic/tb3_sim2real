"""RViz — identical for all three backends, and for every mode.

Split out of the old common/nav2.launch.py when `nav` stopped being the only
reason to open a window. With nav:=false slam:=false there is no stack at all,
and on backend:=real there are no local processes either — RViz is then the
only useful thing the workstation can do, and showing the robot, its tf tree
and its live scan is exactly how you find out whether the robot is actually
there. See docs/troubleshooting.md: a half-dead TurtleBot3 publishes /scan
while `odom` does not exist, and that is visible here in a second.

Wraps nav2_bringup's rviz_launch.py rather than running rviz2 directly,
because that file also wires the lifecycle-driven exit handling.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('tb3_bringup')
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'rviz_launch.py')),
            launch_arguments={
                'namespace': '',
                'use_namespace': 'False',
                'rviz_config': os.path.join(pkg, 'rviz', 'tb3.rviz'),
            }.items(),
        ),
    ])
