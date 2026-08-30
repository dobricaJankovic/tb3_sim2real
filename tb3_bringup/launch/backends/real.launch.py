"""Real TurtleBot3.

Supplies: /scan, /odom, /joint_states, tf odom->base_footprint, /cmd_vel sink.
Does NOT supply /clock — wall time, so use_sim_time is false for this backend.

Mirrors turtlebot3_bringup/launch/robot.launch.py, minus the state publisher
(that lives in common/ and is shared).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# LDS_MODEL -> (driver package, launch file)
LIDAR = {
    'LDS-01': ('hls_lfcd_lds_driver', 'hlds_laser.launch.py'),
    'LDS-02': ('ld08_driver', 'ld08.launch.py'),
    'LDS-03': ('coin_d4_driver', 'single_lidar_node.launch.py'),
}


def setup(context, *args, **kwargs):
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    lds = os.environ.get('LDS_MODEL', 'LDS-02')
    if lds not in LIDAR:
        raise RuntimeError(f'LDS_MODEL must be one of {tuple(LIDAR)}, got {lds!r}')

    lidar_pkg, lidar_launch = LIDAR[lds]
    tb3_param = os.path.join(
        get_package_share_directory('turtlebot3_bringup'),
        'param', 'humble', f'{model}.yaml',
    )
    usb_port = LaunchConfiguration('usb_port')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(lidar_pkg), 'launch', lidar_launch)),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ),
        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[tb3_param],
            arguments=['-i', usb_port],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('usb_port', default_value='/dev/ttyACM0',
                              description='OpenCR serial port'),
        OpaqueFunction(function=setup),
    ])
