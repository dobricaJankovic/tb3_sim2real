"""Isaac Sim backend — attach only.

Isaac Sim runs in its OWN container (docker compose up isaacsim) and reaches us
over DDS. There is deliberately nothing to spawn here.

Do NOT reach for isaacsim_bringup/run_isaacsim.launch.py from the NVIDIA
workspaces: that node does subprocess.Popen("<install>/isaac-sim.sh ...") and so
requires ros2, rclpy and isaac-sim.sh in the same filesystem and namespace —
i.e. the single-container design we rejected (Isaac Sim's image is noble, Humble
is jammy). The pattern we want is carter_navigation.launch.py, which assumes the
simulator is already running and playing somewhere on the DDS domain.

Isaac Sim supplies: /clock, /scan, /odom, /joint_states, tf odom->base_footprint,
/cmd_vel sink — all from OmniGraph nodes built by isaac/scripts/tb3_sim.py.
Those nodes only produce data while the sim is PLAYING.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'wait_for_sim', default_value='true',
            description='Block until /clock appears, so the logs say "no simulator" '
                        'instead of Nav2 silently hanging'),
        DeclareLaunchArgument(
            'pointcloud_to_laserscan', default_value='false',
            description='Set true only if the Isaac scene publishes PointCloud2 '
                        'instead of a LaserScan. Prefer the LaserScan writer so '
                        '/scan matches the real LDS-02 and no extra node is needed.'),

        Node(
            package='tb3_bringup',
            executable='wait_for_sim',
            name='wait_for_sim',
            output='screen',
            condition=IfCondition(LaunchConfiguration('wait_for_sim')),
        ),

        # Fallback path, off by default. Topic names and frame here are Carter's
        # and are WRONG for TB3 — fix if we ever turn this on.
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            condition=IfCondition(LaunchConfiguration('pointcloud_to_laserscan')),
            remappings=[('cloud_in', '/point_cloud'), ('scan', '/scan')],
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'base_scan',
                'transform_tolerance': 0.01,
                'min_height': -0.1,
                'max_height': 0.5,
                'angle_min': -3.1415,
                'angle_max': 3.1415,
                'angle_increment': 0.0175,
                'scan_time': 0.2,
                'range_min': 0.12,
                'range_max': 3.5,
                'use_inf': True,
            }],
        ),
    ])
