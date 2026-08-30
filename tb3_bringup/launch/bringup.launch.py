"""Single entry point for all three backends.

    ros2 launch tb3_bringup bringup.launch.py backend:=gazebo
    ros2 launch tb3_bringup bringup.launch.py backend:=isaacsim
    ros2 launch tb3_bringup bringup.launch.py backend:=real

OpaqueFunction rather than IfCondition, so the backend argument is a plain
Python string we can branch on. That also lets us derive use_sim_time from the
backend instead of making the operator remember it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

BACKENDS = ('real', 'gazebo', 'isaacsim')


def setup(context, *args, **kwargs):
    pkg = get_package_share_directory('tb3_bringup')

    backend = LaunchConfiguration('backend').perform(context)
    if backend not in BACKENDS:
        raise RuntimeError(
            f"backend must be one of {BACKENDS}, got '{backend}'"
        )

    # The one thing that should never be set by hand again.
    use_sim_time = 'false' if backend == 'real' else 'true'

    def inc(rel, **extra):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', rel)),
            launch_arguments={'use_sim_time': use_sim_time, **extra}.items(),
        )

    actions = [
        # Same URDF drives the kinematic tree in all three worlds. Each backend
        # only has to supply odom->base_footprint plus sensor topics.
        inc('common/state_publisher.launch.py'),
        inc(f'backends/{backend}.launch.py'),
    ]

    if LaunchConfiguration('nav').perform(context) == 'true':
        actions.append(inc(
            'common/nav2.launch.py',
            params_file=os.path.join(pkg, 'config', f'nav2_{backend}.yaml'),
            map=LaunchConfiguration('map').perform(context),
            rviz=LaunchConfiguration('rviz').perform(context),
        ))

    return actions


def generate_launch_description():
    pkg = get_package_share_directory('tb3_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'backend',
            description='Which robot to bring up: real | gazebo | isaacsim'),
        DeclareLaunchArgument(
            'nav', default_value='true',
            description='Also start the Nav2 stack'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Also start RViz'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg, 'maps', 'map.yaml'),
            description='Occupancy map for Nav2'),
        OpaqueFunction(function=setup),
    ])
