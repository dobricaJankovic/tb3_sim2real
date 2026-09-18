"""RViz — identical for all three backends, and for every mode.

Split out of the old common/nav2.launch.py when `nav` stopped being the only
reason to open a window. With nav:=false slam:=false there is no stack at all,
and on backend:=real there are no local processes either — RViz is then the
only useful thing the workstation can do, and showing the robot, its tf tree
and its live scan is exactly how you find out whether the robot is actually
there. See docs/troubleshooting.md: a half-dead TurtleBot3 publishes /scan
while `odom` does not exist, and that is visible here in a second.

Runs rviz2 directly rather than wrapping nav2_bringup's rviz_launch.py,
because of `use_sim_time`. That file declares no such argument and forwards
nothing to the node, so under either simulator RViz ran on WALL time against
tf stamped in simulated time, and spilled

    Message Filter dropping message: frame 'base_scan' ... the timestamp on
    the message is earlier than all the data in the transform cache

Harmless in itself — but that is the same line, word for word, that real
clock skew between the workstation and the Pi produces (docs/roadmap.md §2),
and a known-bogus copy of a diagnostic makes the real one unreadable. The one
thing the upstream file added was shutting the launch down when the window is
closed, which is kept below.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('tb3_bringup')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'tb3.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        rviz,
        # Closing the window ends the run, as it did upstream: RViz is the only
        # thing with a window under backend:=real, so leaving the launch alive
        # after it exits leaves a stack nobody can see.
        RegisterEventHandler(OnProcessExit(
            target_action=rviz,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited')))),
    ])
