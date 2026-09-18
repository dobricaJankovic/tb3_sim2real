"""Gazebo Classic backend.

Supplies: /clock, /scan, /odom, /joint_states, tf odom->base_footprint,
/cmd_vel sink — all from gazebo_ros plugins baked into the turtlebot3_gazebo
model SDF — plus /ground_truth/odom, the true body pose, which this file adds.

Deliberately does not include robot_state_publisher: that is in common/ and is
shared with the other two backends.

`world` arrives already resolved to a .world path — bringup.launch.py does the
registry lookup once, for whichever backend is running, so that `world:=` means
the same environment everywhere. The same goes for the spawn pose.

The robot is spawned with spawn_entity.py directly rather than through
turtlebot3_gazebo's spawn_turtlebot3.launch.py, which takes only x_pose and
y_pose and pins z to 0.01. The manifest's spawn carries a yaw, and Isaac Sim
honours it; a Gazebo backend that silently dropped it would put the two
simulators at different headings from the same manifest, which is exactly the
class of divergence this repository is meant to make impossible.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# P3D reports a link's true pose in the world frame, as nav_msgs/Odometry on
# its own topic. It is a sensor: it reads the physics state and writes a
# message, and changes no dynamics.
#
# It is here because /odom on this backend is NOT the body pose. The
# turtlebot3 diff-drive plugin integrates the wheel joints, so /odom and a
# forward integration of /joint_states agree to five decimal places by
# construction, and wheel slip is invisible in both.
#
# Isaac Sim publishes the same topic, from the chassis-prim node that used to
# feed its /odom until 2026-09-18. Both simulators therefore now carry a
# drifting /odom and a true /ground_truth/odom, and mean the same thing by
# each name. The origins still differ -- this plugin reports world-absolute
# coordinates, Isaac's is relative to the spawn pose -- so consumers use
# deltas.
#
# Loaded at spawn rather than by -s: gazebo_ros_state is a WORLD plugin, and
# -s is the SYSTEM plugin loader, so it is accepted and silently never
# attaches. Putting it in the world file is not an option either -- the .world
# is generated from the manifest and check_worlds.py would report it as drift.
GROUND_TRUTH_PLUGIN = """
    <plugin name="ground_truth" filename="libgazebo_ros_p3d.so">
      <ros>
        <namespace>/</namespace>
        <remapping>odom:=ground_truth/odom</remapping>
      </ros>
      <frame_name>world</frame_name>
      <body_name>base_footprint</body_name>
      <update_rate>50.0</update_rate>
      <gaussian_noise>0.0</gaussian_noise>
    </plugin>
"""


def with_ground_truth(sdf_path):
    """Return a copy of the robot SDF with the P3D ground-truth plugin added.

    A copy, in a temporary file, rather than an edit: turtlebot3_gazebo's
    model.sdf is a package's installed data and belongs to that package.
    """
    with open(sdf_path) as f:
        sdf = f.read()
    if 'libgazebo_ros_p3d.so' in sdf:
        return sdf_path
    tail = sdf.rindex('</model>')
    out = tempfile.NamedTemporaryFile(
        mode='w', suffix='.sdf', prefix='tb3_ground_truth_', delete=False)
    out.write(sdf[:tail] + GROUND_TRUTH_PLUGIN + sdf[tail:])
    out.close()
    return out.name


def generate_launch_description():
    gazebo_ros = get_package_share_directory('gazebo_ros')
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    robot_sdf = with_ground_truth(os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models', f'turtlebot3_{model}', 'model.sdf'))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            description='Path to the .world gzserver should load'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.01'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Skip gzclient; gzserver still runs'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=UnlessCondition(LaunchConfiguration('headless')),
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', model,
                '-file', robot_sdf,
                '-x', LaunchConfiguration('x_pose'),
                '-y', LaunchConfiguration('y_pose'),
                '-z', LaunchConfiguration('z_pose'),
                '-Y', LaunchConfiguration('yaw'),
            ],
        ),
    ])
