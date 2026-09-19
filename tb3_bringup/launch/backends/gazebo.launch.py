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
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tb3_bringup import worlds


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


# The wheels' coefficient of friction, as upstream ships it:
#
#     <!-- This friction pamareter don't contain reliable data!! -->
#     <mu>100000.0</mu>
#
# That comment is ROBOTIS's, verbatim, typo included. 100000 is not a physical
# value, it is a "must not slip" sentinel, and it is the whole of why Gazebo
# shows no wheel slip while Isaac Sim and the real robot do. The manifest
# declares a real coefficient now (tb3_bringup.worlds, `surfaces`), and the
# wheel is set to the SAME number as the floor so that ODE's rule for combining
# a contact pair -- believed to be the minimum, unverified here -- cannot
# change the effective value.
#
# What is deliberately NOT touched:
#
#   slip1/slip2, which stay 0.0. These are ODE's force-dependent slip, an extra
#   compliance proportional to applied force; 0.0 means ordinary Coulomb
#   friction, NOT "slipping disabled". Coulomb slip happens as soon as mu*N is
#   exceeded, and with mu = 100000 it never was. Turning FDS on would add a
#   second free parameter with nothing to set it from, and the point of this
#   change is that exactly ONE thing moves.
#
#   caster_back_joint, which upstream makes a BALL joint, so Gazebo's caster
#   ROLLS where the real robot and Isaac Sim drag a skid. That is a structural
#   difference in the model rather than a coefficient, it is upstream's asset,
#   and it is left alone -- recorded as a limitation that would explain a
#   residual disagreement rather than patched over.
WHEEL_COLLISIONS = ('wheel_left_collision', 'wheel_right_collision')


def with_surfaces(sdf_path, mu):
    """Return a copy of the robot SDF with ground truth added and wheel mu set.

    A copy, in a temporary file, rather than an edit: turtlebot3_gazebo's
    model.sdf is a package's installed data and belongs to that package.

    Both edits in one pass, so there is one temp file per run rather than one
    per change, and one place that knows the robot SDF is rewritten at all.
    """
    with open(sdf_path) as f:
        sdf = f.read()

    if mu is not None:
        for name in WHEEL_COLLISIONS:
            start = sdf.find('<collision name="{}">'.format(name))
            if start < 0:
                raise RuntimeError(
                    "turtlebot3_gazebo's model.sdf has no {} — the wheel "
                    'friction cannot be set, and leaving it at upstream\'s '
                    'unreliable 100000 silently would make this backend the '
                    'only one that cannot slip.'.format(name))
            end = sdf.index('</collision>', start)
            block = sdf[start:end]
            patched, n = re.subn(r'<mu>[^<]*</mu>',
                                 '<mu>{:g}</mu>'.format(mu), block)
            patched, n2 = re.subn(r'<mu2>[^<]*</mu2>',
                                  '<mu2>{:g}</mu2>'.format(mu), patched)
            if not (n and n2):
                raise RuntimeError(
                    '{} declares no <mu>/<mu2> to replace'.format(name))
            sdf = sdf[:start] + patched + sdf[end:]

    if 'libgazebo_ros_p3d.so' not in sdf:
        tail = sdf.rindex('</model>')
        sdf = sdf[:tail] + GROUND_TRUTH_PLUGIN + sdf[tail:]

    out = tempfile.NamedTemporaryFile(
        mode='w', suffix='.sdf', prefix='tb3_robot_', delete=False)
    out.write(sdf)
    out.close()
    return out.name


def spawn(context, *args, **kwargs):
    """The robot, rewritten to carry this world's floor friction.

    An OpaqueFunction because the SDF has to be written before spawn_entity is
    handed a path, and the value comes from `wheel_mu`, which is a launch
    argument and so is not resolved until now.
    """
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    mu = LaunchConfiguration('wheel_mu').perform(context)
    robot_sdf = with_surfaces(
        os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                     'models', f'turtlebot3_{model}', 'model.sdf'),
        float(mu) if mu else None)
    return [Node(
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
    )]


def generate_launch_description():
    gazebo_ros = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            description='Path to the .world gzserver should load'),
        # The floor's coefficient, from the manifest, applied to the wheel too
        # so the pair resolves to it whatever ODE's combination rule is.
        # bringup.launch.py passes world.surface['mu']; the default is for
        # running this file directly. Empty means "leave upstream's 100000
        # alone", which no experiment should want but a bisect might.
        DeclareLaunchArgument('wheel_mu',
                              default_value=str(worlds.FLOOR_MU)),
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
        OpaqueFunction(function=spawn),
    ])
