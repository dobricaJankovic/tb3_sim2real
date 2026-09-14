#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

# Loud about the two settings that cause silent failures between containers.
echo "ROS_DISTRO=${ROS_DISTRO}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-unset}"

exec "$@"
