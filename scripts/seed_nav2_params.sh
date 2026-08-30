#!/usr/bin/env bash
# Seed the three Nav2 param files from the stock TurtleBot3 ones.
#
# They start as identical copies. The whole point is that they then DIVERGE —
# the delta between nav2_gazebo.yaml, nav2_isaacsim.yaml and nav2_real.yaml is a
# measurement of the sim-to-real gap, which is why each backend gets its own file
# rather than sharing one with overrides.
#
# Run inside the tb3_ros container.

set -euo pipefail

MODEL="${TURTLEBOT3_MODEL:-burger}"
SRC="/opt/ros/${ROS_DISTRO}/share/turtlebot3_navigation2/param/humble/${MODEL}.yaml"
DEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/tb3_bringup/config"

if [ ! -f "$SRC" ]; then
    echo "error: $SRC not found. Is ros-${ROS_DISTRO}-turtlebot3-navigation2 installed?" >&2
    exit 1
fi

for backend in real gazebo isaacsim; do
    dest="${DEST_DIR}/nav2_${backend}.yaml"
    cp "$SRC" "$dest"
    echo "seeded $dest"
done

echo
echo "Done. Next: set use_sim_time per file (false for real, true for the sims)"
echo "and tune costmap inflation / controller gains per backend."
