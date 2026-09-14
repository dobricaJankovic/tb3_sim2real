#!/usr/bin/env bash
#
# Fetch the source dependencies listed in tb3_sim2real.repos into src/.
#
#   scripts/workspace.sh            # clone or update
#
# Idempotent: re-running updates the checkouts to the pinned versions. src/ is
# gitignored -- these are imported, not vendored.
#
#   src/turtlebot3_isaacsim        the Isaac Sim backend, a peer of
#                                  turtlebot3_gazebo, developed in its own
#                                  repository. Needs an SSH key with access.
#   src/IsaacSim-ros_workspaces    NVIDIA's, for isaacsim_bringup alone.
#
# vcstool comes from the image (python3-vcstool); on the host, `pipx install
# vcstool` or run this inside the container.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

command -v vcs >/dev/null || {
    echo "workspace: vcstool not found. Run this inside the container:" >&2
    echo "  docker compose exec tb3_ros /entrypoint.sh scripts/workspace.sh" >&2
    exit 1
}

mkdir -p src
# Not --recursive: the only submodules in NVIDIA's tree belong to the packages
# we are about to ignore, and they are ~300 MB of MoveIt and ros2_control.
vcs import src --skip-existing < tb3_sim2real.repos

# NVIDIA's repository carries nine packages per distro, in humble_ws/ and
# jazzy_ws/ side by side -- so `isaacsim_bringup` appears TWICE, which colcon
# rejects as a duplicate package name before it builds anything. We include
# exactly one launch file out of it (run_isaacsim.launch.py), so hide every
# package except our distro's copy. COLCON_IGNORE is colcon's own mechanism:
# no fork, no sparse checkout, and it is untracked in their tree so a later
# `vcs import` does not disturb it.
keep="src/IsaacSim-ros_workspaces/${ROS_DISTRO:-humble}_ws/src/isaacsim_bringup"
find src/IsaacSim-ros_workspaces -name package.xml -printf '%h\n' \
  | grep -vx "$keep" \
  | xargs -r -I{} touch {}/COLCON_IGNORE
[ -d "$keep" ] || { echo "workspace: $keep is missing -- wrong ROS_DISTRO?" >&2; exit 1; }
rm -f "$keep/COLCON_IGNORE"

echo
vcs export src 2>/dev/null | sed -n '1,40p' || true
echo
echo "workspace: ready. Next: docker compose exec tb3_ros /entrypoint.sh colcon build --symlink-install"
