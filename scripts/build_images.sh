#!/usr/bin/env bash
#
# Build the two images, in order. The second is FROM the first, and docker
# compose only builds one image per service, so the base cannot be a compose
# service without pretending it is something you run.
#
#   src/turtlebot3_isaacsim/docker/isaacsim-ros2/Dockerfile.humble
#       -> isaacsim61-humble:ngc
#       Isaac Sim 6.1.0 + ROS 2 Humble on Ubuntu 22.04. Generic: no TurtleBot3,
#       no X11, no workspace. NOT a file in this repository -- it belongs to the
#       turtlebot3_isaacsim package, which is imported here rather than copied,
#       and its container is part of what is imported. Verify it on its own with
#       src/turtlebot3_isaacsim/docker/isaacsim-ros2/verify.sh.
#   docker/Dockerfile                  -> tb3_sim2real/tb3_ros:humble
#       Gazebo Classic's TurtleBot3 worlds, the real robot's drivers, and the
#       workspace -- everything specific to sim-to-real, layered on top.
#
#   scripts/build_images.sh              # both
#   scripts/build_images.sh --base-only  # just the base, e.g. before verify.sh
#
# The Isaac Sim source needs `docker login nvcr.io` plus a one-time licence
# acceptance in a browser for the same NGC org -- the repository is NOT
# anonymously pullable; NGC hands out a token and then 401s on the manifest.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

BASE_IMAGE="${TB3_BASE_IMAGE:-isaacsim61-humble:ngc}"
ISAACSIM_IMAGE="${ISAACSIM_IMAGE:-nvcr.io/nvidia/isaac-sim:6.1.0}"
BASE_CONTEXT=src/turtlebot3_isaacsim/docker/isaacsim-ros2

[ -d "$BASE_CONTEXT" ] || {
    echo "build_images: $BASE_CONTEXT is missing. Run scripts/workspace.sh first." >&2
    exit 1
}

echo "==> base: ${BASE_IMAGE}  (Isaac Sim from ${ISAACSIM_IMAGE})"
docker build \
    -f "${BASE_CONTEXT}/Dockerfile.humble" \
    --build-arg "ISAACSIM_IMAGE=${ISAACSIM_IMAGE}" \
    -t "${BASE_IMAGE}" \
    "${BASE_CONTEXT}/"

if [ "${1-}" = "--base-only" ]; then
    echo "==> base built. Verify with: ${BASE_CONTEXT}/verify.sh ${BASE_IMAGE}"
    exit 0
fi

echo "==> app: tb3_sim2real/tb3_ros:humble"
TB3_BASE_IMAGE="${BASE_IMAGE}" docker compose build

echo "==> done. ./docker/x11-auth.sh, then docker compose up -d"
