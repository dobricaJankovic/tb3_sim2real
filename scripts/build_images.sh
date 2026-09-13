#!/usr/bin/env bash
#
# Build the two images, in order. The second is FROM the first, and docker
# compose only builds one image per service, so the base cannot be a compose
# service without pretending it is something you run.
#
#   docker/isaacsim-ros2/Dockerfile.humble -> isaacsim6-humble:ngc
#       Isaac Sim 6.0 + ROS 2 Humble on Ubuntu 22.04. Generic: no TurtleBot3,
#       no X11, no workspace. Verify it on its own with
#       docker/isaacsim-ros2/verify.sh.
#   docker/ros/Dockerfile                  -> tb3_sim2real/tb3_ros:humble
#       Everything specific to this repo, layered on top.
#
#   scripts/build_images.sh              # both
#   scripts/build_images.sh --base-only  # just the base, e.g. before verify.sh
#
# The Isaac Sim source needs `docker login nvcr.io` plus a one-time licence
# acceptance in a browser for the same NGC org -- the repository is NOT
# anonymously pullable; NGC hands out a token and then 401s on the manifest.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

BASE_IMAGE="${TB3_BASE_IMAGE:-isaacsim6-humble:ngc}"
ISAACSIM_IMAGE="${ISAACSIM_IMAGE:-nvcr.io/nvidia/isaac-sim:6.0.1}"

echo "==> base: ${BASE_IMAGE}  (Isaac Sim from ${ISAACSIM_IMAGE})"
docker build \
    -f docker/isaacsim-ros2/Dockerfile.humble \
    --build-arg "ISAACSIM_IMAGE=${ISAACSIM_IMAGE}" \
    -t "${BASE_IMAGE}" \
    docker/isaacsim-ros2/

if [ "${1-}" = "--base-only" ]; then
    echo "==> base built. Verify with: docker/isaacsim-ros2/verify.sh ${BASE_IMAGE}"
    exit 0
fi

echo "==> app: tb3_sim2real/tb3_ros:humble"
TB3_BASE_IMAGE="${BASE_IMAGE}" docker compose build

echo "==> done. ./docker/x11-auth.sh, then docker compose up -d"
