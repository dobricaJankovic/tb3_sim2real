#!/usr/bin/env bash
# Build a world's Isaac Sim stage: worlds/<name>/isaac/<name>.usd
#
#   scripts/build_world_usd.sh [world]      # default: turtlebot3_world
#
# Counterpart of scripts/build_world.py, which produces the Gazebo .world from
# the same worlds/<name>/world.yaml. Both read the same manifest and the same
# meshes; that is what keeps the two backends from drifting.
#
# This wrapper exists to create the output directory before the container needs
# it, and to go through `isaacsim-python` rather than /isaac-sim/python.sh --
# the wrapper runs ros-isolate, which takes the system ROS 2 back off Kit's
# search paths. Calling python.sh directly from this image loads the wrong
# interpreter's modules and Kit aborts.
#
# `run --rm` rather than `exec`, deliberately and unlike the rest of the
# workflow (see docs/setup.md): this is a one-shot batch job that needs no
# workspace overlay and no running simulator, so it should not require
# `docker compose up -d` first. It still gets the warm Kit cache, which lives in
# named volumes.
#
# The container runs as root, so the generated worlds/<name>/isaac/<name>.usd is
# root-owned in your checkout. It is gitignored, and `rm` on it wants sudo --
# delete it from inside the container instead:
#
#   docker compose run --rm tb3_ros rm -rf /worlds/<name>/isaac
set -euo pipefail

WORLD="${1:-turtlebot3_world}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="$REPO/worlds/$WORLD"

if [[ ! -f "$DIR/world.yaml" ]]; then
    echo "build_world_usd: no manifest at $DIR/world.yaml" >&2
    echo "  available: $(cd "$REPO/worlds" 2>/dev/null && ls -d */ 2>/dev/null | tr -d / | tr '\n' ' ')" >&2
    exit 1
fi

mkdir -p "$DIR/isaac"

cd "$REPO"
exec docker compose run --rm \
    -e PYTHONUNBUFFERED=1 \
    -e "WORLD=$WORLD" \
    tb3_ros isaacsim-python /scripts/build_world_usd.py --world "$WORLD"
