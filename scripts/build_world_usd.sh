#!/usr/bin/env bash
# Build a world's Isaac Sim stage: worlds/<name>/isaac/<name>.usd
#
#   scripts/build_world_usd.sh [world]      # default: turtlebot3_world
#
# Counterpart of scripts/build_world.py, which produces the Gazebo .world from
# the same worlds/<name>/world.yaml. Both read the same manifest and the same
# meshes; that is what keeps the two backends from drifting.
#
# This wrapper exists for one reason: the isaacsim container runs as uid 1234
# (NVIDIA's image ends `USER isaac-sim`) while your checkout is owned by you, so
# the container cannot create its own output directory. Git records no directory
# modes, so this cannot be fixed by committing anything — it has to happen at
# build time, on every clone.
#
# The directory is made world-writable rather than the whole world directory, so
# world.yaml and meshes/ keep normal permissions. Files inside end up owned by
# uid 1234, but you can still delete them because the parent directory is yours
# and writable — which is what makes this better than chmod-ing the world dir.
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
chmod 777 "$DIR/isaac"

cd "$REPO"
exec docker compose run --rm \
    -e PYTHONUNBUFFERED=1 \
    -e "WORLD=$WORLD" \
    --entrypoint /isaac-sim/python.sh \
    isaacsim /scripts/build_world_usd.py --world "$WORLD"
