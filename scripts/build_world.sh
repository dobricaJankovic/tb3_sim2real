#!/usr/bin/env bash
# Regenerate BOTH of a world's representations from its manifest.
#
#   scripts/build_world.sh                    # turtlebot3_world
#   scripts/build_world.sh empty_stage
#   scripts/build_world.sh /path/to/my_office # a world outside the registry
#   scripts/build_world.sh --all
#
# One command on purpose. The Gazebo .world and the Isaac Sim .usd are two
# representations of one world.yaml, and regenerating one without the other is
# the single way the two backends can come to describe different rooms. Running
# them separately is still possible -- scripts/build_world.py is pure Python and
# needs no container -- but this is the documented way, and the one the
# provenance digest in each generated file assumes.
#
#   build_world.py       host or container, PyYAML only  -> <name>.world
#   build_world_usd.py   Isaac Sim's interpreter         -> isaac/<name>.usd
#
# The USD half goes through `docker compose run --rm ... isaacsim-python`, which
# runs ros-isolate to take the system ROS 2 back off Kit's search paths. Calling
# /isaac-sim/python.sh directly from this image loads the wrong interpreter's
# modules and Kit aborts. `run --rm` rather than `exec`, deliberately and unlike
# the rest of the workflow: this is a one-shot batch job that needs no workspace
# overlay and no running simulator, so it does not require `up -d` first. It
# still gets the warm Kit cache, which lives in named volumes.
#
# The container runs as root, so worlds/<name>/isaac/ ends up root-owned in your
# checkout. It is gitignored, and `rm` on it wants sudo -- clear it from inside
# the container instead:
#
#   docker compose run --rm tb3_ros rm -rf /worlds/<name>/isaac
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

WORLD="${1:-turtlebot3_world}"

if [ "$WORLD" = "--all" ]; then
    python3 scripts/build_world.py --all
    for w in $(python3 scripts/build_world.py --list); do
        "$0" "$w"
    done
    exit 0
fi

# Resolve name-or-path the same way the launch files do, so this script and
# `world:=` can never disagree about which directory is meant.
DIR="$(python3 - "$WORLD" <<'PY'
import sys, os
sys.path.insert(0, 'tb3_bringup')
from tb3_bringup import worlds
try:
    print(worlds.locate(sys.argv[1]))
except RuntimeError as e:
    sys.exit('build_world: %s' % e)
PY
)"

python3 scripts/build_world.py "$WORLD"

# Created on the host so it belongs to you, not to the container's root.
mkdir -p "$DIR/isaac"

exec docker compose run --rm \
    -e PYTHONUNBUFFERED=1 \
    tb3_ros isaacsim-python /repo/scripts/build_world_usd.py --world "$WORLD"
