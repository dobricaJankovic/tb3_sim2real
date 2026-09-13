#!/usr/bin/env bash
#
# Build this package's robot assets from turtlebot3_description.
#
#     scripts/build_models.sh [burger|waffle|waffle_pi]
#
# Run it from a ROS 2 shell, NOT from Kit's. It does the two things that have to
# happen on either side of the interpreter split:
#
#   here, under ROS 2      expand the xacro template and locate
#                          turtlebot3_description's meshes (needs the package index)
#   there, under Kit       convert URDF -> USD (needs Isaac Sim)
#
# The USD it produces is a build artifact. It is gitignored, so a fresh clone has
# none and this has to be run once before any launch file will start.
set -euo pipefail

MODEL="${1:-${TURTLEBOT3_MODEL:-burger}}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(dirname "$HERE")"

case "$MODEL" in
  burger|waffle|waffle_pi) ;;
  *) echo "error: unknown model '$MODEL' (burger|waffle|waffle_pi)" >&2; exit 1 ;;
esac

# Kit's interpreter. ISAACSIM_PYTHON wins; then a wrapper on PATH; then the
# usual install roots.
if [ -n "${ISAACSIM_PYTHON:-}" ]; then
  PYTHON="$ISAACSIM_PYTHON"
elif command -v isaacsim-python >/dev/null 2>&1; then
  PYTHON="isaacsim-python"
elif [ -x "${ISAACSIM_PATH:-/isaac-sim}/python.sh" ]; then
  PYTHON="${ISAACSIM_PATH:-/isaac-sim}/python.sh"
else
  echo "error: cannot find Isaac Sim's python.sh." >&2
  echo "  Set ISAACSIM_PYTHON=/path/to/python.sh or ISAACSIM_PATH=/isaac-sim." >&2
  exit 1
fi

DESCRIPTION="$(ros2 pkg prefix --share turtlebot3_description)"
URDF_IN="$DESCRIPTION/urdf/turtlebot3_${MODEL}.urdf"
URDF_OUT="$(mktemp -d)/turtlebot3_${MODEL}.urdf"

# namespace:= expands the template's ${namespace} to nothing. Without this every
# frame comes out literally named "${namespace}base_footprint".
echo "expanding $URDF_IN"
xacro "$URDF_IN" namespace:= > "$URDF_OUT"

echo "importing with $PYTHON"
"$PYTHON" "$HERE/import_turtlebot3.py" \
  --model "$MODEL" \
  --urdf "$URDF_OUT" \
  --description-share "$DESCRIPTION" \
  --output "$PKG/models/turtlebot3_${MODEL}/turtlebot3_${MODEL}.usd"

rm -rf "$(dirname "$URDF_OUT")"

echo
echo "Built $PKG/models/turtlebot3_${MODEL}/"
echo "If this package is installed rather than run from source, rebuild it now:"
echo "    colcon build --packages-select turtlebot3_isaacsim"
