#!/usr/bin/env bash
# Generates a docker-specific Xauthority cookie file so root-uid containers
# can open GUI windows (gzclient, RViz, Isaac Sim) on the host X server
# without permanently loosening access control via `xhost +`.
#
# Host X access control is per-UID (`xhost` shows `SI:localuser:<you>`), but
# containers here run as root, so plain `/tmp/.X11-unix` + DISPLAY mounts
# aren't enough — the client fails with "Authorization required, but no
# authorization protocol specified" and aborts.
#
# Run this once per X session before `docker compose run`/`up`. Re-run if the
# X session restarts (cookie rotates on login).
#
#   ./docker/x11-auth.sh
#   docker compose run --rm tb3_ros

set -euo pipefail

XAUTH="${TB3_XAUTH:-/tmp/tb3_sim2real.docker.xauth}"

touch "$XAUTH"
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -
chmod 644 "$XAUTH"

echo "Wrote $XAUTH for DISPLAY=$DISPLAY"
