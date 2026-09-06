#!/usr/bin/env bash
# Generates a docker-specific Xauthority cookie file so containers running
# under a different uid than yours can open GUI windows (gzclient, RViz,
# Isaac Sim) on the host X server without permanently loosening access
# control via `xhost +`.
#
# Host X access control is per-UID (`xhost` shows `SI:localuser:<you>`), and
# neither container runs as you — tb3_ros is root, isaacsim is uid 1234
# (NVIDIA's Dockerfile ends with `USER isaac-sim`). So plain `/tmp/.X11-unix`
# + DISPLAY mounts aren't enough; the client fails with "Authorization
# required, but no authorization protocol specified" and aborts. Presenting a
# valid MIT-MAGIC-COOKIE via XAUTHORITY satisfies the server regardless of uid.
#
# The cookie must land somewhere the container's uid can actually read: see the
# per-service XAUTHORITY paths in docker-compose.yml (they differ on purpose).
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
