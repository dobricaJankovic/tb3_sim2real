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
#
# The ordering above is load-bearing, not a style preference: both services
# bind-mount this file, and Docker's behaviour for a bind-mount source that
# does not exist is to CREATE IT AS A ROOT-OWNED DIRECTORY. Start a container
# first and the cookie slot is permanently occupied by a directory that this
# script cannot then overwrite — see the guard below.

set -euo pipefail

XAUTH="${TB3_XAUTH:-/tmp/tb3_sim2real.docker.xauth}"

# Docker made a directory here (see above), or an earlier root-owned run left a
# file we cannot rewrite. Either way `touch` is about to fail with a bare
# "Permission denied"/"Is a directory", and — far worse — the containers would
# keep starting against a cookie that carries nothing. That failure is silent
# in exactly the way this whole file exists to prevent: Kit logs "Authorization
# required" and "GLFW initialization failed" only as WARNINGS, runs on with no
# window, publishes ROS topics normally, and the healthcheck still reports
# healthy because it only greps the log for AppReady. So say the fix out loud.
if [ -d "$XAUTH" ] || { [ -e "$XAUTH" ] && [ ! -w "$XAUTH" ]; }; then
  what=$([ -d "$XAUTH" ] && echo "a directory" || echo "a file you cannot write")
  cat >&2 <<EOF
error: $XAUTH is $what.

  $(ls -ld "$XAUTH")

A directory here means a container was started before this script ever ran:
Docker creates a root-owned directory in place of a bind-mount source that does
not exist. An unwritable file usually means an earlier run created it as root.

Either way the containers mount it as their XAUTHORITY and silently open no
window: Kit downgrades the X auth failure to a warning, keeps simulating, and
its healthcheck still says healthy.

Clear it, re-key, and restart the container (in that order):

  docker stop isaacsim
  sudo rm -rf $XAUTH
  $0
  docker compose run --rm isaacsim
EOF
  exit 1
fi

touch "$XAUTH"
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -
chmod 644 "$XAUTH"

# `xauth nmerge` writes a temp file and renames it into place, so this path has
# a NEW inode now. A container started earlier still holds the old one through
# its bind mount and will not see this cookie — restart it after re-keying.
echo "Wrote $XAUTH for DISPLAY=$DISPLAY (restart any running container to pick it up)"
