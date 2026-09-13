#!/usr/bin/env bash
# Generates a docker-specific Xauthority cookie file so containers running
# under a different uid than yours can open GUI windows (gzclient, RViz,
# Isaac Sim) on the host X server without permanently loosening access
# control via `xhost +`.
#
# Host X access control is per-UID (`xhost` shows `SI:localuser:<you>`), and the
# container does not run as you — it runs as root. So plain `/tmp/.X11-unix`
# + DISPLAY mounts aren't enough; the client fails with "Authorization
# required, but no authorization protocol specified" and aborts. Presenting a
# valid MIT-MAGIC-COOKIE via XAUTHORITY satisfies the server regardless of uid.
#
# One cookie, mounted at /root/.Xauthority. There were two paths while Isaac Sim
# had its own container as uid 1234; see docker-compose.yml.
#
# Run this once per X session before `docker compose up`. Re-run if the X
# session restarts (cookie rotates on login).
#
#   ./docker/x11-auth.sh
#   docker compose up -d
#
# The ordering above is load-bearing, not a style preference: the compose file
# bind-mounts this file, and Docker's behaviour for a bind-mount source that
# does not exist is to CREATE IT AS A ROOT-OWNED DIRECTORY. Start the container
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

Either way the container mounts it as XAUTHORITY and silently opens no window:
Kit downgrades the X auth failure to a warning and keeps simulating.

Clear it, re-key, and restart the container (in that order):

  docker compose down
  sudo rm -rf $XAUTH
  $0
  docker compose up -d
EOF
  exit 1
fi

touch "$XAUTH"
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -
chmod 644 "$XAUTH"

# `xauth nmerge` writes a temp file and renames it into place, so this path has
# a NEW inode now. A container started earlier still holds the old one through
# its bind mount and will not see this cookie — `docker compose restart tb3_ros`
# after re-keying. Confirmed the hard way on 2026-09-13.
echo "Wrote $XAUTH for DISPLAY=$DISPLAY (restart any running container to pick it up)"
