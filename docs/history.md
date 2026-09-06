# History

Append-only log of problems/fixes and concepts/ideas from work sessions,
kept for the user's own reference. New conversations: do NOT read this whole
file for context — only append a new dated entry at the end after finishing
meaningful work. (See `CLAUDE.md`.) Everything before this file's inception
is in `git log`.

---

## 2026-09-06 — gzclient X11 auth crash on `backend:=gazebo`

**Problem:** `ros2 launch tb3_bringup bringup.launch.py backend:=gazebo
nav:=false` spawned the robot fine (`robot_state_publisher`, `gzserver`,
`spawn_entity` all succeeded — `/cmd_vel`, `/odom` live) but `gzclient`
aborted (SIGABRT) with `Authorization required, but no authorization
protocol specified`.

**Root cause:** Both the `isaacsim` and `tb3_ros` containers run as root,
but the host X server's access control is per-UID (`xhost` shows
`SI:localuser:<user>`, not root). `docker-compose.yml` mounted
`/tmp/.X11-unix` and passed `DISPLAY`, but never gave the container an
Xauthority cookie, so root's X11 clients couldn't authenticate. `gzserver`
only warns and keeps going (it's headless-capable); `gzclient` actually
needs to open a window and aborts without one.

**Fix:** Added `docker/x11-auth.sh` — generates a docker-specific Xauthority
cookie from the host's current `$DISPLAY`, re-keyed under a wildcard family
so it authenticates regardless of the container's hostname. `docker-compose.yml`
now mounts that cookie to `/root/.Xauthority` and sets `XAUTHORITY` for both
services. Run the script once per X session before `docker compose up/run`
(re-run after an X session restart — the cookie rotates on login).

Verified by opening a bare `gzclient` in a fresh container and confirming
its window actually appeared on the host X server (`xwininfo -root -tree`
showed the full Gazebo widget tree, no auth error in the log).

**Also:** updated `docs/troubleshooting.md`'s existing X11 bullet, which
recommended `xhost +local:docker` — that permanently loosens host X access
control instead of scoping a single cookie file, and doesn't get mentioned
again here for that reason.
