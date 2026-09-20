#!/usr/bin/env bash
# Clear the Fast DDS shared-memory leftovers that make nodes hang with no error.
#
#   scripts/dds_clean.sh            # refuse if anything ROS is still running
#   scripts/dds_clean.sh --kill     # kill orphaned ROS/Gazebo processes first
#   scripts/dds_clean.sh --dry-run  # say what would go, remove nothing
#
# Run it INSIDE the container (`docker compose exec tb3_ros bash`), or on the
# host -- /dev/shm is the same directory either way, see below.
#
# ## What goes wrong
#
# Fast DDS's shared-memory transport keeps one segment and one named mutex per
# port under /dev/shm: fastrtps_port<N>, fastrtps_port<N>_el and
# sem.fastrtps_port<N>_mutex. A participant that dies without closing -- a
# Ctrl-C'd `ros2 launch` that orphans its nodes, a killed gzserver, a container
# torn down mid-run -- leaves those behind, and can leave the mutex HELD. The
# next participant that hashes onto that port blocks forever trying to take it.
#
# It blocks before rclcpp emits its first log line, so the symptom is total
# silence: the process is alive, its /root/.ros/log/<node>_<pid>.log is zero
# bytes, and it never appears in `ros2 node list`. Downstream you see only
#
#     [lifecycle_manager] Waiting for service map_server/get_state...
#
# repeating until you give up. Measured on 2026-09-20: 546 stale segments dating
# back four days, `backend:=gazebo world:=turtlebot3_world` fine on its own
# because it is four participants and they got lucky, the same command with
# nav:=true dead because it adds eleven more at once and map_server, amcl,
# controller_server, bt_navigator, behavior_server, waypoint_follower and
# velocity_smoother all landed on poisoned ports. Clearing /dev/shm took the
# whole stack to "Managed nodes are active" in four seconds.
#
# ## Why it accumulates here in particular
#
# docker-compose.yml mounts `- /dev:/dev`, so /dev/shm inside the container IS
# the host's (the comment there says so, measured 2026-09-13). Segments outlive
# `docker compose down`. Every aborted run ratchets the count up.
#
# ## What this does NOT touch, and why
#
#   carb-*, carbonite-*   Isaac Sim's. Kit names a segment after its own PID and
#                         reuses a root-owned leftover; one owned by anyone else
#                         aborts it before it starts. Not ours to delete.
#   ad_*, libpod_lock,    other tenants of a SHARED /dev/shm -- see above, this
#   lttng-ust-*           is the host's directory, not this container's.
#   /tmp/fastdds_peers.xml  the cross-subnet unicast profile the entrypoint
#                         writes from TB3_DDS_PEERS. See below.
#
# ## Effect on the real robot: none on the link, everything on the local half
#
# Shared memory is a SAME-HOST transport. The robot is a second machine
# (docs/network.md), so every packet between it and this workstation is UDP over
# the network -- unicast to the address in TB3_DDS_PEERS, or multicast on the
# same subnet. Nothing here can reach it. This script does not read or write
# /tmp/fastdds_peers.xml, does not touch TB3_DDS_PEERS, and needs no
# `docker compose up -d` afterwards -- so unlike editing .env it does not
# recreate the container and does not empty /ws/install.
#
# What it DOES affect is discovery between processes on THIS machine, which
# under backend:=real is the whole workstation layer: Nav2, RViz, drive_test.
# Deleting a segment out from under a live participant is the one way to make
# things worse, and the failure looks exactly like the locator trap in
# docs/network.md -- topics from the robot still listed, local nodes not. Hence
# the liveness check below, and hence:
#
#   /dev/shm is SHARED WITH THE HOST and this container has its own PID
#   namespace, so the check below CANNOT see host-side ROS 2. The workstation's
#   own ROS is Jazzy (docs/network.md) and writes into the same directory. Shut
#   that down too before running this.
#
# The robot's own segments live in the robot's own /dev/shm and are its problem;
# if turtlebot3_bringup there starts hanging the same silent way, this script
# works unmodified over ssh.
#
# Stopping the ros2 daemon is part of the job, not a side effect: it caches the
# graph across DDS changes, docs/network.md already says to stop it after one,
# and a wedged daemon is this same bug wearing a different hat -- `ros2 topic
# list` returning NOTHING while rqt_graph, which talks DDS directly, shows
# everything.
set -euo pipefail

kill_first=false
dry_run=false
for arg in "$@"; do
    case "$arg" in
        --kill)    kill_first=true ;;
        --dry-run) dry_run=true ;;
        -h|--help) sed -n '2,6p' "$0" | sed 's/^# \?//'; exit 0 ;;
        *) echo "dds_clean: unknown argument '$arg'" >&2; exit 2 ;;
    esac
done

# Matched on the executable path rather than a bare name: `nav2` alone also
# matches this script's own `bash -c` command line, and a self-kill is an
# unreadable exit 143.
ROS_PROCS='/opt/ros/[^ ]*/lib/|gzserver|gzclient|ros2 launch|ros2cli\.daemon|rviz2'

alive() { pgrep -af "$ROS_PROCS" 2>/dev/null | grep -v "$$" || true; }

running="$(alive)"
# --dry-run reports regardless: it removes nothing, and "what is still holding
# a segment" is half of what you want to see when you ask.
if [ -n "$running" ] && [ "$dry_run" = false ]; then
    if [ "$kill_first" = true ]; then
        echo "dds_clean: stopping $(echo "$running" | wc -l) ROS process(es)"
        echo "$running" | sed 's/^/  /'
        # SIGTERM first so a launch tears its own children down cleanly and
        # closes its segments; -9 only for what is left, which is the orphan
        # case this whole script exists for.
        echo "$running" | awk '{print $1}' | xargs -r kill 2>/dev/null || true
        sleep 3
        echo "$(alive)" | awk '{print $1}' | xargs -r kill -9 2>/dev/null || true
        sleep 1
    else
        echo "dds_clean: ROS 2 is still running -- refusing." >&2
        echo "$running" | sed 's/^/  /' >&2
        echo >&2
        echo "Deleting a segment a live participant owns breaks LOCAL discovery" >&2
        echo "(see the header). Stop them, or re-run with --kill." >&2
        exit 1
    fi
fi

shopt -s nullglob
stale=(/dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*)
shopt -u nullglob

if [ ${#stale[@]} -eq 0 ]; then
    echo "dds_clean: /dev/shm is already clear of Fast DDS segments"
    exit 0
fi

if [ "$dry_run" = true ]; then
    [ -n "$running" ] && { echo "dds_clean: still running --"; echo "$running" | sed 's/^/  /'; }
    echo "dds_clean: would remove ${#stale[@]} Fast DDS object(s) from /dev/shm"
    printf '  %s\n' "${stale[@]:0:10}"
    [ ${#stale[@]} -gt 10 ] && echo "  ... and $(( ${#stale[@]} - 10 )) more"
    exit 0
fi

rm -f "${stale[@]}"
echo "dds_clean: removed ${#stale[@]} Fast DDS object(s) from /dev/shm"

# The daemon caches the graph and survives everything above. Killed rather than
# `ros2 daemon stop`, which talks to it over XML-RPC -- the socket a wedged
# daemon is precisely not answering on, where stop hangs and then dies with
# `TimeoutError: [Errno 110] Connection timed out`.
if pkill -f 'ros2cli\.daemon' 2>/dev/null; then
    echo "dds_clean: ros2 daemon stopped (it restarts on the next ros2 command)"
fi
