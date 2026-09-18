"""What an instrument writes: the rosbag, and who ends up owning the files.

Shared by drive_test and nav_test because the bag is the raw data and the
summary is derived from it: a metric nobody thought of on the night of the run
is recoverable from the bag and from nothing else. This repository has already
been caught three times by a mean impersonating a constant, and each time the
way back was the unaveraged samples.

Two things this handles that a bare `ros2 bag record` does not.

**Topics that do not exist on every backend.** `/clock` and `/ground_truth/odom`
are simulator-only, `/battery_state` is the real robot's alone, and `/imu` —
measured 2026-09-18 — is published by Gazebo and the real robot but NOT by
Isaac Sim. rosbag2 subscribes to a named topic when it appears and simply
records nothing for one that never does. That absence is data — it is how a bag
says which backend it came from — so the list is the same on all three and is
not filtered per backend.

**Root-owned files in the repository.** Everything here runs as root inside the
container, and the workspace is a bind mount, so anything an instrument writes
into the repo lands root:root and cannot be removed or rewritten from the host
without sudo. A directory of bags did exactly that on 2026-09-18 and blocked a
git merge. `give_back()` is the fix and it is not only about bags — the JSON
summaries and the raw traces beside them are written the same way and had the
same problem, which is why all three instruments call it on every file they
write.
"""

import os
import shutil
import signal
import subprocess

# The open-loop chain, end to end: what was commanded, what the wheels did,
# where the body went, and what the robot thinks all of that means.
#
# Not every backend publishes every one of these, and that is the point: see
# the module docstring. Recording the same list everywhere is what makes the
# bag self-describing rather than needing a note about which backend wrote it.
DRIVE_TOPICS = ('/odom', '/cmd_vel', '/joint_states', '/tf', '/tf_static',
                '/imu', '/clock', '/ground_truth/odom',
                # Real robot only, and a CONTROL rather than a measurement: the
                # OpenCR tracks a commanded wheel velocity less well as the
                # pack drains, so a run at the end of a session may not be
                # comparable with one from the start. Without the voltage in
                # the bag there is no way to find that out afterwards.
                '/battery_state')

# The task layer adds what Nav2 decided, on top of everything the plant did.
NAV_TOPICS = DRIVE_TOPICS + ('/scan', '/amcl_pose', '/plan',
                             '/behavior_tree_log')


def start_bag(bag_dir, topics):
    """Start `ros2 bag record` in the background, or return None."""
    if not bag_dir or shutil.which('ros2') is None:
        return None
    if os.path.exists(bag_dir):
        shutil.rmtree(bag_dir)
    return subprocess.Popen(
        ['ros2', 'bag', 'record', '-o', bag_dir, *topics],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop_bag(proc, bag_dir):
    """SIGINT the recorder, wait for it to close the file, hand back the files.

    SIGINT and not kill: rosbag2 writes its metadata.yaml on shutdown, and a
    bag without one is unreadable by every tool that would open it.
    """
    if proc is None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    give_back(bag_dir)


def give_back(path):
    """chown a written file or directory to whoever owns the directory above it.

    Call it on everything an instrument writes into the repository. Inside the
    container that is the host user; run anywhere else it is already a no-op,
    because the process is not root or the parent is.

    Best effort on purpose: a recording that exists and is awkward to delete
    beats a recording that failed at the last step.
    """
    if not path or not os.path.exists(path) or os.geteuid() != 0:
        return
    parent = os.path.dirname(os.path.abspath(path)) or '/'
    try:
        owner = os.stat(parent)
    except OSError:
        return
    if owner.st_uid == 0:
        return

    def chown(p):
        try:
            os.chown(p, owner.st_uid, owner.st_gid)
        except OSError:
            pass

    chown(path)
    for root, dirs, files in os.walk(path):
        for name in (*dirs, *files):
            chown(os.path.join(root, name))
