#!/bin/bash
# Experiment 1's run matrix for ONE simulated backend, start to finish.
#
#   scripts/run_experiment1.sh gazebo
#   scripts/run_experiment1.sh isaacsim
#
# Run inside the tb3_ros container. Writes into measurements/experiment1/,
# which is where official runs live -- see the README there.
#
# The simulator is launched ONCE and every repeat runs against it without a
# respawn. That is safe because every number this produces is a DELTA over the
# run, resolved into the pose the run started from (`net` in drive_test), so
# where the robot happens to be on the floor when a repeat begins does not
# enter any result. It also turns ~45 minutes of launching into one launch.
#
# The real backend is deliberately NOT accepted here. It needs a person with a
# tape between runs, so it is driven by hand, one sequence at a time, with
# -p truth_xy_m / -p truth_yaw_deg. The README has the invocation.
set -euo pipefail

# Quoted: an unquoted `}` inside ${var:?word} ends the expansion early, so
# the message's own braces made this assign "$1}" for every argument and the
# case below never matched. Committed that way and dead on arrival.
BACKEND=${1:?"usage: run_experiment1.sh {gazebo|isaacsim}"}
DATE=${2:-$(date +%Y-%m-%d)}
OUT=/repo/measurements/experiment1
WORLD=empty_stage

case "$BACKEND" in
  gazebo)   SETTLE=35 ;;
  isaacsim) SETTLE=150 ;;   # Kit takes minutes, not seconds, to reach a stage
  *) echo "run_experiment1.sh: backend must be gazebo or isaacsim" >&2; exit 2 ;;
esac

# The matrix. Repeats per the README: 3 for the diagnostic sequences, 5 each
# way for UMBmark, which is its own prescription.
MATRIX="sweep:3 line:3 spin_cw:3 spin_ccw:3 square_cw:5 square_ccw:5"

# `set -u` off across this one line: colcon's generated setup.bash reads
# COLCON_TRACE unguarded and aborts the script under -u before anything
# has run. Strictness is worth keeping everywhere else.
set +u; source /ws/install/setup.bash; set -u
mkdir -p "$OUT"

# A stray publisher on the domain would drive the robot in the other
# simulator, or the real one. This has happened; refuse rather than discover it
# in the numbers afterwards.
# drive_test and robot_state_publisher are in the pattern because they are
# exactly what a KILLED run orphans: the launch dies, those two survive, and a
# leftover drive_test resumes the moment a new simulator publishes /clock --
# which silently contaminated a run the first time this matrix was driven.
STRAY="ros2 launch|gzserver|isaac-sim|drive_test|robot_state_publisher"
if pgrep -f "$STRAY" > /dev/null; then
  echo "run_experiment1.sh: something is already running on this domain:" >&2
  pgrep -af "$STRAY" >&2
  echo "a killed launch orphans gzserver and robot_state_publisher; they need" >&2
  echo "kill -9 by PID, and ros2 daemon stop afterwards." >&2
  exit 3
fi

echo "== launching $BACKEND =="
ros2 launch tb3_bringup bringup.launch.py \
    backend:="$BACKEND" world:="$WORLD" headless:=true rviz:=false \
    > "/tmp/exp1_${BACKEND}.log" 2>&1 &
LAUNCH=$!
trap 'kill $LAUNCH 2>/dev/null || true; pkill -f "ros2 launch" 2>/dev/null || true' EXIT
sleep "$SETTLE"

for entry in $MATRIX; do
  seq_name=${entry%%:*}
  repeats=${entry##*:}
  for r in $(seq 1 "$repeats"); do
    f="$OUT/${DATE}_exp1_${BACKEND}_${seq_name}_r${r}.json"
    echo "-- $seq_name r$r"
    ros2 run tb3_bringup drive_test --ros-args \
        -p label:="$BACKEND" -p sequence:="$seq_name" -p out:="$f" \
        > "/tmp/exp1_${BACKEND}_${seq_name}_r${r}.log" 2>&1
    python3 - "$f" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
n = d['net']
assert n['source'] == '/ground_truth/odom', 'NO GROUND TRUTH in ' + sys.argv[1]
print('   fwd %+8.4f m  lat %+8.4f m  yaw %+9.3f deg'
      % (n['forward_m'], n['lateral_m'], n['yaw_deg']))
PY
  done
done

echo "== $BACKEND done: $(ls "$OUT"/${DATE}_exp1_${BACKEND}_*.json | wc -l) runs =="
