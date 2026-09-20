#!/usr/bin/env bash
set -e

PYTHON_BIN="/home/space/miniconda3/envs/omx/bin/python"
LEROBOT_TELEOP="/home/space/miniconda3/envs/omx/bin/lerobot-teleoperate"
SUPERVISOR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/omx_supervisor.py"

SUP_PID=""

# Speed of the initial slow approach to the leader's pose, in normalized action units
# per second. A joint spans 200 units, so 12.5 is roughly 16 s for a full sweep.
# Lower = slower and gentler. Set to 0 to disable the approach.
export LEROBOT_APPROACH_SPEED=12.5
export LEROBOT_APPROACH_TIMEOUT=45

shutdown_supervisor() {
    if [[ -n "$SUP_PID" ]] && kill -0 "$SUP_PID" 2>/dev/null; then
        echo "[teleop.sh] Forwarding shutdown to control window (PID $SUP_PID)..."
        kill -INT "$SUP_PID" 2>/dev/null || true
        for _ in $(seq 1 120); do
            kill -0 "$SUP_PID" 2>/dev/null || return 0
            sleep 0.5
        done
        echo "[teleop.sh] Control window did not exit; sending SIGTERM."
        kill -TERM "$SUP_PID" 2>/dev/null || true
        sleep 3
        kill -KILL "$SUP_PID" 2>/dev/null || true
    fi
}

cleanup() {
    shutdown_supervisor
    pkill -f "rerun.*9876" 2>/dev/null || true
    [[ -t 0 ]] && stty sane 2>/dev/null || true
}

trap 'cleanup' EXIT
trap 'echo "[teleop.sh] Signal received."; exit 130' INT TERM HUP

set +e
"$PYTHON_BIN" "$SUPERVISOR" --title "OMX Teleoperation" -- \
  "$LEROBOT_TELEOP" \
  --robot.type=omx_follower \
  --robot.port=/dev/omx_follower \
  --robot.id=omx_follower_arm \
  --robot.cameras="{wrist: {type: opencv, index_or_path: '/dev/video2', width: 640, height: 480, fps: 30}}" \
  --teleop.type=omx_leader \
  --teleop.port=/dev/omx_leader \
  --teleop.id=omx_leader_arm \
  --display_data=true &

SUP_PID=$!
wait "$SUP_PID"
TELEOP_STATUS=$?
SUP_PID=""
set -e

if [[ $TELEOP_STATUS -ne 0 ]]; then
    echo "WARNING: teleoperation exited with status $TELEOP_STATUS."
    exit "$TELEOP_STATUS"
fi

echo "Teleoperation finished."
