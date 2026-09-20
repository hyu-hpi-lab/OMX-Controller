#!/usr/bin/env bash
set -e

PYTHON_BIN="/home/space/miniconda3/envs/omx/bin/python"
LEROBOT_RECORD="/home/space/miniconda3/envs/omx/bin/lerobot-record"
SUPERVISOR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/omx_supervisor.py"

SUP_PID=""

# Speed of the initial slow approach to the leader's pose, in normalized action units
# per second. A joint spans 200 units, so 12.5 is roughly 16 s for a full sweep.
# Lower = slower and gentler. Set to 0 to disable the approach.
export LEROBOT_APPROACH_SPEED=12.5
export LEROBOT_APPROACH_TIMEOUT=45

# Enter each episode armed but not capturing, so the control window's countdown
# decides when frames actually start being written.
export LEROBOT_START_EPISODES_PAUSED=1

# Graceful teardown of the supervisor: it owns the escalation ladder that stops
# lerobot-record without ever yanking the serial bus out from under it.
shutdown_supervisor() {
    if [[ -n "$SUP_PID" ]] && kill -0 "$SUP_PID" 2>/dev/null; then
        echo "[record.sh] Forwarding shutdown to control window (PID $SUP_PID)..."
        kill -INT "$SUP_PID" 2>/dev/null || true
        # Allow the full ladder (graceful stop + dataset finalize) to run.
        for _ in $(seq 1 360); do
            kill -0 "$SUP_PID" 2>/dev/null || return 0
            sleep 0.5
        done
        echo "[record.sh] Control window did not exit; sending SIGTERM."
        kill -TERM "$SUP_PID" 2>/dev/null || true
        sleep 3
        kill -KILL "$SUP_PID" 2>/dev/null || true
    fi
}

# Last-resort sweep. The supervisor normally kills the Rerun viewer it tracked as a
# descendant; this stays as a backstop for the paths that never reach it.
cleanup() {
    shutdown_supervisor
    [[ -n "${NAME_FILE:-}" ]] && rm -f "$NAME_FILE"
    pkill -f "rerun.*9876" 2>/dev/null || true
    # SIGKILL bypasses LeRobot's atexit hook, which can leave the TTY in cbreak mode.
    [[ -t 0 ]] && stty sane 2>/dev/null || true
}

trap 'cleanup' EXIT
trap 'echo "[record.sh] Signal received."; exit 130' INT TERM HUP

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

# The dataset name is now entered in the control window; it reports the chosen name
# back through this file so the upload step below knows what to send.
NAME_FILE="$(mktemp -t omx_dataset_name.XXXXXX)"

DATASET_DIR="$HOME/Projects/OMX/datasets"

# Run in the background so this shell keeps servicing traps, but in the SAME process
# group. {DATASET} is substituted by the control window once the name is entered.
set +e
"$PYTHON_BIN" "$SUPERVISOR" \
  --title "OMX Recording" \
  --controls record \
  --countdown 3 \
  --prompt-dataset-name \
  --dataset-name-default "$TIMESTAMP" \
  --dataset-name-out "$NAME_FILE" \
  --dataset-dir "$DATASET_DIR" -- \
  "$LEROBOT_RECORD" \
  --robot.type=omx_follower \
  --robot.port=/dev/omx_follower \
  --robot.id=omx_follower_arm \
  --robot.cameras="{wrist: {type: opencv, index_or_path: '/dev/video2', width: 640, height: 480, fps: 30}}" \
  --teleop.type=omx_leader \
  --teleop.port=/dev/omx_leader \
  --teleop.id=omx_leader_arm \
  --dataset.repo_id="local/{DATASET}" \
  --dataset.root="$DATASET_DIR/{DATASET}" \
  --dataset.num_episodes=0 \
  --dataset.single_task="{DATASET}" \
  --dataset.push_to_hub=false \
  --display_data=true &

SUP_PID=$!
wait "$SUP_PID"
RECORD_STATUS=$?
SUP_PID=""
set -e

if [[ $RECORD_STATUS -ne 0 ]]; then
    echo "WARNING: recording exited with status $RECORD_STATUS."
    echo "The dataset may be incomplete. Upload aborted."
    exit "$RECORD_STATUS"
fi

DATASET_NAME="$(cat "$NAME_FILE" 2>/dev/null || true)"
DATASET_ROOT="$DATASET_DIR/$DATASET_NAME"

if [[ -z "$DATASET_NAME" ]]; then
    echo "No dataset was recorded (name never confirmed). Nothing to upload."
    exit 0
fi

echo "Dataset: $DATASET_NAME"
echo "Path:    $DATASET_ROOT"
echo "Preparing dataset upload..."

if [[ -z "$DATASET_NAME" || -z "$DATASET_ROOT" ]]; then
    echo "ERROR: Dataset variables are empty. Upload aborted."
    exit 1
fi

case "$DATASET_ROOT" in
    "$HOME/Projects/OMX/datasets/"*) ;;
    *)
        echo "ERROR: Unsafe dataset path: $DATASET_ROOT"
        exit 1
        ;;
esac

if [[ ! -d "$DATASET_ROOT" ]]; then
    echo "ERROR: Dataset directory does not exist: $DATASET_ROOT"
    exit 1
fi

echo "Uploading dataset:"
echo "  Local:  $DATASET_ROOT"
echo "  Remote: leoperator:~/Projects/Datasets/OMX/$DATASET_NAME"

ssh leoperator 'mkdir -p ~/Projects/Datasets/OMX'

rsync -a --partial --info=progress2 \
  "$DATASET_ROOT/" \
  "leoperator:~/Projects/Datasets/OMX/$DATASET_NAME/"

echo "Upload complete."
