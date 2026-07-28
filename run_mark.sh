#!/bin/bash
# Stage 1: manual danger-point marking (FAST-LIVO2)
set -e
source /opt/ros/noetic/setup.bash
source ~/fastlivo2_ws/devel/setup.bash

if ! rostopic list &>/dev/null; then
    echo "[x] ROS not running. Start Lite-SLAM-Base or Demo-Base first."
    read -p "Press Enter to exit..."
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=demo_config.sh
source "$SCRIPT_DIR/demo_config.sh"

cd "$DEMO_DIR"
export OMP_NUM_THREADS="$MARK_OMP_THREADS"
export PYTHONUNBUFFERED=1
export DISPLAY="${DISPLAY:-:0}"

if [[ "$DANGER_NPZ" = /* ]]; then
    NPZ="$DANGER_NPZ"
else
    NPZ="$DEMO_DIR/$DANGER_NPZ"
fi

LOG_DIR="$DEMO_DIR/logs"
mkdir -p "$LOG_DIR"
STAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/stage1_${STAMP}.log"
LATEST_LOG="$LOG_DIR/stage1_latest.log"

echo "[INFO] stage1 log -> $LOG_FILE"
echo "[INFO] latest     -> $LATEST_LOG"

{
  echo "=== stage1 start $(date -Iseconds) npz=$NPZ ==="
  python3 "$DEMO_DIR/2_fastlivo.py" --npz "$NPZ" "$@"
  ec=$?
  echo "=== stage1 exit code=$ec $(date -Iseconds) ==="
  exit $ec
} 2>&1 | tee "$LOG_FILE" | tee "$LATEST_LOG"
