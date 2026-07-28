#!/bin/bash
# Stage 2: YOLO seg (TensorRT) + lidar distance
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
demo_config_apply_yolo

cd "$DEMO_DIR"
export LD_LIBRARY_PATH=/usr/lib/llvm-8/lib:${LD_LIBRARY_PATH:-}

gpu_ok=0
if python3 - <<'PY' 2>/dev/null; then
import torch
from ultralytics import YOLO
assert torch.cuda.is_available()
PY
    gpu_ok=1
fi

if [ "$gpu_ok" != "1" ]; then
    echo "[!] GPU PyTorch / ultralytics 未就绪"
    echo "    请执行: bash ~/Desktop/my_demo/install_gpu_pytorch.sh"
    read -p "Press Enter to exit..."
    exit 1
fi

MODEL="$(demo_config_model_path)"
ENGINE="$(demo_config_engine_path)"
if [ -f "$ENGINE" ]; then
    echo "[INFO] TensorRT engine: $ENGINE"
else
    echo "[WARN] 未找到 $ENGINE，暂用 PyTorch .pt（较慢）"
    echo "       一次性导出: bash ~/Desktop/my_demo/tools/export_trt.sh $IMGSZ $MODEL_PT"
fi

demo_config_print
_render_note="fast-render"
EXTRA_PY=()
if [ "$FAST_RENDER" != "1" ]; then
    EXTRA_PY+=(--full-render)
    _render_note="full-render"
fi
echo "[INFO] render=${_render_note}  (改 demo_config.env 中 FAST_RENDER)"

LOG_DIR="$DEMO_DIR/logs"
mkdir -p "$LOG_DIR"
STAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/stage2_${STAMP}.log"
LATEST_LOG="$LOG_DIR/stage2_latest.log"
export PYTHONUNBUFFERED=1

PY=(python3 "$DEMO_DIR/3_fastlivo.py"
  --model "$MODEL"
  --conf "$CONF" --imgsz "$IMGSZ" --infer-hz "$INFER_HZ"
  --nice "$YOLO_NICE" --cpu-affinity "$YOLO_CPU_AFFINITY"
  "${EXTRA_PY[@]}"
  "$@")

CMD=("${PY[@]}")
[ -n "$YOLO_CPU_AFFINITY" ] && CMD=(taskset -c "$YOLO_CPU_AFFINITY" "${CMD[@]}")
[ "$YOLO_NICE" -ne 0 ] && CMD=(nice -n "$YOLO_NICE" "${CMD[@]}")

{
  echo "=== stage2 start $(date -Iseconds) nice=$YOLO_NICE cpu=$YOLO_CPU_AFFINITY model=$(basename "$MODEL") imgsz=$IMGSZ ==="
  "${CMD[@]}"
  ec=$?
  echo "=== stage2 exit code=$ec $(date -Iseconds) ==="
  exit $ec
} 2>&1 | tee "$LOG_FILE" | tee "$LATEST_LOG"
