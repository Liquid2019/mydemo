#!/bin/bash
# Background Base runner used by democtl.sh.

set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export MY_DEMO_ROOT="$ROOT"

source /opt/ros/noetic/setup.bash
source "$HOME/fastlivo2_ws/devel/setup.bash"

# shellcheck source=../demo_config.sh
source "$ROOT/demo_config.sh"
demo_config_apply_base

PIDS=()

base_launch() {
    if [ -n "${BASE_CPU_AFFINITY:-}" ]; then
        taskset -c "$BASE_CPU_AFFINITY" "$@"
    else
        "$@"
    fi
}

launch_bg() {
    local label="$1"
    shift
    echo "[$(date -Iseconds)] start $label: $*"
    base_launch "$@" &
    PIDS+=("$!")
}

cleanup() {
    echo "[$(date -Iseconds)] stopping Base worker"
    for pid in "${PIDS[@]:-}"; do
        kill "$pid" >/dev/null 2>&1 || true
    done
    sleep 1
    for pid in "${PIDS[@]:-}"; do
        kill -9 "$pid" >/dev/null 2>&1 || true
    done
}

trap cleanup INT TERM EXIT

echo "=== FAST-LIVO2 Demo Base worker ==="
echo "[INFO] root=$ROOT"
echo "[INFO] Base CPU affinity=${BASE_CPU_AFFINITY:-all} OMP=${OMP_NUM_THREADS:-unset}"

launch_bg "camera" roslaunch "$ROOT/drivers/launch/mvs_camera_demo.launch"
sleep 3

launch_bg "livox" roslaunch "$ROOT/drivers/launch/livox_lidar_demo.launch"
sleep 3

launch_bg "fast-livo" roslaunch fast_livo mapping_avia_demo.launch
sleep 2

echo "[$(date -Iseconds)] Base worker is running."
echo "Ready when status shows camera, lidar and pose data."

while true; do
    for pid in "${PIDS[@]}"; do
        if ! kill -0 "$pid" >/dev/null 2>&1; then
            echo "[$(date -Iseconds)] child process exited: $pid"
            exit 1
        fi
    done
    sleep 5
done
