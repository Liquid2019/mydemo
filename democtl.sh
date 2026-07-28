#!/bin/bash
# Unified controller for the outdoor obstacle-avoidance demo.

set +u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$ROOT/logs"
BACKUP_DIR="$ROOT/backups"
mkdir -p "$LOG_DIR" "$BACKUP_DIR"

ROS_SETUP="/opt/ros/noetic/setup.bash"
WS_SETUP="$HOME/fastlivo2_ws/devel/setup.bash"

ts() {
    date +%Y%m%d_%H%M%S
}

say() {
    echo "$*"
}

setup_ros() {
    [ -f "$ROS_SETUP" ] && source "$ROS_SETUP" >/dev/null 2>&1 || true
    [ -f "$WS_SETUP" ] && source "$WS_SETUP" >/dev/null 2>&1 || true
}

setup_desktop_env() {
    export DISPLAY="${DISPLAY:-:1}"
    if [ -z "${XAUTHORITY:-}" ]; then
        if [ -f "/run/user/$(id -u)/gdm/Xauthority" ]; then
            export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
        elif [ -f "$HOME/.Xauthority" ]; then
            export XAUTHORITY="$HOME/.Xauthority"
        fi
    fi
    if [ -z "${DBUS_SESSION_BUS_ADDRESS:-}" ] && [ -S "/run/user/$(id -u)/bus" ]; then
        export DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u)/bus"
    fi
}

load_config() {
    # shellcheck source=demo_config.sh
    [ -f "$ROOT/demo_config.sh" ] && source "$ROOT/demo_config.sh" >/dev/null 2>&1 || true
}

danger_npz_path() {
    load_config
    local npz="${DANGER_NPZ:-danger_points_global.npz}"
    if [[ "$npz" = /* ]]; then
        echo "$npz"
    else
        echo "$ROOT/$npz"
    fi
}

ros_ok() {
    setup_ros
    timeout 2 rostopic list >/dev/null 2>&1
}

wait_for_ros() {
    local seconds="${1:-25}"
    local i
    for i in $(seq 1 "$seconds"); do
        if ros_ok; then
            return 0
        fi
        sleep 1
    done
    return 1
}

proc_line() {
    local label="$1"
    local pattern="$2"
    local pids
    pids="$(pgrep -f "$pattern" 2>/dev/null | tr '\n' ' ' || true)"
    if [ -n "$pids" ]; then
        say "[OK] $label: 运行中 (PID $pids)"
    else
        say "[--] $label: 未运行"
    fi
}

topic_line() {
    local label="$1"
    local topic="$2"
    local rate
    if ! rostopic list 2>/dev/null | grep -qx "$topic"; then
        say "[--] $label: 话题不存在"
        return 0
    fi
    rate="$(timeout 5 rostopic hz "$topic" 2>/dev/null | sed -n 's/.*average rate: \([0-9.]*\).*/\1/p' | tail -1 || true)"
    if [ -n "$rate" ]; then
        say "[OK] $label: ${rate} Hz"
    else
        say "[!!] $label: 暂时没有新数据"
    fi
}

danger_status() {
    local npz
    npz="$(danger_npz_path)"
    python3 - "$npz" <<'PY'
import os
import sys
import numpy as np

path = sys.argv[1]
if not os.path.exists(path):
    print("[--] 危险区: 没有文件，还没有画区")
    raise SystemExit(0)

try:
    data = np.load(path)
except Exception as exc:
    print(f"[!!] 危险区: 文件打不开 ({exc})")
    raise SystemExit(0)

points_key = None
for key in ("xyz", "points", "pts_global", "danger_points_global", "danger_points"):
    if key in data and getattr(data[key], "ndim", 0) >= 2:
        points_key = key
        break

if points_key is None:
    print(f"[!!] 危险区: 文件里没有可用点，keys={list(data.files)}")
    raise SystemExit(0)

pts = data[points_key]
anchor = ""
if "anchor_rect" in data:
    rect = data["anchor_rect"].astype(float).tolist()
    anchor = f", 锚点框={int(rect[2])}x{int(rect[3])}"
frame = ""
if "coord_frame" in data:
    frame = f", 坐标={str(data['coord_frame'])}"
print(f"[OK] 危险区: {len(pts)} 个点{frame}{anchor}")
PY
}

config_status() {
    load_config
    say "[配置] 类别=${DEFAULT_ON_CLASSES:-未设置}  模型=${MODEL_PT:-未设置}  imgsz=${IMGSZ:-?}  检测=${INFER_HZ:-?}Hz  测距=${DIST_HZ:-?}Hz"
    if declare -f demo_config_model_path >/dev/null 2>&1; then
        local model
        model="$(demo_config_model_path)"
        if [ -f "$model" ]; then
            say "[OK] 当前模型: $model"
        else
            say "[!!] 当前模型文件不存在: $model"
        fi
    fi
}

cmd_status() {
    say "=== 智能避障项目状态 ==="
    config_status
    danger_status
    say ""
    proc_line "画危险区窗口" "[2]_fastlivo.py"
    proc_line "距离检测窗口" "[3]_fastlivo.py"
    proc_line "Base后台管理" "[s]cripts/base_worker.sh"
    proc_line "相机节点" "[g]rabImgWithTrigger"
    proc_line "雷达节点" "[l]ivox_ros_driver_node"
    proc_line "定位/建图节点" "[l]aserMapping"
    say ""
    setup_ros
    if ros_ok; then
        say "[OK] ROS: 已连接"
        topic_line "相机画面" "/left_camera/image"
        topic_line "雷达点云" "/livox/lidar"
        topic_line "实时位姿" "/aft_mapped_to_init"
    else
        say "[!!] ROS: 未连接，Base 没启动或还没启动完成"
    fi
}

cmd_stop_app() {
    say "正在关闭画区/检测窗口..."
    pkill -TERM -f "[r]un_mark.sh" 2>/dev/null || true
    pkill -TERM -f "[r]un_3.sh" 2>/dev/null || true
    pkill -TERM -f "[2]_fastlivo.py" 2>/dev/null || true
    pkill -TERM -f "[3]_fastlivo.py" 2>/dev/null || true
    sleep 1
    pkill -KILL -f "[2]_fastlivo.py" 2>/dev/null || true
    pkill -KILL -f "[3]_fastlivo.py" 2>/dev/null || true
    say "画区/检测窗口已处理。"
}

cmd_start_base() {
    setup_desktop_env
    if pgrep -f "[s]cripts/base_worker.sh" >/dev/null 2>&1; then
        say "Base 后台已经在运行。"
        return 0
    fi
    if pgrep -f "[l]aserMapping" >/dev/null 2>&1 || pgrep -f "[r]oslaunch.*mapping_avia" >/dev/null 2>&1; then
        say "检测到旧方式启动的 Base 已经在运行，直接复用。"
        return 0
    fi
    local worker="$ROOT/scripts/base_worker.sh"
    if [ ! -f "$worker" ]; then
        say "[x] 缺少 $worker"
        return 1
    fi
    local log="$LOG_DIR/base_worker_$(ts).log"
    say "正在启动 Base（相机 + 雷达 + 定位）..."
    nohup setsid bash "$worker" >"$log" 2>&1 </dev/null &
    echo $! >"$LOG_DIR/base_worker.pid"
    sleep 2
    if pgrep -f "[s]cripts/base_worker.sh" >/dev/null 2>&1; then
        ln -sfn "$(basename "$log")" "$LOG_DIR/base_worker_latest.log" 2>/dev/null || true
        say "[OK] Base 已在后台启动，日志: $log"
        say "     完全出画面/点云通常还要等十几秒，可以运行: ./democtl.sh status"
    else
        say "[x] Base 启动失败，最近日志如下:"
        tail -40 "$log" 2>/dev/null || true
        return 1
    fi
}

cmd_stop_base() {
    say "正在停止 Base..."
    pkill -TERM -f "[s]cripts/base_worker.sh" 2>/dev/null || true
    sleep 1
    [ -x "$ROOT/scripts/stop_fastlivo_base.sh" ] && bash "$ROOT/scripts/stop_fastlivo_base.sh" >/dev/null 2>&1 || true
    pkill -TERM -f "[r]oslaunch.*mvs_camera_demo" 2>/dev/null || true
    pkill -TERM -f "[r]oslaunch.*livox_lidar_demo" 2>/dev/null || true
    pkill -TERM -f "[r]oslaunch.*mapping_avia" 2>/dev/null || true
    pkill -TERM -f "[g]rabImgWithTrigger" 2>/dev/null || true
    pkill -TERM -f "[l]ivox_ros_driver_node" 2>/dev/null || true
    pkill -TERM -f "[l]aserMapping" 2>/dev/null || true
    sleep 2
    pkill -KILL -f "[g]rabImgWithTrigger" 2>/dev/null || true
    pkill -KILL -f "[l]ivox_ros_driver_node" 2>/dev/null || true
    pkill -KILL -f "[l]aserMapping" 2>/dev/null || true
    setup_ros
    timeout 4 rosnode cleanup >/dev/null 2>&1 || true
    say "Base 已停止。"
}

cmd_restart_base() {
    cmd_stop_app
    cmd_stop_base
    sleep 1
    cmd_start_base
}

launch_script() {
    local name="$1"
    local script="$2"
    setup_desktop_env
    local log="$LOG_DIR/${name}_launcher_$(ts).log"
    nohup setsid bash -lc 'cd "$1" && exec "$2"' _ "$ROOT" "$script" >"$log" 2>&1 </dev/null &
    say "[OK] 已启动，启动日志: $log"
}

backup_danger_file() {
    local npz
    npz="$(danger_npz_path)"
    if [ -f "$npz" ]; then
        local dst="$BACKUP_DIR/danger_points_global_$(ts).npz"
        cp -f "$npz" "$dst"
        say "[OK] 原危险区已备份: $dst"
        rm -f "$npz"
    fi
}

ensure_base_ready() {
    if ros_ok; then
        return 0
    fi
    say "Base 还没连上，先帮你启动 Base。"
    cmd_start_base || return 1
    if wait_for_ros 30; then
        return 0
    fi
    say "[!!] Base 还没完全起来。先等相机/雷达稳定后，再运行同一个命令。"
    return 1
}

cmd_mark() {
    cmd_stop_app
    backup_danger_file
    ensure_base_ready || return 1
    say "正在打开“重新画危险区”窗口..."
    launch_script "stage1" "$ROOT/run_mark.sh"
}

cmd_detect() {
    local npz
    npz="$(danger_npz_path)"
    if [ ! -f "$npz" ]; then
        say "[!!] 还没有危险区，请先运行: ./democtl.sh mark"
        return 1
    fi
    cmd_stop_app
    ensure_base_ready || return 1
    say "正在打开“距离检测”窗口..."
    launch_script "stage2" "$ROOT/run_3.sh"
}

cmd_stop_all() {
    cmd_stop_app
    cmd_stop_base
    pkill -TERM -f "[r]osmaster" 2>/dev/null || true
    pkill -TERM -f "[r]oscore" 2>/dev/null || true
    say "全部已停止。"
}

cmd_logs() {
    say "=== 最近日志 ==="
    ls -t "$LOG_DIR"/*.log 2>/dev/null | head -8 || true
    say ""
    if [ -f "$LOG_DIR/stage2_latest.log" ]; then
        say "--- stage2_latest.log 最后 60 行 ---"
        tail -60 "$LOG_DIR/stage2_latest.log"
    elif [ -f "$LOG_DIR/stage1_latest.log" ]; then
        say "--- stage1_latest.log 最后 60 行 ---"
        tail -60 "$LOG_DIR/stage1_latest.log"
    fi
}

cmd_help() {
    cat <<'EOF'
用法:
  ./democtl.sh menu          打开中文菜单
  ./democtl.sh status        检查 Base/雷达/相机/危险区/检测窗口
  ./democtl.sh restart-base  重启 Base
  ./democtl.sh mark          重新画危险区
  ./democtl.sh detect        启动距离检测
  ./democtl.sh stop-app      只关闭画区/检测窗口
  ./democtl.sh stop-all      关闭窗口、Base 和 ROS
  ./democtl.sh logs          查看最近日志
EOF
}

pause_menu() {
    echo ""
    read -r -p "按回车继续..." _
}

cmd_menu() {
    while true; do
        clear 2>/dev/null || true
        cat <<'EOF'
================================
 智能避障项目控制台
================================
 1. 状态检查
 2. 启动/重启 Base
 3. 重新画危险区
 4. 启动距离检测
 5. 关闭画区/检测窗口
 6. 停止全部
 7. 查看最近日志
 q. 退出
EOF
        echo ""
        read -r -p "请选择: " choice
        echo ""
        case "$choice" in
            1) cmd_status; pause_menu ;;
            2) cmd_restart_base; pause_menu ;;
            3) cmd_mark; pause_menu ;;
            4) cmd_detect; pause_menu ;;
            5) cmd_stop_app; pause_menu ;;
            6) cmd_stop_all; pause_menu ;;
            7) cmd_logs; pause_menu ;;
            q|Q) exit 0 ;;
            *) say "没看懂这个选项。"; pause_menu ;;
        esac
    done
}

case "${1:-menu}" in
    menu) cmd_menu ;;
    status) cmd_status ;;
    start-base) cmd_start_base ;;
    stop-base) cmd_stop_base ;;
    restart-base) cmd_restart_base ;;
    mark) cmd_mark ;;
    detect|run) cmd_detect ;;
    stop-app) cmd_stop_app ;;
    stop-all) cmd_stop_all ;;
    logs) cmd_logs ;;
    help|-h|--help) cmd_help ;;
    *)
        say "未知命令: $1"
        cmd_help
        exit 2
        ;;
esac
