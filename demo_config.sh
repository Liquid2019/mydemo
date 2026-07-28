#!/bin/bash
# 加载 demo_config.env 并导出环境变量；各 run_*.sh / start_demo_base.sh source 本文件

_DEMO_CFG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$_DEMO_CFG_DIR/demo_config.env"

export BASE_CPU_AFFINITY BASE_OMP_THREADS
export YOLO_CPU_AFFINITY YOLO_NICE
export IMGSZ CONF INFER_HZ DIST_HZ FAST_RENDER RENDER_MS OVERLAY_HZ
export STAGE2_LIDAR_R_MAX BBOX_EXPAND_RATIO DIST_SAFE_FALLBACK
export DEFAULT_ON_CLASSES DIST_MODE DETECTION_HOLD_SEC MAX_DETECTIONS
export LIDAR_STALE_SEC DIST_PERCENTILE
export ANCHOR_DISTANCE ANCHOR_MATCH ANCHOR_MIN_SCORE ANCHOR_MIN_TEXTURE
export ANCHOR_MIN_REGION_PTS ANCHOR_MATCH_INTERVAL ANCHOR_BBOX_EXPAND_RATIO
export ANCHOR_DENSIFY ANCHOR_DENSIFY_STEP ANCHOR_DENSIFY_MAX
export ANCHOR_SEARCH_MARGIN ANCHOR_FULL_SEARCH_INTERVAL ANCHOR_KEEP_SEC ANCHOR_IDLE_HZ
export SHOW_ANCHOR_BOX
export LIDAR_MAX DANGER_DRAW_MAX DANGER_DOT_RADIUS DANGER_DIST_MAX
export MIN_MASK_PTS CLUSTER_Z_GAP MASK_ALPHA MASK_POINT_DILATE_PX
export DANGER_NPZ MODEL_PT MARK_OMP_THREADS MARK_LINE_WIDTH_PX
export ROI_SAVE_MAX DANGER_TOTAL_MAX ROI_VOXEL
export DEMO_DIR="$_DEMO_CFG_DIR"

_affinity_ncores() {
    local aff="$1" n=0 part a b
    IFS=',' read -ra PARTS <<< "$aff"
    for part in "${PARTS[@]}"; do
        part="${part// /}"
        [ -z "$part" ] && continue
        if [[ "$part" == *-* ]]; then
            a="${part%-*}"; b="${part#*-}"
            n=$((n + b - a + 1))
        else
            n=$((n + 1))
        fi
    done
    echo "$n"
}

demo_config_apply_base() {
    export OMP_NUM_THREADS="${OMP_NUM_THREADS:-$BASE_OMP_THREADS}"
}

demo_config_apply_yolo() {
    local _nc
    _nc=$(_affinity_ncores "$YOLO_CPU_AFFINITY")
    export OMP_NUM_THREADS="${OMP_NUM_THREADS:-$_nc}"
}

demo_config_engine_path() {
    local model_path model_base
    if [[ "$MODEL_PT" = /* ]]; then
        model_path="$MODEL_PT"
    else
        model_path="$DEMO_DIR/$MODEL_PT"
    fi
    model_base="$(basename "$model_path")"
    model_base="${model_base%.*}"
    echo "$DEMO_DIR/${model_base}_i${IMGSZ}.engine"
}

demo_config_model_path() {
    local engine pt
    engine="$(demo_config_engine_path)"
    if [[ "$MODEL_PT" = /* ]]; then
        pt="$MODEL_PT"
    else
        pt="$DEMO_DIR/$MODEL_PT"
    fi
    if [ -f "$engine" ]; then
        echo "$engine"
    elif [ -f "$pt" ]; then
        echo "$pt"
    else
        echo "$pt"
    fi
}

demo_config_print() {
    echo "[CONFIG] Base cpu=$BASE_CPU_AFFINITY omp=$BASE_OMP_THREADS"
    echo "[CONFIG] Stage2 cpu=$YOLO_CPU_AFFINITY nice=$YOLO_NICE imgsz=$IMGSZ conf=$CONF infer_hz=$INFER_HZ dist_hz=$DIST_HZ render_ms=$RENDER_MS"
    echo "[CONFIG] Stage2 lidar_r_max=$STAGE2_LIDAR_R_MAX lidar_max=$LIDAR_MAX danger_dist_max=$DANGER_DIST_MAX bbox_expand=$BBOX_EXPAND_RATIO safe_fallback=$DIST_SAFE_FALLBACK"
    echo "[CONFIG] Stage2 classes=$DEFAULT_ON_CLASSES dist_mode=$DIST_MODE hold=$DETECTION_HOLD_SEC max_dets=$MAX_DETECTIONS stale=$LIDAR_STALE_SEC percentile=$DIST_PERCENTILE"
    echo "[CONFIG] Stage2 anchor=$ANCHOR_DISTANCE match=$ANCHOR_MATCH score=$ANCHOR_MIN_SCORE min_pts=$ANCHOR_MIN_REGION_PTS"
}
