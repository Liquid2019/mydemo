# -*- coding: utf-8 -*-
"""Load demo_config.env — Python 侧默认值与 3_fastlivo.py 共用。"""

import os

_CFG_DIR = os.path.dirname(os.path.abspath(__file__))
_ENV_PATH = os.path.join(_CFG_DIR, "demo_config.env")


def _load_env(path):
    cfg = {}
    if not os.path.isfile(path):
        return cfg
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            cfg[k.strip()] = v.strip()
    return cfg


_CFG = _load_env(_ENV_PATH)


def _get(key, default=""):
    return _CFG.get(key, default)


def _int(key, default=0):
    try:
        return int(_get(key, str(default)))
    except ValueError:
        return default


def _float(key, default=0.0):
    try:
        return float(_get(key, str(default)))
    except ValueError:
        return default


def _bool(key, default=False):
    v = _get(key, "1" if default else "0").lower()
    return v in ("1", "true", "yes", "on")


SCRIPT_DIR = _CFG_DIR

IMGSZ = _int("IMGSZ", 416)
DEFAULT_IMGSZ = IMGSZ
DEFAULT_CONF = _float("CONF", 0.25)
DEFAULT_INFER_HZ = _float("INFER_HZ", 0.0)
DIST_HZ = _float("DIST_HZ", 15.0)
RENDER_MS = _int("RENDER_MS", 50)
OVERLAY_HZ = _float("OVERLAY_HZ", 15.0)
STAGE2_LIDAR_R_MAX = _float("STAGE2_LIDAR_R_MAX", 25.0)
BBOX_EXPAND_RATIO = _float("BBOX_EXPAND_RATIO", 0.18)
DIST_SAFE_FALLBACK = _bool("DIST_SAFE_FALLBACK", True)
DEFAULT_ON_CLASSES = _get("DEFAULT_ON_CLASSES", "person")
DIST_MODE = _get("DIST_MODE", "mask_bbox").lower()
DETECTION_HOLD_SEC = _float("DETECTION_HOLD_SEC", 0.8)
MAX_DETECTIONS = _int("MAX_DETECTIONS", 8)
LIDAR_STALE_SEC = _float("LIDAR_STALE_SEC", 0.45)
DIST_PERCENTILE = _float("DIST_PERCENTILE", 10.0)
POSE_HISTORY_SEC = _float("POSE_HISTORY_SEC", 3.0)
POSE_SYNC_MAX_DT = _float("POSE_SYNC_MAX_DT", 0.15)
ANCHOR_DISTANCE = _bool("ANCHOR_DISTANCE", True)
ANCHOR_MATCH = _bool("ANCHOR_MATCH", True)
ANCHOR_MIN_SCORE = _float("ANCHOR_MIN_SCORE", 0.62)
ANCHOR_MIN_TEXTURE = _float("ANCHOR_MIN_TEXTURE", 8.0)
ANCHOR_MIN_REGION_PTS = _int("ANCHOR_MIN_REGION_PTS", 8)
ANCHOR_MATCH_INTERVAL = _float("ANCHOR_MATCH_INTERVAL", 0.18)
ANCHOR_BBOX_EXPAND_RATIO = _float("ANCHOR_BBOX_EXPAND_RATIO", 0.04)
ANCHOR_DENSIFY = _bool("ANCHOR_DENSIFY", True)
ANCHOR_DENSIFY_STEP = _float("ANCHOR_DENSIFY_STEP", 0.03)
ANCHOR_DENSIFY_MAX = _int("ANCHOR_DENSIFY_MAX", 2500)
ANCHOR_SEARCH_MARGIN = _float("ANCHOR_SEARCH_MARGIN", 2.2)
ANCHOR_FULL_SEARCH_INTERVAL = _float("ANCHOR_FULL_SEARCH_INTERVAL", 8.0)
ANCHOR_KEEP_SEC = _float("ANCHOR_KEEP_SEC", 0.25)
ANCHOR_IDLE_HZ = _float("ANCHOR_IDLE_HZ", 2.0)
SHOW_ANCHOR_BOX = _bool("SHOW_ANCHOR_BOX", False)

LIDAR_MAX = _int("LIDAR_MAX", 8000)
DANGER_DRAW_MAX = _int("DANGER_DRAW_MAX", 300)
DANGER_DOT_RADIUS = _int("DANGER_DOT_RADIUS", 0)
DANGER_DIST_MAX = _int("DANGER_DIST_MAX", 6000)
MIN_MASK_PTS = _int("MIN_MASK_PTS", 5)
CLUSTER_Z_GAP = _float("CLUSTER_Z_GAP", 0.15)
MASK_ALPHA = _float("MASK_ALPHA", 0.35)
MASK_POINT_DILATE_PX = _int("MASK_POINT_DILATE_PX", 0)

MODEL_PT = _get("MODEL_PT", "yolov8n-seg.pt")
DEFAULT_MODEL = MODEL_PT if os.path.isabs(MODEL_PT) else os.path.join(SCRIPT_DIR, MODEL_PT)

_npz = _get("DANGER_NPZ", "danger_points_global.npz")
DEFAULT_NPZ = _npz if os.path.isabs(_npz) else os.path.join(SCRIPT_DIR, _npz)

FAST_RENDER = _bool("FAST_RENDER", True)

ROI_SAVE_MAX = _int("ROI_SAVE_MAX", 10000)
DANGER_TOTAL_MAX = _int("DANGER_TOTAL_MAX", 80000)
ROI_VOXEL = _float("ROI_VOXEL", 0.02)
ROI_DENSIFY = _bool("ROI_DENSIFY", True)
ROI_DENSIFY_STEP = _float("ROI_DENSIFY_STEP", 0.02)
ROI_DENSIFY_MAX = _int("ROI_DENSIFY_MAX", 12000)
ROI_DENSIFY_MAX_THICKNESS = _float("ROI_DENSIFY_MAX_THICKNESS", 0.10)
MARK_LINE_WIDTH_PX = _int("MARK_LINE_WIDTH_PX", 18)
