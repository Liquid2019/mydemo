#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FAST-LIVO2 stage 2: YOLO seg + mask/lidar distance to danger points."""

import os
os.environ.setdefault("OMP_NUM_THREADS", "4")
os.environ["WANDB_DISABLED"] = "true"
_ll = os.environ.get("LD_LIBRARY_PATH", "")
if "/usr/lib/llvm-8/lib" not in _ll:
    os.environ["LD_LIBRARY_PATH"] = "/usr/lib/llvm-8/lib:" + _ll

import sys
import time
import argparse
import threading
import numpy as np
# TensorRT (Jetson) still uses np.bool; removed in numpy>=1.24
for _n, _v in (("bool", np.bool_), ("int", np.int_), ("float", np.float_),
               ("complex", np.complex_), ("object", np.object_), ("str", np.str_)):
    if _n not in np.__dict__:
        setattr(np, _n, _v)
import cv2

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from livox_ros_driver.msg import CustomMsg

from fastlivo_calib import (
    load_pinhole_calib,
    load_proc_image_size,
    apply_T,
    inv_T,
    T_world_from_imu_pose,
    project_uv_pinhole,
    ros_image_to_bgr,
    parse_livox_msg,
    stamp_bgr,
    cKDTree,
    min_dist_pts_to_danger,
    robust_dist_pts_to_danger,
    front_depth_cluster,
    view_rect,
    wait_ros_topics,
    msg_stamp_sec,
    interp_pose,
    densify_planar_points,
)

from demo_ui import (
    BG as UI_BG,
    PANEL as UI_PANEL,
    GREEN as UI_GREEN,
    YELLOW as UI_YELLOW,
    RED as UI_RED,
    layout_buttons,
    hit_button,
    draw_button,
    draw_status_strip,
    draw_banner,
    draw_detection_label,
    friendly_reason,
    put_text_fit,
)

try:
    from ultralytics import YOLO
    YOLO_OK = True
except Exception:
    YOLO = None
    YOLO_OK = False

from demo_config import (
    SCRIPT_DIR,
    DEFAULT_MODEL,
    DEFAULT_NPZ,
    DEFAULT_CONF,
    DEFAULT_INFER_HZ,
    DEFAULT_IMGSZ,
    RENDER_MS,
    DIST_HZ,
    DANGER_DRAW_MAX,
    DANGER_DOT_RADIUS,
    LIDAR_MAX,
    DANGER_DIST_MAX,
    STAGE2_LIDAR_R_MAX,
    BBOX_EXPAND_RATIO,
    DIST_SAFE_FALLBACK,
    DEFAULT_ON_CLASSES,
    DIST_MODE,
    DETECTION_HOLD_SEC,
    MAX_DETECTIONS,
    LIDAR_STALE_SEC,
    DIST_PERCENTILE,
    POSE_HISTORY_SEC,
    POSE_SYNC_MAX_DT,
    MIN_MASK_PTS,
    CLUSTER_Z_GAP,
    MASK_ALPHA,
    MASK_POINT_DILATE_PX,
    OVERLAY_HZ,
    FAST_RENDER,
    ANCHOR_DISTANCE,
    ANCHOR_MATCH,
    ANCHOR_MIN_SCORE,
    ANCHOR_MIN_TEXTURE,
    ANCHOR_MIN_REGION_PTS,
    ANCHOR_MATCH_INTERVAL,
    ANCHOR_BBOX_EXPAND_RATIO,
    ANCHOR_DENSIFY,
    ANCHOR_DENSIFY_STEP,
    ANCHOR_DENSIFY_MAX,
    ANCHOR_SEARCH_MARGIN,
    ANCHOR_FULL_SEARCH_INTERVAL,
    ANCHOR_KEEP_SEC,
    ANCHOR_IDLE_HZ,
    SHOW_ANCHOR_BOX,
)

WIN_W, WIN_H = 1280, 800
VIEW_H = 620
BTN_BAR_H = WIN_H - VIEW_H
BTN_MARGIN, BTN_GAP = 10, 10


def _engine_path_for(model_path, imgsz):
    if not os.path.isabs(model_path):
        model_path = os.path.join(SCRIPT_DIR, os.path.basename(model_path))
    base = os.path.splitext(os.path.basename(model_path))[0]
    return os.path.join(SCRIPT_DIR, f"{base}_i{imgsz}.engine")


def resolve_model_path(model_path, imgsz):
    if not os.path.isabs(model_path):
        model_path = os.path.join(SCRIPT_DIR, os.path.basename(model_path))
    engine = _engine_path_for(model_path, imgsz)
    legacy_engine = os.path.join(SCRIPT_DIR, f"yolov8n-seg_i{imgsz}.engine")
    prefer_engine = model_path.endswith((".pt", ".onnx")) or model_path == DEFAULT_MODEL
    if prefer_engine and os.path.isfile(engine):
        return engine
    if prefer_engine and os.path.isfile(legacy_engine):
        return legacy_engine
    if os.path.isfile(model_path):
        return model_path
    if os.path.isfile(engine):
        return engine
    if os.path.isfile(legacy_engine):
        return legacy_engine
    return DEFAULT_MODEL


def _model_backend(path):
    if path.endswith(".engine"):
        return "TensorRT"
    if path.endswith(".onnx"):
        return "ONNX"
    return "PyTorch"


BOTTLE_IDS = [39]
CLASS_DEFS = {
    "cup":    {"ids": BOTTLE_IDS, "label": "Bottle", "color": (0, 200, 0),   "btn_on": (40, 160, 40)},
    "person": {"ids": [0],          "label": "Person", "color": (255, 200, 0), "btn_on": (40, 140, 200)},
    "car":    {"ids": [2],          "label": "Car",    "color": (0, 140, 255), "btn_on": (40, 100, 220)},
}
BTN_OFF = (70, 70, 70)
QUIT_COLOR = (40, 40, 180)
FRONT_ON = (40, 120, 160)
PROFILE_INTERVAL = 3.0


def _default_enabled():
    wanted = {x.strip().lower() for x in str(DEFAULT_ON_CLASSES).split(",") if x.strip()}
    if "bottle" in wanted:
        wanted.add("cup")
    if "all" in wanted:
        return {k: True for k in CLASS_DEFS}
    enabled = {k: k in wanted for k in CLASS_DEFS}
    if not any(enabled.values()):
        enabled["person"] = True
    return enabled


def _normal_dist_mode():
    mode = str(DIST_MODE).strip().lower()
    if mode not in ("mask_bbox", "bbox_only", "mask_only"):
        print(f"[WARN] bad DIST_MODE={DIST_MODE!r}, use mask_bbox")
        return "mask_bbox"
    return mode


def _log_torch_cuda(yolo_device):
    try:
        import torch
        cuda_ok = torch.cuda.is_available()
        msg = f"[INFO] torch={torch.__version__} cuda={cuda_ok} yolo_device={yolo_device}"
        if cuda_ok:
            msg += f" gpu={torch.cuda.get_device_name(0)}"
        else:
            msg += " (CPU fallback -> bash ~/Desktop/my_demo/install_gpu_pytorch.sh)"
        print(msg)
        return cuda_ok
    except Exception as e:
        print(f"[WARN] torch/cuda check: {e}")
        return False


class SegTimer:
    """Rolling average segment timings, printed every interval seconds."""

    def __init__(self, interval=PROFILE_INTERVAL):
        self.interval = interval
        self.lock = threading.Lock()
        self._sum = {}
        self._cnt = {}
        self._last = time.time()

    def add(self, name, ms):
        with self.lock:
            self._sum[name] = self._sum.get(name, 0.0) + float(ms)
            self._cnt[name] = self._cnt.get(name, 0) + 1
            now = time.time()
            if now - self._last >= self.interval:
                self._flush(now)

    def _flush(self, now):
        order = ("infer", "mask", "anchor", "dist", "cam", "overlay", "render")
        parts = []
        for name in order:
            n = self._cnt.get(name, 0)
            if n:
                parts.append(f"{name}:{self._sum[name] / n:.1f}ms")
        if parts:
            print("[PERF] " + " ".join(parts))
        self._sum.clear()
        self._cnt.clear()
        self._last = now


def _dist_color(dist):
    if dist > 2.0:
        return (0, 255, 0)
    if dist > 1.0:
        return (0, 255, 255)
    return (0, 0, 255)


def _parse_cpu_affinity(text):
    if not text or str(text).lower() in ("", "none", "off", "all"):
        return None
    cpus = set()
    for part in str(text).split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            cpus.update(range(int(a), int(b) + 1))
        else:
            cpus.add(int(part))
    return cpus or None


def _affinity_core_count(cpu_affinity):
    cpus = _parse_cpu_affinity(cpu_affinity)
    if cpus is not None:
        return len(cpus)
    if hasattr(os, "sched_getaffinity"):
        return len(os.sched_getaffinity(0))
    return None


def limit_cpu_threads(n=None):
    """Keep OpenMP/PyTorch/OpenCV worker threads on assigned cores only."""
    if n is None:
        n = _affinity_core_count(None) or int(os.environ.get("OMP_NUM_THREADS", "2"))
    n = max(1, int(n))
    for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ[key] = str(n)
    try:
        cv2.setNumThreads(n)
    except Exception:
        pass
    try:
        import torch
        torch.set_num_threads(n)
        if hasattr(torch, "set_num_interop_threads"):
            torch.set_num_interop_threads(max(1, n // 2))
    except Exception:
        pass
    print(f"[INFO] cpu worker threads={n}")


def apply_sched_priority(nice=None, cpu_affinity=None):
    """Bind stage2 to dedicated cores; optional nice (keep 0 when Base is also bound)."""
    if nice is not None:
        try:
            target = int(nice)
            cur = os.getpriority(os.PRIO_PROCESS, 0)
            delta = target - cur
            if delta != 0:
                os.nice(delta)
            print(f"[INFO] sched nice={os.getpriority(os.PRIO_PROCESS, 0)} (target {target})")
        except PermissionError:
            print(f"[WARN] nice={nice} denied -> run: sudo bash ~/Desktop/my_demo/run_3.sh")
            print(f"[WARN] current nice={os.getpriority(os.PRIO_PROCESS, 0)}")
        except Exception as e:
            print(f"[WARN] nice failed: {e}")

    cpus = _parse_cpu_affinity(cpu_affinity)
    if cpus is not None and hasattr(os, "sched_setaffinity"):
        try:
            os.sched_setaffinity(0, cpus)
            print(f"[INFO] cpu affinity={sorted(os.sched_getaffinity(0))}")
        except Exception as e:
            print(f"[WARN] cpu affinity failed: {e}")
    elif cpu_affinity:
        print("[WARN] sched_setaffinity not available")


def _yolo_device(prefer="auto"):
    if prefer != "auto":
        return prefer
    try:
        import torch
        if torch.cuda.is_available():
            return 0
    except Exception:
        pass
    return "cpu"


class DistanceApp:
    def __init__(self, npz_path, model_path, conf=DEFAULT_CONF,
                 infer_hz=DEFAULT_INFER_HZ, infer_imgsz=DEFAULT_IMGSZ, device="auto",
                 fast_render=True, use_half=True):
        self.quit = False
        self.pending_action = None
        self.conf = conf
        self.infer_dt = 0.0 if infer_hz <= 0 else (1.0 / infer_hz)
        self.infer_imgsz = infer_imgsz
        self.yolo_device = _yolo_device(device)
        self.fast_render = fast_render
        self.use_half = use_half
        self.model_backend = _model_backend(model_path)
        self.status = "Starting..."
        self.enabled = _default_enabled()
        self.dist_mode = _normal_dist_mode()
        self.detect_hold_sec = max(0.0, float(DETECTION_HOLD_SEC))
        self.max_detections = max(1, int(MAX_DETECTIONS))
        self.cluster_filter = True
        self.buttons = []

        self.lock = threading.Lock()
        self.pose = None
        self.pose_stamp = 0.0
        self.pose_history = []
        self.frame = None
        self.frame_id = 0
        self.frame_stamp = 0.0
        self.frame_pose = None
        self.frame_pose_dt = float("inf")
        self._last_render_fid = -1
        self.lidar = None
        self.lidar_seq = 0
        self.lidar_stamp = 0.0
        self.lidar_msg_stamp = 0.0
        self.lidar_pose = None
        self.lidar_pose_dt = float("inf")

        self._proj_key = None
        self._danger_uv = self._danger_vis = None
        self._danger_dx = self._danger_dy = None
        self._danger_base_uv = self._danger_base_vis = None
        self._danger_stamp_idx = None

        self._geo_key = None
        self._geo_lidar_seq = -1
        self._geo_pts_cam = self._geo_uv = self._geo_danger_cam = None
        self._geo_danger_uv = None
        self._geo_danger_sub_idx = None
        self._geo_danger_source = "MAP"
        self._geo_ui = self._geo_vi = self._geo_frame_ok = None
        self._geo_danger_tree = None

        self.K = self.dist = self.T_lidar_cam = self.T_imu_lidar = None
        self.proc_w, self.proc_h = 612, 512
        self.vx0 = self.vy0 = self.dw = self.dh = self.vscale = 0
        self.disp_cache = None
        self.anchor_image = None
        self.anchor_gray = None
        self.anchor_rect = None
        self.anchor_quality = 0.0
        self.anchor_enabled = False
        self._anchor_cache_key = None
        self._anchor_cache_time = 0.0
        self._anchor_cache = None
        self._anchor_draw_rect = None
        self._anchor_draw_score = 0.0
        self._anchor_draw_source = "MAP"
        self._anchor_align_score = 0.0
        self._anchor_align_on = False
        self._anchor_last_rect = None
        self._anchor_last_score = 0.0
        self._anchor_last_t = 0.0
        self._anchor_last_full_t = 0.0
        self._live_danger_uv = None

        self.model_path = model_path
        self.yolo = None
        self.yolo_ready = False
        self._yolo_event = threading.Event()
        self.infer_busy = False
        self.last_infer_t = 0.0
        self._last_infer_fid = -1

        self.result_lock = threading.Lock()
        self.dets = []
        self._tracks = []
        self._last_tracks = []
        self._last_track_t = 0.0
        self._dist_dt = 1.0 / max(DIST_HZ, 5.0)
        self._last_dist_t = 0.0
        self._last_anchor_idle_t = 0.0

        self._canvas = None
        self._btn_layer = None
        self._btn_dirty = True
        self._overlay_draw_dt = 1.0 / max(OVERLAY_HZ, 1.0)
        self._last_overlay_draw_t = 0.0
        self.danger_xyz = None
        self._perf = SegTimer()
        self._cuda_ok = False
        self._load_calib()
        self._load_npz(npz_path)
        self._setup_view()

    def _setup_view(self):
        self.vx0, self.vy0, self.dw, self.dh, self.vscale = view_rect(
            self.proc_w, self.proc_h, WIN_W, VIEW_H)
        self.disp_cache = None
        self._disp_buffers = [
            np.empty((self.dh, self.dw, 3), np.uint8),
            np.empty((self.dh, self.dw, 3), np.uint8),
        ]
        self._disp_write_idx = 0

    def _load_calib(self):
        K, dist, T_lc, T_il, scale = load_pinhole_calib()
        self.K, self.dist, self.T_lidar_cam, self.T_imu_lidar = K, dist, T_lc, T_il
        self.proc_w, self.proc_h = load_proc_image_size(scale)
        self._cuda_ok = _log_torch_cuda(self.yolo_device)
        print(f"[INFO] model={os.path.basename(self.model_path)} backend={self.model_backend} "
              f"render={'fast' if self.fast_render else 'full'}")
        print(f"[INFO] proc={self.proc_w}x{self.proc_h} imgsz={self.infer_imgsz} conf={self.conf}")
        print(f"[INFO] lidar_r_max={STAGE2_LIDAR_R_MAX}m lidar_max={LIDAR_MAX} "
              f"danger_dist_max={DANGER_DIST_MAX} danger_dot={DANGER_DOT_RADIUS} "
              f"bbox_expand={BBOX_EXPAND_RATIO} "
              f"safe_fallback={DIST_SAFE_FALLBACK}")
        print(f"[INFO] classes={DEFAULT_ON_CLASSES} dist_mode={self.dist_mode} "
              f"hold={self.detect_hold_sec:.2f}s max_dets={self.max_detections}")
        print(f"[INFO] lidar_stale={LIDAR_STALE_SEC:.2f}s dist_percentile={DIST_PERCENTILE:.1f} "
              f"mask_dilate={MASK_POINT_DILATE_PX}px")
        print(f"[INFO] pose_history={POSE_HISTORY_SEC:.1f}s pose_sync_max={POSE_SYNC_MAX_DT:.2f}s")
        print(f"[INFO] anchor_distance={int(bool(ANCHOR_DISTANCE))} match={int(bool(ANCHOR_MATCH))} "
              f"score>={ANCHOR_MIN_SCORE:.2f} min_pts={ANCHOR_MIN_REGION_PTS}")

    def _predict_kwargs(self):
        kw = dict(verbose=False, imgsz=self.infer_imgsz, device=self.yolo_device, conf=self.conf)
        if self.model_backend != "TensorRT" and self.use_half and self._cuda_ok:
            kw["half"] = True
        return kw

    def _load_npz(self, path):
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        data = np.load(path, allow_pickle=True)
        self.danger_xyz = data["xyz"].astype(np.float32)
        n = len(self.danger_xyz)
        if n > DANGER_DIST_MAX:
            self._danger_sub_idx = np.linspace(0, n - 1, DANGER_DIST_MAX, dtype=np.int32)
            self.danger_dist_xyz = self.danger_xyz[self._danger_sub_idx]
        else:
            self._danger_sub_idx = None
            self.danger_dist_xyz = self.danger_xyz
        print(f"[INFO] danger points: {n} (dist use {len(self.danger_dist_xyz)})")
        self._load_anchor_from_npz(data)

    def _load_anchor_from_npz(self, data):
        self.anchor_enabled = False
        self.anchor_image = None
        self.anchor_gray = None
        self.anchor_rect = None
        self.anchor_quality = 0.0
        if not ANCHOR_DISTANCE or "anchor_image" not in data or "anchor_rect" not in data:
            print("[INFO] anchor distance: OFF/none")
            return
        try:
            img = data["anchor_image"].astype(np.uint8)
            rect = data["anchor_rect"].astype(np.float32).reshape(4)
            quality = float(data["anchor_quality"]) if "anchor_quality" in data else 0.0
            if img.ndim != 3 or img.shape[0] < 8 or img.shape[1] < 8:
                print("[WARN] anchor distance: bad image")
                return
            if quality <= 0.0:
                gray0 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                quality = float(cv2.Laplacian(gray0, cv2.CV_32F).std())
            if quality < float(ANCHOR_MIN_TEXTURE):
                print(f"[WARN] anchor distance: texture weak {quality:.1f} < {ANCHOR_MIN_TEXTURE:.1f}")
                return
            self.anchor_image = img
            self.anchor_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            self.anchor_rect = rect
            self.anchor_quality = quality
            self.anchor_enabled = True
            print(f"[INFO] anchor distance: ON rect={tuple(rect.astype(int))} "
                  f"size={img.shape[1]}x{img.shape[0]} quality={quality:.1f}")
        except Exception as e:
            print(f"[WARN] anchor distance disabled: {e}")

    def active_class_ids(self):
        ids = []
        for k, on in self.enabled.items():
            if on:
                ids.extend(CLASS_DEFS[k]["ids"])
        return ids

    def active_labels(self):
        return [CLASS_DEFS[k]["label"] for k, on in self.enabled.items() if on]

    def _id_to_key(self, cls_id):
        cid = int(cls_id)
        for k, d in CLASS_DEFS.items():
            if cid in d["ids"]:
                return k
        return None

    def _remember_pose_locked(self, stamp, pose):
        self.pose = pose
        self.pose_stamp = stamp
        self.pose_history.append((stamp, pose))
        cutoff = stamp - max(0.5, float(POSE_HISTORY_SEC))
        while self.pose_history and self.pose_history[0][0] < cutoff:
            self.pose_history.pop(0)

    def _nearest_pose_locked(self, stamp):
        if not self.pose_history:
            return self.pose, float("inf")
        if stamp <= 0:
            t, pose = self.pose_history[-1]
            return pose, 0.0 if t > 0 else float("inf")
        for i in range(1, len(self.pose_history)):
            t0, p0 = self.pose_history[i - 1]
            t1, p1 = self.pose_history[i]
            if t0 <= stamp <= t1:
                span = max(t1 - t0, 1e-6)
                return interp_pose(p0, p1, (stamp - t0) / span), 0.0
        best_i = min(range(len(self.pose_history)),
                     key=lambda i: abs(self.pose_history[i][0] - stamp))
        t, pose = self.pose_history[best_i]
        return pose, abs(t - stamp)

    def _pose_key(self, pose):
        if pose is None:
            return None
        return tuple(np.round(pose[0], 3)) + tuple(np.round(pose[1], 3))

    def _global_to_cam(self, pts_global, pose):
        if pts_global is None or len(pts_global) == 0:
            return np.empty((0, 3), np.float32)
        T_w_l = T_world_from_imu_pose(pose[0], pose[1], self.T_imu_lidar)
        return apply_T(self.T_lidar_cam, apply_T(inv_T(T_w_l), pts_global))

    def _geo_cache(self, pose, lidar, lidar_seq):
        if lidar is None or len(lidar) == 0:
            return None, None, None, None, None, "MAP"
        key = self._pose_key(pose)
        if key is None:
            key = ("NOPOSE",)
        if key == self._geo_key and lidar_seq == self._geo_lidar_seq:
            return (self._geo_pts_cam, self._geo_uv, self._geo_danger_cam,
                    self._geo_danger_uv, self._geo_danger_sub_idx, self._geo_danger_source)
        pts_cam = apply_T(self.T_lidar_cam, lidar)
        uv, _ = project_uv_pinhole(pts_cam, self.K, self.dist)
        ui = np.round(uv[:, 0]).astype(np.int32)
        vi = np.round(uv[:, 1]).astype(np.int32)
        frame_ok = ((ui >= 0) & (ui < self.proc_w) & (vi >= 0) & (vi < self.proc_h) &
                    (pts_cam[:, 2] > 0.05))
        danger_cam = (self._global_to_cam(self.danger_dist_xyz, pose)
                      if pose is not None else None)
        danger_tree = (cKDTree(danger_cam) if cKDTree is not None and
                       danger_cam is not None and len(danger_cam) > 0 else None)
        danger_uv = None
        danger_sub_idx = self._danger_sub_idx
        danger_source = "MAP"
        self._geo_key, self._geo_lidar_seq = key, lidar_seq
        self._geo_pts_cam, self._geo_uv, self._geo_danger_cam = pts_cam, uv, danger_cam
        self._geo_ui, self._geo_vi, self._geo_frame_ok = ui, vi, frame_ok
        self._geo_danger_tree = danger_tree
        self._geo_danger_uv = danger_uv
        self._geo_danger_sub_idx = danger_sub_idx
        self._geo_danger_source = danger_source
        return pts_cam, uv, danger_cam, danger_uv, danger_sub_idx, danger_source

    def _extract_mask(self, result, idx):
        if result.masks is None:
            return None
        try:
            seg = result.masks.xy[idx]
            if seg is not None and len(seg) >= 3:
                mask = np.zeros((self.proc_h, self.proc_w), dtype=np.uint8)
                cv2.fillPoly(mask, [np.asarray(seg, np.int32).reshape(-1, 1, 2)], 1)
                if mask.sum() > 0:
                    return mask.astype(bool)
        except Exception:
            pass
        try:
            md = result.masks.data[idx].cpu().numpy()
            md = cv2.resize(md, (self.proc_w, self.proc_h), interpolation=cv2.INTER_NEAREST)
            return md > 0.5
        except Exception:
            return None

    def _prepared_track_mask(self, track):
        mask = track.get("mask")
        if mask is None:
            return None
        r = max(0, int(MASK_POINT_DILATE_PX))
        if track.get("_mask_prepared_r") == r and track.get("_mask_prepared") is not None:
            return track["_mask_prepared"]
        if r > 0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
            mask_use = cv2.dilate(mask.astype(np.uint8), k) > 0
        else:
            mask_use = mask
        track["_mask_prepared_r"] = r
        track["_mask_prepared"] = mask_use
        return mask_use

    def _pts_in_mask(self, mask, pts_cam, uv):
        ui, vi, ok = self._geo_ui, self._geo_vi, self._geo_frame_ok
        if ui is None or vi is None or ok is None or len(ui) != len(pts_cam):
            ui = np.round(uv[:, 0]).astype(np.int32)
            vi = np.round(uv[:, 1]).astype(np.int32)
            ok = ((ui >= 0) & (ui < self.proc_w) & (vi >= 0) & (vi < self.proc_h) &
                  (pts_cam[:, 2] > 0.05))
        if not np.any(ok):
            return np.empty((0, 3), np.float32)
        return pts_cam[ok][mask[vi[ok], ui[ok]]]

    def _pts_in_bbox(self, xyxy, pts_cam, uv, expand_ratio=BBOX_EXPAND_RATIO):
        if xyxy is None:
            return np.empty((0, 3), np.float32)
        x1, y1, x2, y2 = [float(v) for v in xyxy]
        w, h = max(1.0, x2 - x1), max(1.0, y2 - y1)
        pad_x, pad_y = w * float(expand_ratio), h * float(expand_ratio)
        x1, x2 = max(0.0, x1 - pad_x), min(float(self.proc_w - 1), x2 + pad_x)
        y1, y2 = max(0.0, y1 - pad_y), min(float(self.proc_h - 1), y2 + pad_y)
        ui, vi, frame_ok = self._geo_ui, self._geo_vi, self._geo_frame_ok
        if ui is None or vi is None or frame_ok is None or len(ui) != len(pts_cam):
            ui = np.round(uv[:, 0]).astype(np.int32)
            vi = np.round(uv[:, 1]).astype(np.int32)
            frame_ok = pts_cam[:, 2] > 0.05
        ok = (frame_ok & (ui >= x1) & (ui <= x2) & (vi >= y1) & (vi <= y2))
        if not np.any(ok):
            return np.empty((0, 3), np.float32)
        return pts_cam[ok]

    def _anchor_search_window(self, rect, margin):
        if rect is None:
            return None
        x, y, w, h = [float(v) for v in rect]
        mx = max(w * float(margin), 40.0)
        my = max(h * float(margin), 40.0)
        x1 = max(0.0, x - mx)
        y1 = max(0.0, y - my)
        x2 = min(float(self.proc_w), x + w + mx)
        y2 = min(float(self.proc_h), y + h + my)
        if x2 - x1 < 12 or y2 - y1 < 12:
            return None
        return x1, y1, x2, y2

    def _match_template_in_window(self, gray_w, tmpl_base, work_scale, window, scales):
        if window is None:
            roi = gray_w
            ox = oy = 0.0
        else:
            x1, y1, x2, y2 = window
            sx1 = max(0, int(round(x1 * work_scale)))
            sy1 = max(0, int(round(y1 * work_scale)))
            sx2 = min(gray_w.shape[1], int(round(x2 * work_scale)))
            sy2 = min(gray_w.shape[0], int(round(y2 * work_scale)))
            if sx2 - sx1 < 12 or sy2 - sy1 < 12:
                return None, -1.0
            roi = gray_w[sy1:sy2, sx1:sx2]
            ox = sx1 / work_scale
            oy = sy1 / work_scale

        best_score, best_rect = -1.0, None
        for rel in scales:
            tw = int(round(tmpl_base.shape[1] * rel))
            th = int(round(tmpl_base.shape[0] * rel))
            if tw < 8 or th < 8 or tw >= roi.shape[1] or th >= roi.shape[0]:
                continue
            tmpl = cv2.resize(tmpl_base, (tw, th), interpolation=cv2.INTER_AREA)
            res = cv2.matchTemplate(roi, tmpl, cv2.TM_CCOEFF_NORMED)
            _, score, _, loc = cv2.minMaxLoc(res)
            if score > best_score:
                best_rect = (
                    ox + loc[0] / work_scale,
                    oy + loc[1] / work_scale,
                    tw / work_scale,
                    th / work_scale,
                )
                best_score = float(score)
        return best_rect, best_score

    def _match_anchor_rect(self, frame):
        if not self.anchor_enabled or frame is None:
            return None, 0.0
        if not ANCHOR_MATCH:
            x, y, w, h = [float(v) for v in self.anchor_rect]
            return (x, y, w, h), 1.0
        try:
            now = time.time()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tmpl0 = self.anchor_gray
            if tmpl0 is None or tmpl0.size == 0:
                return None, 0.0
            work_scale = 0.5 if gray.size > 280000 or tmpl0.size > 22000 else 1.0
            if work_scale != 1.0:
                gray_w = cv2.resize(gray, None, fx=work_scale, fy=work_scale,
                                    interpolation=cv2.INTER_AREA)
                tmpl_base = cv2.resize(tmpl0, None, fx=work_scale, fy=work_scale,
                                       interpolation=cv2.INTER_AREA)
            else:
                gray_w = gray
                tmpl_base = tmpl0

            local_scales = (0.94, 1.0, 1.06)
            full_scales = (0.90, 1.0, 1.12)
            best_rect, best_score = None, -1.0

            if self._anchor_last_rect is not None:
                window = self._anchor_search_window(self._anchor_last_rect, ANCHOR_SEARCH_MARGIN)
                best_rect, best_score = self._match_template_in_window(
                    gray_w, tmpl_base, work_scale, window, local_scales)

            full_due = (
                self._anchor_last_full_t <= 0.0 or
                now - self._anchor_last_full_t >= float(ANCHOR_FULL_SEARCH_INTERVAL)
            )
            need_full = (
                best_rect is None or
                best_score < float(ANCHOR_MIN_SCORE) or
                (self._anchor_last_rect is not None and full_due)
            )
            if need_full and full_due:
                full_rect, full_score = self._match_template_in_window(
                    gray_w, tmpl_base, work_scale, None, full_scales)
                self._anchor_last_full_t = now
                if full_score > best_score:
                    best_rect, best_score = full_rect, full_score

            if best_rect is None or best_score < float(ANCHOR_MIN_SCORE):
                if (self._anchor_last_rect is not None and
                        now - self._anchor_last_t <= float(ANCHOR_KEEP_SEC) and
                        self._anchor_last_score >= float(ANCHOR_MIN_SCORE)):
                    return self._anchor_last_rect, self._anchor_last_score
                return None, max(0.0, best_score)

            self._anchor_last_rect = best_rect
            self._anchor_last_score = float(best_score)
            self._anchor_last_t = now
            return best_rect, best_score
        except Exception as e:
            print(f"[WARN] anchor match: {e}")
            return None, 0.0

    def _pts_in_rect(self, rect, pts_cam, uv, expand_ratio=0.0):
        if rect is None or pts_cam is None or uv is None:
            return np.empty((0, 3), np.float32), np.empty((0, 2), np.float32)
        x, y, w, h = [float(v) for v in rect]
        pad_x, pad_y = w * float(expand_ratio), h * float(expand_ratio)
        x1, x2 = max(0.0, x - pad_x), min(float(self.proc_w - 1), x + w + pad_x)
        y1, y2 = max(0.0, y - pad_y), min(float(self.proc_h - 1), y + h + pad_y)
        ui = np.round(uv[:, 0]).astype(np.int32)
        vi = np.round(uv[:, 1]).astype(np.int32)
        ok = ((ui >= x1) & (ui <= x2) & (vi >= y1) & (vi <= y2) &
              (pts_cam[:, 2] > 0.05))
        if not np.any(ok):
            return np.empty((0, 3), np.float32), np.empty((0, 2), np.float32)
        return pts_cam[ok], uv[ok]

    def _anchor_danger(self, frame, frame_id, pts_cam, uv, lidar_seq):
        self._anchor_draw_source = "MAP"
        if not self.anchor_enabled or frame is None or pts_cam is None or uv is None:
            self._live_danger_uv = None
            self._anchor_draw_rect = None
            return None, None, "MAP"

        now = time.time()
        if (self._anchor_cache is not None and
                now - self._anchor_cache_time < float(ANCHOR_MATCH_INTERVAL)):
            rect, score = self._anchor_cache
        else:
            t_anchor = time.perf_counter()
            rect, score = self._match_anchor_rect(frame)
            self._perf.add("anchor", (time.perf_counter() - t_anchor) * 1000.0)
            self._anchor_cache = (rect, score)
            self._anchor_cache_time = now

        self._anchor_draw_rect = rect
        self._anchor_draw_score = score
        if rect is None:
            self._live_danger_uv = None
            return None, None, "MAP"

        danger_raw, danger_uv = self._pts_in_rect(
            rect, pts_cam, uv, expand_ratio=ANCHOR_BBOX_EXPAND_RATIO)
        if len(danger_raw) < int(ANCHOR_MIN_REGION_PTS):
            self._live_danger_uv = danger_uv
            return None, None, "MAP"

        danger_cam = front_depth_cluster(danger_raw, CLUSTER_Z_GAP, int(ANCHOR_MIN_REGION_PTS))
        if danger_cam is None or len(danger_cam) < int(ANCHOR_MIN_REGION_PTS):
            self._live_danger_uv = danger_uv
            return None, None, "MAP"
        if ANCHOR_DENSIFY:
            danger_cam = densify_planar_points(
                danger_cam,
                step=ANCHOR_DENSIFY_STEP,
                max_pts=ANCHOR_DENSIFY_MAX,
                max_thickness=max(0.10, CLUSTER_Z_GAP * 1.5),
            )
        self._live_danger_uv = danger_uv
        self._anchor_draw_source = "ANCHOR"
        return danger_cam, danger_uv, "ANCHOR"

    def _distance_from_points(self, pts_in, danger_cam, danger_sub_idx=None):
        n_raw = len(pts_in)
        pts_use = (front_depth_cluster(pts_in, CLUSTER_Z_GAP, MIN_MASK_PTS)
                   if self.cluster_filter and n_raw >= MIN_MASK_PTS else pts_in)
        if len(pts_use) >= MIN_MASK_PTS:
            tree = self._geo_danger_tree if danger_cam is self._geo_danger_cam else None
            if tree is not None:
                nearest, local = tree.query(pts_use, k=1)
                nearest = np.asarray(nearest, dtype=np.float32).reshape(-1)
                local = np.asarray(local, dtype=np.int32).reshape(-1)
                if len(nearest) == 0:
                    dist, didx = float("inf"), -1
                elif DIST_PERCENTILE > 0:
                    pct = float(np.clip(DIST_PERCENTILE, 0.0, 50.0))
                    rank = int(np.floor((pct / 100.0) * max(len(nearest) - 1, 0)))
                    pick = int(np.argpartition(nearest, rank)[rank])
                    dist, didx = float(nearest[pick]), int(local[pick])
                else:
                    pick = int(np.argmin(nearest))
                    dist, didx = float(nearest[pick]), int(local[pick])
            elif DIST_PERCENTILE > 0:
                dist, didx = robust_dist_pts_to_danger(pts_use, danger_cam, DIST_PERCENTILE)
            else:
                dist, didx = min_dist_pts_to_danger(pts_use, danger_cam)
            if danger_sub_idx is not None and didx >= 0:
                didx = int(danger_sub_idx[didx])
            if dist < float("inf"):
                return dist, didx, len(pts_use), n_raw
        return None, -1, len(pts_use), n_raw

    def _fallback_distance(self, reason, n_pts=0, n_raw=0):
        if DIST_SAFE_FALLBACK:
            return {"dist": 0.0, "dist_valid": False, "dist_status": reason,
                    "danger_idx": -1, "lidar_pts": n_pts, "lidar_raw": n_raw}
        return {"dist": float("inf"), "dist_valid": False, "dist_status": reason,
                "danger_idx": -1, "lidar_pts": n_pts, "lidar_raw": n_raw}

    def _source_status(self, source, danger_source="MAP"):
        prefix = str(danger_source or "MAP").upper()
        if DIST_PERCENTILE > 0:
            return f"{prefix}_{source}_P{int(round(DIST_PERCENTILE))}"
        return f"{prefix}_{source}_MIN"

    def _distance_for_track(self, track, pts_cam, uv, danger_cam,
                            danger_uv=None, danger_sub_idx=None, danger_source="MAP"):
        forced = track.get("force_stop_reason")
        if forced:
            return self._fallback_distance(forced)
        if danger_cam is None or len(danger_cam) == 0:
            return self._fallback_distance("NO_REGION")

        mask = track.get("mask")
        n_pts = n_raw = 0
        if self.dist_mode != "bbox_only" and mask is not None:
            pts_mask = self._pts_in_mask(self._prepared_track_mask(track), pts_cam, uv)
            dist, didx, n_pts, n_raw = self._distance_from_points(
                pts_mask, danger_cam, danger_sub_idx)
            if dist is not None:
                out = {"dist": dist, "dist_valid": True,
                       "dist_status": self._source_status("MASK", danger_source),
                       "danger_idx": didx, "lidar_pts": n_pts, "lidar_raw": n_raw,
                       "danger_source": danger_source}
                if danger_source == "ANCHOR" and danger_uv is not None and 0 <= didx < len(danger_uv):
                    out["danger_uv"] = danger_uv[didx].astype(np.float32)
                return out

        if self.dist_mode != "mask_only":
            passes = [(0.03, "BBOX_CORE" if mask is not None else "BOX_CORE")]
            if float(BBOX_EXPAND_RATIO) > 0.031:
                passes.append((BBOX_EXPAND_RATIO, "BBOX_PAD" if mask is not None else "BOX_PAD"))
            for expand, status in passes:
                pts_box = self._pts_in_bbox(track.get("xyxy"), pts_cam, uv, expand_ratio=expand)
                dist, didx, n_pts, n_raw = self._distance_from_points(
                    pts_box, danger_cam, danger_sub_idx)
                if dist is not None:
                    out = {"dist": dist, "dist_valid": True,
                           "dist_status": self._source_status(status, danger_source),
                           "danger_idx": didx, "lidar_pts": n_pts, "lidar_raw": n_raw,
                           "danger_source": danger_source}
                    if danger_source == "ANCHOR" and danger_uv is not None and 0 <= didx < len(danger_uv):
                        out["danger_uv"] = danger_uv[didx].astype(np.float32)
                    return out

        return self._fallback_distance("STOP:NO_POINTS", n_pts=n_pts, n_raw=n_raw)

    def _fill_distances(self, tracks, pose, lidar, lidar_seq):
        if not tracks:
            return []
        pts_cam, uv, danger_cam, danger_uv, danger_sub_idx, danger_source = self._geo_cache(
            pose, lidar, lidar_seq)
        if pts_cam is None:
            reason = "NO_POSE" if pose is None else "NO_LIDAR"
            return [{**t, **self._fallback_distance(t.get("force_stop_reason") or reason)}
                    for t in tracks]
        out = []
        for t in tracks:
            out.append({
                **t,
                **self._distance_for_track(
                    t, pts_cam, uv, danger_cam,
                    danger_uv=danger_uv,
                    danger_sub_idx=danger_sub_idx,
                    danger_source=danger_source,
                ),
            })
        return out

    def _lidar_state(self, lidar, stamp, now=None):
        if now is None:
            now = time.time()
        if lidar is None or len(lidar) == 0 or stamp <= 0:
            return False, "NO_LIDAR", float("inf")
        age = max(0.0, now - stamp)
        if age > LIDAR_STALE_SEC:
            return False, "STALE_LIDAR", age
        return True, "OK", age

    def _remember_tracks_for_hold(self, tracks):
        self._last_tracks = [{k: v for k, v in t.items() if k != "mask"} for t in tracks]
        self._last_track_t = time.time()

    def _held_tracks(self):
        if self.detect_hold_sec <= 0.0 or not self._last_tracks:
            return []
        if time.time() - self._last_track_t > self.detect_hold_sec:
            return []
        held = []
        for t in self._last_tracks:
            if not self.enabled.get(t.get("key"), False):
                continue
            h = dict(t)
            h["mask"] = None
            h["held"] = True
            h["force_stop_reason"] = "STOP:DET_LOST"
            held.append(h)
        return held[:self.max_detections]

    def _yolo_loader(self):
        try:
            if not YOLO_OK:
                self.status = "ERROR: pip install ultralytics"
                return
            if not os.path.isfile(self.model_path):
                self.status = "ERROR: model missing"
                return
            print(f"[INFO] loading {self.model_path} ({self.model_backend})")
            self.status = "Loading YOLO seg..."
            model = YOLO(self.model_path)
            dummy = np.zeros((self.proc_h, self.proc_w, 3), np.uint8)
            model.predict(dummy, classes=(self.active_class_ids() or BOTTLE_IDS),
                          **self._predict_kwargs())
            self.yolo = model
            self.yolo_ready = True
            self.status = "Ready"
            print("[INFO] YOLO seg ready")
        except Exception as e:
            self.status = f"YOLO failed: {e}"
            print(f"[ERR] YOLO: {e}")
        finally:
            self._yolo_event.set()

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg_stamp_sec(msg, fallback=time.time(), max_wall_skew=1.0)
        pose = (np.array([p.x, p.y, p.z], np.float32),
                np.array([q.x, q.y, q.z, q.w], np.float32))
        with self.lock:
            self._remember_pose_locked(stamp, pose)

    def on_image(self, msg):
        stamp = msg_stamp_sec(msg, fallback=time.time(), max_wall_skew=1.0)
        img = ros_image_to_bgr(msg)
        if img is None:
            return
        if img.shape[1] != self.proc_w or img.shape[0] != self.proc_h:
            img = cv2.resize(img, (self.proc_w, self.proc_h), interpolation=cv2.INTER_LINEAR)
        with self.lock:
            write_idx = self._disp_write_idx
            disp = self._disp_buffers[write_idx]
        cv2.resize(img, (self.dw, self.dh), dst=disp, interpolation=cv2.INTER_LINEAR)
        with self.lock:
            self.frame = img
            self.frame_stamp = stamp
            self.frame_pose, self.frame_pose_dt = self._nearest_pose_locked(stamp)
            self.disp_cache = disp
            self._disp_write_idx = 1 - write_idx
            self.frame_id += 1

    def on_lidar(self, msg):
        msg_stamp = msg_stamp_sec(msg, fallback=time.time(), max_wall_skew=1.0)
        pts = parse_livox_msg(msg, max_pts=LIDAR_MAX, r_min=0.15, r_max=STAGE2_LIDAR_R_MAX)
        with self.lock:
            self.lidar = pts if len(pts) else None
            self.lidar_seq += 1
            self.lidar_stamp = time.time()
            self.lidar_msg_stamp = msg_stamp
            self.lidar_pose, self.lidar_pose_dt = self._nearest_pose_locked(msg_stamp)

    def _dist_loop(self):
        while not self.quit:
            now = time.time()
            if now - self._last_dist_t < self._dist_dt:
                time.sleep(0.005)
                continue
            self._last_dist_t = now
            with self.lock:
                pose, lidar, lidar_seq = self.pose, self.lidar, self.lidar_seq
                lidar_stamp = self.lidar_stamp
                lidar_pose, lidar_pose_dt = self.lidar_pose, self.lidar_pose_dt
            with self.result_lock:
                tracks = list(self._tracks)
            t0 = time.perf_counter()
            lidar_ok, lidar_reason, _ = self._lidar_state(lidar, lidar_stamp, now)
            sync_pose = lidar_pose if lidar_pose is not None else pose
            pose_sync_ok = (lidar_pose_dt <= POSE_SYNC_MAX_DT) or lidar_pose_dt == float("inf")
            pose_blocked = sync_pose is None and not self.anchor_enabled
            sync_blocked = (not pose_sync_ok) and not self.anchor_enabled
            if tracks and (pose_blocked or not lidar_ok or sync_blocked):
                if pose_blocked:
                    reason = "NO_POSE"
                elif not lidar_ok:
                    reason = lidar_reason
                else:
                    reason = "STOP:POSE_SYNC"
                dets = [
                    {**t, **self._fallback_distance(t.get("force_stop_reason") or reason)}
                    for t in tracks
                ]
            else:
                if tracks:
                    dets = self._fill_distances(tracks, sync_pose, lidar, lidar_seq)
                else:
                    dets = []
                    idle_hz = float(ANCHOR_IDLE_HZ)
                    if self.anchor_enabled and lidar_ok and idle_hz > 0:
                        idle_dt = 1.0 / max(idle_hz, 0.1)
                        if now - self._last_anchor_idle_t >= idle_dt:
                            self._geo_cache(sync_pose, lidar, lidar_seq)
                            self._last_anchor_idle_t = now
            self._perf.add("dist", (time.perf_counter() - t0) * 1000.0)
            with self.result_lock:
                self.dets = dets

    def _infer_loop(self):
        while not self.quit:
            if not self.yolo_ready:
                time.sleep(0.1)
                continue
            active_ids = self.active_class_ids()
            if not active_ids:
                with self.result_lock:
                    self.dets, self._tracks = [], []
                    self._last_tracks = []
                time.sleep(0.1)
                continue
            now = time.time()
            if self.infer_dt > 0 and now - self.last_infer_t < self.infer_dt:
                time.sleep(0.02)
                continue
            if self.infer_busy:
                time.sleep(0.02)
                continue
            with self.lock:
                frame, fid = self.frame, self.frame_id
            if frame is None or fid == self._last_infer_fid:
                time.sleep(0.02)
                continue
            self.infer_busy = True
            self.last_infer_t = now
            self._last_infer_fid = fid
            try:
                t0 = time.perf_counter()
                r = self.yolo.predict(
                    frame, classes=active_ids, **self._predict_kwargs())[0]
                self._perf.add("infer", (time.perf_counter() - t0) * 1000.0)
                tracks = []
                if r.boxes is not None:
                    t_mask = time.perf_counter()
                    for i, box in enumerate(r.boxes):
                        key = self._id_to_key(int(box.cls[0]))
                        if key is None or not self.enabled.get(key, False):
                            continue
                        xyxy = box.xyxy[0]
                        if hasattr(xyxy, "cpu"):
                            xyxy = xyxy.cpu().numpy()
                        conf = 0.0
                        try:
                            conf = float(box.conf[0])
                        except Exception:
                            pass
                        tracks.append({
                            "key": key,
                            "xyxy": [float(v) for v in xyxy],
                            "mask": None if self.dist_mode == "bbox_only" else self._extract_mask(r, i),
                            "conf": conf,
                        })
                    self._perf.add("mask", (time.perf_counter() - t_mask) * 1000.0)
                tracks.sort(key=lambda t: t.get("conf", 0.0), reverse=True)
                tracks = tracks[:self.max_detections]
                if tracks:
                    self._remember_tracks_for_hold(tracks)
                else:
                    tracks = self._held_tracks()
                with self.result_lock:
                    self._tracks = tracks
                    self.dets = [
                        {**t, **self._fallback_distance(t.get("force_stop_reason") or "DETECTING")}
                        for t in tracks
                    ]
            except Exception as e:
                print(f"[WARN] infer: {e}")
                self._last_infer_fid = -1
            finally:
                self.infer_busy = False

    def _build_buttons(self):
        class_specs = [
            {"id": "toggle_cup", "label": "Bottle", "key": "cup"},
            {"id": "toggle_person", "label": "Person", "key": "person"},
            {"id": "toggle_car", "label": "Car", "key": "car"},
        ]
        specs = class_specs + [
            {"id": "toggle_cluster", "label": "Point Filter", "key": "cluster"},
            {"id": "quit", "label": "Exit", "key": None},
        ]
        status_h = 26
        bh = BTN_BAR_H - BTN_MARGIN * 2 - status_h
        by = VIEW_H + BTN_MARGIN
        self.buttons = layout_buttons(
            specs, BTN_MARGIN, by, WIN_W - BTN_MARGIN * 2, bh, BTN_GAP)

    def _hit_button(self, x, y):
        button = hit_button(self.buttons, x, y)
        return None if button is None else button["id"]

    def _process_action(self, bid):
        if bid == "quit":
            self.quit = True
        elif bid == "toggle_cup":
            self.enabled["cup"] = not self.enabled["cup"]
        elif bid == "toggle_person":
            self.enabled["person"] = not self.enabled["person"]
        elif bid == "toggle_car":
            self.enabled["car"] = not self.enabled["car"]
        elif bid == "toggle_cluster":
            self.cluster_filter = not self.cluster_filter
            print(f"[INFO] front cluster -> {'ON' if self.cluster_filter else 'OFF'}")
        with self.result_lock:
            self._tracks = [t for t in self._tracks if self.enabled.get(t.get("key"), False)]
            self.dets = [t for t in self.dets if self.enabled.get(t.get("key"), False)]
            self._last_tracks = [
                t for t in self._last_tracks if self.enabled.get(t.get("key"), False)]
        on = ",".join(self.active_labels()) or "none"
        f = "ON" if self.cluster_filter else "OFF"
        self.status = f"Active: {on}   Point filter: {f}"
        self._btn_dirty = True

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:
            bid = self._hit_button(x, y)
            if bid:
                self.pending_action = bid

    def _proj_indices(self):
        """Only project danger points needed for dots + distance lines."""
        n = len(self.danger_xyz)
        nd = min(max(int(DANGER_DRAW_MAX), 1), n)
        draw_idx = np.linspace(0, n - 1, nd, dtype=np.int32)
        if self._danger_sub_idx is not None:
            return np.unique(np.concatenate([draw_idx, self._danger_sub_idx.astype(np.int32)]))
        return draw_idx

    def _update_danger_proj(self, pose):
        key = self._pose_key(pose)
        if key is None:
            self._proj_key = None
            self._danger_uv = self._danger_vis = None
            self._danger_dx = self._danger_dy = None
            self._danger_base_uv = self._danger_base_vis = None
            self._danger_stamp_idx = None
            return 0
        if key == self._proj_key:
            return 0 if self._danger_dx is None else len(self._danger_dx)
        need_idx = self._proj_indices()
        d_cam = self._global_to_cam(self.danger_xyz[need_idx], pose)
        uv_s, Z_s = project_uv_pinhole(d_cam, self.K, self.dist)
        n = len(self.danger_xyz)
        uv = np.zeros((n, 2), np.float32)
        vis = np.zeros(n, np.bool_)
        uv[need_idx] = uv_s
        vis[need_idx] = Z_s > 0.05
        nd = min(max(int(DANGER_DRAW_MAX), 1), n)
        draw_idx = np.linspace(0, n - 1, nd, dtype=np.int32)
        stamp_idx = draw_idx[vis[draw_idx]]
        self._proj_key = key
        self._danger_uv, self._danger_vis = uv, vis
        self._danger_base_uv, self._danger_base_vis = uv, vis
        self._danger_stamp_idx = stamp_idx
        sc, x0, y0 = self.vscale, self.vx0, self.vy0
        self._danger_dx = (x0 + uv[stamp_idx, 0] * sc).astype(np.int32)
        self._danger_dy = (y0 + uv[stamp_idx, 1] * sc).astype(np.int32)
        return len(stamp_idx)

    def _matched_anchor_rect(self, frame):
        if not self.anchor_enabled or frame is None:
            self._anchor_draw_rect = None
            self._anchor_align_on = False
            self._anchor_align_score = 0.0
            return None, 0.0
        now = time.time()
        if (self._anchor_cache is not None and
                now - self._anchor_cache_time < float(ANCHOR_MATCH_INTERVAL)):
            rect, score = self._anchor_cache
        else:
            t_anchor = time.perf_counter()
            rect, score = self._match_anchor_rect(frame)
            self._perf.add("anchor", (time.perf_counter() - t_anchor) * 1000.0)
            self._anchor_cache = (rect, score)
            self._anchor_cache_time = now
        self._anchor_draw_rect = rect
        self._anchor_draw_score = score
        return rect, score

    def _apply_anchor_projection_correction(self, frame):
        if self._danger_base_uv is None or self._danger_base_vis is None:
            self._anchor_align_on = False
            return

        uv = self._danger_base_uv
        vis = self._danger_base_vis
        stamp_idx = self._danger_stamp_idx
        if stamp_idx is None:
            stamp_idx = np.flatnonzero(vis)

        rect, score = self._matched_anchor_rect(frame)
        corrected = uv
        self._anchor_align_on = False
        self._anchor_align_score = float(score)

        if rect is not None and score >= float(ANCHOR_MIN_SCORE) and len(stamp_idx) >= 4:
            pts = uv[stamp_idx]
            ok = np.isfinite(pts).all(axis=1)
            pts = pts[ok]
            if len(pts) >= 4:
                x0, y0 = pts.min(axis=0)
                x1, y1 = pts.max(axis=0)
                bw, bh = max(float(x1 - x0), 1.0), max(float(y1 - y0), 1.0)
                rx, ry, rw, rh = [float(v) for v in rect]
                sx = float(np.clip(rw / bw, 0.65, 1.55))
                sy = float(np.clip(rh / bh, 0.65, 1.55))
                src_cx, src_cy = (x0 + x1) * 0.5, (y0 + y1) * 0.5
                dst_cx, dst_cy = rx + rw * 0.5, ry + rh * 0.5
                corrected = uv.copy()
                corrected[:, 0] = (uv[:, 0] - src_cx) * sx + dst_cx
                corrected[:, 1] = (uv[:, 1] - src_cy) * sy + dst_cy
                self._anchor_align_on = True

        self._danger_uv, self._danger_vis = corrected, vis
        draw_idx = stamp_idx
        sc, x0, y0 = self.vscale, self.vx0, self.vy0
        self._danger_dx = (x0 + corrected[draw_idx, 0] * sc).astype(np.int32)
        self._danger_dy = (y0 + corrected[draw_idx, 1] * sc).astype(np.int32)

    def _init_canvas(self):
        canvas = np.full((WIN_H, WIN_W, 3), UI_BG, np.uint8)
        self._canvas = canvas

    def _draw_button_bar(self):
        canvas = self._canvas
        bar = canvas[VIEW_H:WIN_H, 0:WIN_W]
        cv2.rectangle(canvas, (0, VIEW_H), (WIN_W - 1, WIN_H - 1), UI_PANEL, -1)
        for b in self.buttons:
            if b["id"] == "quit":
                draw_button(canvas, b, active=True, destructive=True)
                continue
            if b["key"] == "cluster":
                on = self.cluster_filter
                draw_button(canvas, b, active=on, accent=FRONT_ON,
                            state_text="ON" if on else "OFF")
            else:
                on = self.enabled.get(b["key"], False)
                draw_button(canvas, b, active=on, accent=CLASS_DEFS[b["key"]]["btn_on"],
                            state_text="ON" if on else "OFF")
        put_text_fit(canvas, self.status, BTN_MARGIN, WIN_H - 7,
                     WIN_W - BTN_MARGIN * 2, color=(205, 210, 214),
                     base=0.46, minimum=0.38, thickness=1)
        self._btn_layer = bar.copy()
        self._btn_dirty = False

    def _blit_button_bar(self):
        if self._btn_layer is not None:
            cv2.copyTo(self._btn_layer, None, self._canvas[VIEW_H:WIN_H, 0:WIN_W])

    def _blit_camera(self, canvas):
        x0, y0, dw, dh = self.vx0, self.vy0, self.dw, self.dh
        cv2.rectangle(canvas, (0, 0), (WIN_W - 1, VIEW_H - 1), UI_BG, -1)
        with self.lock:
            if self.frame is None or self.disp_cache is None:
                return False
            cv2.copyTo(self.disp_cache, None, canvas[y0:y0 + dh, x0:x0 + dw])
        return True

    def _draw_overlay(self, canvas, dets, pose, lidar_state, lidar_age, pose_sync_dt=None):
        x0, y0, dw, dh, sc = self.vx0, self.vy0, self.dw, self.dh, self.vscale
        self._update_danger_proj(pose)
        if self.anchor_enabled:
            with self.lock:
                frame_for_anchor = None if self.frame is None else self.frame.copy()
            self._apply_anchor_projection_correction(frame_for_anchor)
        nproj = (stamp_bgr(canvas, self._danger_dx, self._danger_dy, radius=DANGER_DOT_RADIUS)
                 if self._danger_dx is not None else 0)
        if self.anchor_enabled and SHOW_ANCHOR_BOX and self._anchor_draw_rect is not None:
            rx, ry, rw, rh = self._anchor_draw_rect
            p1 = (int(x0 + rx * sc), int(y0 + ry * sc))
            p2 = (int(x0 + (rx + rw) * sc), int(y0 + (ry + rh) * sc))
            cv2.rectangle(canvas, p1, p2, (80, 80, 80), 1)

        pose_ok = pose is not None
        lidar_ok = lidar_state == "OK"
        sync_bad = (pose_sync_dt is not None and pose_sync_dt < float("inf") and
                    pose_sync_dt > POSE_SYNC_MAX_DT)
        problems = []
        if not lidar_ok:
            problems.append(friendly_reason(lidar_state))
        if not pose_ok:
            problems.append("NO SLAM POSE")
        elif sync_bad:
            problems.append("POSE NOT SYNCED")
        if not self.active_class_ids():
            problems.append("NO CLASS ENABLED")
        label_min_y = 104 if problems else 50

        for item in dets:
            key = item["key"]
            info = CLASS_DEFS[key]
            x1, y1, x2, y2 = item["xyxy"]
            dist = item["dist"]
            valid = bool(item.get("dist_valid", dist < float("inf")))
            status = item.get("dist_status", "MASK" if valid else "STOP")
            finite_dist = dist < float("inf")
            pending = not (valid and finite_dist)
            hard_error = status in ("NO_LIDAR", "STALE_LIDAR", "NO_POSE", "STOP:POSE_SYNC")
            col = _dist_color(dist) if not pending else (UI_RED if hard_error else UI_YELLOW)
            dx1, dy1 = int(x0 + x1 * sc), int(y0 + y1 * sc)
            dx2, dy2 = int(x0 + x2 * sc), int(y0 + y2 * sc)
            cv2.rectangle(canvas, (dx1, dy1), (dx2, dy2), col, 1 if pending else 2)
            bcx, bcy = (dx1 + dx2) // 2, dy2

            if not self.fast_render:
                mask = item.get("mask")
                if mask is not None:
                    m_disp = cv2.resize(mask.astype(np.uint8), (dw, dh),
                                        interpolation=cv2.INTER_NEAREST)
                    region = canvas[y0:y0 + dh, x0:x0 + dw]
                    sel = m_disp > 0
                    if np.any(sel):
                        base = region[sel].astype(np.float32)
                        tint = np.array(info["color"], np.float32)
                        region[sel] = np.clip(
                            base * (1 - MASK_ALPHA) + tint * MASK_ALPHA, 0, 255).astype(np.uint8)
                    off = np.array([[x0, y0]], np.int32)
                    for c in cv2.findContours(m_disp, cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_SIMPLE)[0]:
                        cv2.drawContours(canvas, [c + off], -1, col, 2)

            n_pts, n_raw = item.get("lidar_pts", 0), item.get("lidar_raw", 0)
            if valid and finite_dist:
                value = f"{dist:.2f} m"
                if n_pts > 0 and n_raw > n_pts:
                    pts_text = f"{n_pts}/{n_raw} lidar points"
                elif n_pts > 0:
                    pts_text = f"{n_pts} lidar points"
                else:
                    pts_text = "distance valid"
                method = "MASK" if "MASK" in status else "BOX"
                detail = f"{pts_text}  |  {method}"
            else:
                value = "--"
                detail = friendly_reason(status)
            draw_detection_label(
                canvas, dx1, dy1 - 64, info["label"], value, detail, col,
                min_y=label_min_y)

            didx = item.get("danger_idx", -1)
            item_uv = item.get("danger_uv")
            if valid and item_uv is not None:
                ddx = int(x0 + float(item_uv[0]) * sc)
                ddy = int(y0 + float(item_uv[1]) * sc)
                if 0 <= ddx < WIN_W and 0 <= ddy < VIEW_H:
                    cv2.line(canvas, (bcx, bcy), (ddx, ddy), col, 2, cv2.LINE_AA)
                    cv2.circle(canvas, (ddx, ddy), 5, (0, 0, 255), -1)
            elif (valid and item.get("danger_source") != "ANCHOR" and didx >= 0
                    and self._danger_uv is not None and self._danger_vis is not None
                    and didx < len(self._danger_vis) and self._danger_vis[didx]):
                ddx = int(x0 + self._danger_uv[didx, 0] * sc)
                ddy = int(y0 + self._danger_uv[didx, 1] * sc)
                if 0 <= ddx < WIN_W and 0 <= ddy < VIEW_H:
                    cv2.line(canvas, (bcx, bcy), (ddx, ddy), col, 2, cv2.LINE_AA)
                    cv2.circle(canvas, (ddx, ddy), 5, (0, 0, 255), -1)

        if lidar_ok and lidar_age < float("inf"):
            lidar_value = f"{lidar_age * 1000:.0f}ms"
        else:
            lidar_value = "OFF" if lidar_state == "NO_LIDAR" else "STALE"
        chips = [
            ("CAM", "ok", "LIVE"),
            ("LIDAR", "ok" if lidar_ok else "error", lidar_value),
            ("POSE", "ok" if pose_ok and not sync_bad else "error",
             "LIVE" if pose_ok and not sync_bad else "OFF"),
            ("REGION", "ok", str(len(self.danger_xyz))),
            ("MODEL", "ok" if self.yolo_ready else "warn",
             self.model_backend.upper()),
        ]
        draw_status_strip(canvas, chips, x=10, y=10)
        if problems:
            draw_banner(canvas, "DISTANCE PAUSED  |  " + "  /  ".join(problems),
                        level="error", y=52)

    def _render(self):
        if self._canvas is None:
            self._init_canvas()
        canvas = self._canvas

        t0 = time.perf_counter()
        has_cam = self._blit_camera(canvas)
        self._perf.add("cam", (time.perf_counter() - t0) * 1000.0)

        if not has_cam:
            canvas[0:VIEW_H, 0:WIN_W] = UI_BG
            draw_banner(canvas, "CAMERA OFF  |  CHECK CAMERA CONNECTION",
                        level="error", y=VIEW_H // 2 - 24)
        else:
            with self.lock:
                pose, lidar, lidar_stamp = self.pose, self.lidar, self.lidar_stamp
                frame_pose, frame_pose_dt = self.frame_pose, self.frame_pose_dt
            _, lidar_state, lidar_age = self._lidar_state(lidar, lidar_stamp)
            with self.result_lock:
                dets = list(self.dets)
            t1 = time.perf_counter()
            overlay_pose = frame_pose if frame_pose is not None else pose
            self._draw_overlay(canvas, dets, overlay_pose, lidar_state, lidar_age, frame_pose_dt)
            self._perf.add("overlay", (time.perf_counter() - t1) * 1000.0)

        if self._btn_dirty:
            self._draw_button_bar()
        else:
            self._blit_button_bar()
        return canvas

    def run(self):
        rospy.init_node("distance_fastlivo", anonymous=True)
        if not wait_ros_topics(["/left_camera/image", "/livox/lidar"]):
            print("[ERR] start Demo-Base first")
            return 1

        rospy.Subscriber("/aft_mapped_to_init", Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber("/left_camera/image", Image, self.on_image, queue_size=1)
        rospy.Subscriber("/livox/lidar", CustomMsg, self.on_lidar, queue_size=1)
        threading.Thread(target=rospy.spin, daemon=True).start()
        threading.Thread(target=self._yolo_loader, daemon=True).start()

        print("[INFO] waiting for YOLO...")
        if not self._yolo_event.wait(300) or not self.yolo_ready:
            print(f"[ERR] {self.status}")
            return 1

        threading.Thread(target=self._infer_loop, daemon=True).start()
        threading.Thread(target=self._dist_loop, daemon=True).start()

        win = "Power Equipment Safety Distance"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, WIN_W, WIN_H)
        cv2.setMouseCallback(win, self.on_mouse)
        self._build_buttons()
        self._init_canvas()
        self.status = (f"Active: {','.join(self.active_labels()) or 'none'}   "
                       f"Point filter: ON   Mode: {self.dist_mode}")
        self._draw_button_bar()
        ms = RENDER_MS
        print(f"[INFO] UI ~{1000 // max(ms, 1)}Hz cam+overlay | infer_hz={DEFAULT_INFER_HZ or 'max'} imgsz={DEFAULT_IMGSZ}")

        try:
            frame_dt = max(ms, 1) / 1000.0
            next_frame = time.perf_counter()
            while not rospy.is_shutdown() and not self.quit:
                if self.pending_action:
                    act = self.pending_action
                    self.pending_action = None
                    self._process_action(act)
                t0 = time.perf_counter()
                cv2.imshow(win, self._render())
                self._perf.add("render", (time.perf_counter() - t0) * 1000.0)
                next_frame += frame_dt
                delay = next_frame - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_frame = time.perf_counter()
                if cv2.waitKey(1) & 0xFF == 27:
                    break
        finally:
            self.quit = True
            cv2.destroyAllWindows()
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--danger-npz", default=DEFAULT_NPZ)
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--conf", type=float, default=DEFAULT_CONF)
    ap.add_argument("--infer-hz", type=float, default=DEFAULT_INFER_HZ)
    ap.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ)
    ap.add_argument("--device", default="auto")
    ap.add_argument("--full-render", action="store_true",
                    help="draw mask overlay+contours (overrides demo_config FAST_RENDER=1)")
    ap.add_argument("--no-half", action="store_true",
                    help="PyTorch .pt only: disable FP16 infer")
    ap.add_argument("--nice", type=int, default=None,
                    help="Target nice (-20 high .. 19 low). Default from run_3.sh / env YOLO_NICE")
    ap.add_argument("--cpu-affinity", default=None,
                    help="CPU cores for stage2, e.g. 3-5 (keep 0-2 for Base). ''=disable")
    args = ap.parse_args()

    nice = args.nice
    if nice is None and os.environ.get("YOLO_NICE") not in (None, ""):
        try:
            nice = int(os.environ["YOLO_NICE"])
        except ValueError:
            pass
    cpu_aff = args.cpu_affinity
    if cpu_aff is None:
        cpu_aff = os.environ.get("YOLO_CPU_AFFINITY", "")
    apply_sched_priority(nice=nice, cpu_affinity=cpu_aff)
    n_threads = _affinity_core_count(cpu_aff)
    if n_threads is None and os.environ.get("OMP_NUM_THREADS"):
        try:
            n_threads = int(os.environ["OMP_NUM_THREADS"])
        except ValueError:
            n_threads = None
    limit_cpu_threads(n_threads)

    npz_path = args.danger_npz if os.path.isabs(args.danger_npz) else os.path.join(SCRIPT_DIR, args.danger_npz)
    requested_model_path = args.model if os.path.isabs(args.model) else os.path.join(SCRIPT_DIR, args.model)
    model_path = resolve_model_path(requested_model_path, args.imgsz)
    if model_path.endswith(".pt") and _model_backend(model_path) == "PyTorch":
        engine = _engine_path_for(requested_model_path, args.imgsz)
        if not os.path.isfile(engine):
            print(f"[WARN] TensorRT engine not found: {engine}")
            print(f"       Suggest: bash ~/Desktop/my_demo/tools/export_trt.sh {args.imgsz} "
                  f"{os.path.basename(requested_model_path)}")

    try:
        app = DistanceApp(npz_path, model_path, conf=args.conf,
                          infer_hz=args.infer_hz, infer_imgsz=args.imgsz, device=args.device,
                          fast_render=False if args.full_render else FAST_RENDER,
                          use_half=not args.no_half)
    except FileNotFoundError:
        print(f"[ERR] npz not found: {npz_path}")
        print("      Run stage 1 (run_mark.sh) first.")
        return 1
    except Exception as e:
        print(f"[ERR] {e}")
        return 1
    return app.run()


if __name__ == "__main__":
    sys.exit(main())
