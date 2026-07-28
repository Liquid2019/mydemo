#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FAST-LIVO2 danger-point marking (touch UI, single window).

Tap bottom buttons: Draw ROI -> drag box -> Confirm Save
Undo / Cancel / Quit

Start FAST-LIVO2 before running this script.
"""

import os
os.environ["OMP_NUM_THREADS"] = "1"

import sys
import time
import argparse
import shutil
import threading
from collections import deque
import numpy as np
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
    view_rect,
    wait_ros_topics,
    voxel_downsample,
    front_depth_cluster,
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
    put_text_fit,
)

from demo_config import (
    DEFAULT_NPZ as NPZ_DEFAULT,
    RENDER_MS,
    ROI_SAVE_MAX,
    DANGER_TOTAL_MAX,
    ROI_VOXEL,
    CLUSTER_Z_GAP,
    POSE_HISTORY_SEC,
    POSE_SYNC_MAX_DT,
    ROI_DENSIFY,
    ROI_DENSIFY_STEP,
    ROI_DENSIFY_MAX,
    ROI_DENSIFY_MAX_THICKNESS,
    MARK_LINE_WIDTH_PX,
)

# ---------- 固定 UI 尺寸（触摸屏，全宽相机） ----------
WIN_W, WIN_H = 1280, 800
CAM_PANEL_H = 620
BTN_BAR_H = WIN_H - CAM_PANEL_H

# 性能参数（Jetson 7GB 实测调优）
ACCUM_MAX = 120000
LIDAR_MAX = 15000
DANGER_DRAW = 1500
ROI_ACCUM_MAX = 40000
ROI_ACCUM_SAMPLE = 12000

# Livox 采集最大距离
LIDAR_RANGE_PRESETS = (
    (10.0, "10m"),
    (20.0, "20m"),
    (50.0, "50m"),
)
RANGE_BTN_W, RANGE_BTN_H = 72, 40
RANGE_BTN_Y = 8
RANGE_BTN_GAP = 8
SENSOR_STALE_SEC = 0.75


class DangerMarkApp:
    def __init__(self, npz_path):
        self.npz_path = npz_path
        self.quit = False

        self.lock = threading.Lock()
        self.pose = None
        self.pose_stamp = 0.0
        self.pose_history = []
        self.frame = None
        self.disp_cache = None
        self.camera_stamp = 0.0
        self.frame_id = 0
        self.frame_pose = None
        self.frame_pose_dt = float("inf")
        self._last_render_fid = -1
        self._accum_chunks = deque()
        self._accum_total = 0
        self.latest_scan = np.zeros((0, 3), np.float32)
        self.lidar_stamp = 0.0
        self.latest_scan_pose = None
        self.latest_scan_pose_dt = float("inf")
        self.danger = []
        self.undo_stack = []
        self.anchor_image = None
        self.anchor_rect = None
        self.anchor_frame_shape = None
        self.anchor_quality = 0.0
        self._canvas = None

        self.pending_action = None
        self.toast_msg = ""
        self.toast_until = 0.0

        self.K = None
        self.dist = None
        self.T_lidar_cam = None
        self.T_imu_lidar = None
        self.proc_w = 612
        self.proc_h = 512

        self.roi_mode = False
        self.roi_drag = False
        self.roi_start = None
        self.roi_rect = None
        self.mark_mode = None
        self.mark_points = []
        self.buttons = []
        self.range_buttons = []
        self.lidar_range_idx = 0
        self.lidar_r_max = LIDAR_RANGE_PRESETS[0][0]
        self.status = "Starting..."

        self._load_calib()
        _, _, dw, dh, _ = self._cam_disp_rect()
        self._disp_buffers = [
            np.empty((dh, dw, 3), np.uint8),
            np.empty((dh, dw, 3), np.uint8),
        ]
        self._disp_write_idx = 0
        self._reset_npz()

    def _load_calib(self):
        K, dist, T_lc, T_il, scale = load_pinhole_calib()
        self.K = K
        self.dist = dist
        self.T_lidar_cam = T_lc
        self.T_imu_lidar = T_il
        self.proc_w, self.proc_h = load_proc_image_size(scale)
        print(f"[INFO] proc size {self.proc_w}x{self.proc_h}  Kfx={K[0,0]:.1f}")

    def _T_world_lidar(self, pose):
        return T_world_from_imu_pose(pose[0], pose[1], self.T_imu_lidar)

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

    def _reset_npz(self):
        """Fresh session: discard previous danger points."""
        self.danger = []
        self.undo_stack = []
        if os.path.isfile(self.npz_path):
            backup_dir = os.path.join(os.path.dirname(self.npz_path), "backups")
            os.makedirs(backup_dir, exist_ok=True)
            base = os.path.splitext(os.path.basename(self.npz_path))[0]
            backup = os.path.join(backup_dir, f"{base}_{time.strftime('%Y%m%d_%H%M%S')}.npz")
            shutil.copy2(self.npz_path, backup)
            os.remove(self.npz_path)
            print(f"[INFO] backup previous danger points: {backup}")
            print(f"[INFO] cleared previous danger points: {self.npz_path}")
        else:
            print("[INFO] starting with empty danger points")

    def save_npz(self):
        pts = np.asarray(self.danger, dtype=np.float32)
        payload = {
            "xyz": pts,
            "coord_frame": "camera_init",
            "created_time": np.float64(time.time()),
        }
        if self.anchor_image is not None and self.anchor_rect is not None:
            payload.update({
                "anchor_image": self.anchor_image.astype(np.uint8),
                "anchor_rect": np.asarray(self.anchor_rect, dtype=np.float32),
                "anchor_frame_shape": np.asarray(self.anchor_frame_shape, dtype=np.int32),
                "anchor_quality": np.float32(self.anchor_quality),
            })
        np.savez(self.npz_path, **payload)
        print(f"[OK] saved {pts.shape} -> {self.npz_path}")

    def _set_anchor_from_frame(self, frame, rect):
        if frame is None or rect is None:
            return
        rx, ry, rw, rh = [int(round(v)) for v in rect]
        h, w = frame.shape[:2]
        x1 = max(0, min(w - 1, rx))
        y1 = max(0, min(h - 1, ry))
        x2 = max(0, min(w, rx + rw))
        y2 = max(0, min(h, ry + rh))
        if x2 - x1 < 8 or y2 - y1 < 8:
            return
        crop = frame[y1:y2, x1:x2].copy()
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        quality = float(cv2.Laplacian(gray, cv2.CV_32F).std())
        self.anchor_image = crop
        self.anchor_rect = np.array([x1, y1, x2 - x1, y2 - y1], dtype=np.float32)
        self.anchor_frame_shape = np.array([h, w], dtype=np.int32)
        self.anchor_quality = quality
        print(f"[ANCHOR] rect={tuple(self.anchor_rect.astype(int))} quality={quality:.1f}")

    # ---------- ROS ----------
    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
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
        _, _, dw, dh, _ = self._cam_disp_rect()
        with self.lock:
            write_idx = self._disp_write_idx
            disp = self._disp_buffers[write_idx]
        cv2.resize(img, (dw, dh), dst=disp, interpolation=cv2.INTER_LINEAR)
        with self.lock:
            self.frame = img
            self.disp_cache = disp
            self._disp_write_idx = 1 - write_idx
            self.camera_stamp = time.time()
            self.frame_pose, self.frame_pose_dt = self._nearest_pose_locked(stamp)
            self.frame_id += 1

    def _accum_add(self, g):
        if len(g) == 0:
            return
        with self.lock:
            self._accum_chunks.append(g)
            self._accum_total += len(g)
            while self._accum_total > ACCUM_MAX and self._accum_chunks:
                drop = self._accum_chunks.popleft()
                self._accum_total -= len(drop)

    def _accum_array(self, max_n=None):
        with self.lock:
            if not self._accum_chunks:
                return np.empty((0, 3), np.float32)
            if len(self._accum_chunks) == 1:
                pts = self._accum_chunks[0]
            else:
                pts = np.vstack(tuple(self._accum_chunks))
        if max_n and len(pts) > max_n:
            pts = pts[-max_n:]
        return pts

    def on_lidar(self, msg):
        msg_stamp = msg_stamp_sec(msg, fallback=time.time(), max_wall_skew=1.0)
        pts = parse_livox_msg(msg, max_pts=LIDAR_MAX, r_max=self.lidar_r_max)
        if len(pts) == 0:
            return
        with self.lock:
            self.latest_scan = pts
            self.lidar_stamp = time.time()
            pose, pose_dt = self._nearest_pose_locked(msg_stamp)
            self.latest_scan_pose = pose
            self.latest_scan_pose_dt = pose_dt
        if pose is None:
            return
        g = apply_T(self._T_world_lidar(pose), pts)
        self._accum_add(g)

    # ---------- 坐标映射 ----------
    def _cam_disp_rect(self):
        return view_rect(self.proc_w, self.proc_h, WIN_W, CAM_PANEL_H)

    def _disp_to_proc(self, dx, dy):
        x0, y0, dw, dh, scale = self._cam_disp_rect()
        if not (x0 <= dx < x0 + dw and y0 <= dy < y0 + dh):
            return None
        px = (dx - x0) / scale
        py = (dy - y0) / scale
        return int(px), int(py)

    def _proc_to_disp(self, px, py):
        x0, y0, dw, dh, scale = self._cam_disp_rect()
        return int(x0 + px * scale), int(y0 + py * scale)

    def _build_range_buttons(self):
        n = len(LIDAR_RANGE_PRESETS)
        total_w = n * RANGE_BTN_W + (n - 1) * RANGE_BTN_GAP
        x0 = WIN_W - total_w - 12
        buttons = []
        for i, (_, label) in enumerate(LIDAR_RANGE_PRESETS):
            bx = x0 + i * (RANGE_BTN_W + RANGE_BTN_GAP)
            buttons.append({
                "idx": i, "label": label,
                "x": bx, "y": RANGE_BTN_Y,
                "w": RANGE_BTN_W, "h": RANGE_BTN_H,
            })
        self.range_buttons = buttons

    def _set_lidar_range(self, idx):
        idx = int(idx) % len(LIDAR_RANGE_PRESETS)
        if idx == self.lidar_range_idx:
            return
        self.lidar_range_idx = idx
        self.lidar_r_max = LIDAR_RANGE_PRESETS[idx][0]
        with self.lock:
            self._accum_chunks = deque()
            self._accum_total = 0
            self.latest_scan = np.zeros((0, 3), np.float32)
        label = LIDAR_RANGE_PRESETS[idx][1]
        self.status = f"Lidar range {label}"
        self._show_toast(f"Lidar max range: {label}", 1.8)
        print(f"[INFO] lidar range -> {label} (r_max={self.lidar_r_max})")

    def _hit_range_button(self, x, y):
        button = hit_button(self.range_buttons, x, y)
        return None if button is None else button["idx"]

    def _draw_range_buttons(self, canvas):
        for b in self.range_buttons:
            x, y, w, h = b["x"], b["y"], b["w"], b["h"]
            on = b["idx"] == self.lidar_range_idx
            fill = (34, 95, 42) if on else UI_PANEL
            border = UI_GREEN if on else (105, 110, 115)
            cv2.rectangle(canvas, (x, y), (x + w, y + h), fill, -1)
            cv2.rectangle(canvas, (x, y), (x + w, y + h), border, 2 if on else 1)
            label = b["label"]
            fs = 0.55
            tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fs, 2)[0][0]
            tx = x + max(4, (w - tw) // 2)
            ty = y + (h + 16) // 2
            cv2.putText(canvas, label, (tx, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 255, 255), 2)

    # ---------- 按钮 ----------
    def _build_buttons(self):
        if self.roi_mode:
            specs = [
                {"id": "confirm", "label": "Save Region", "color": UI_GREEN},
                {"id": "cancel", "label": "Cancel", "color": (130, 135, 140)},
                {"id": "undo", "label": "Undo Point", "color": (40, 150, 220)},
                {"id": "quit", "label": "Finish", "color": UI_RED},
            ]
        else:
            specs = [
                {"id": "box", "label": "Box", "color": UI_GREEN},
                {"id": "poly", "label": "Polygon", "color": (40, 150, 220)},
                {"id": "line", "label": "Cable", "color": UI_YELLOW},
                {"id": "undo", "label": "Undo", "color": (220, 145, 45)},
                {"id": "quit", "label": "Finish", "color": UI_RED},
            ]
        margin, gap = 12, 14
        status_h = 26
        bh = BTN_BAR_H - margin * 2 - status_h
        by = CAM_PANEL_H + margin
        self.buttons = layout_buttons(
            specs, margin, by, WIN_W - margin * 2, bh, gap)

    def _hit_button(self, x, y):
        button = hit_button(self.buttons, x, y)
        return None if button is None else button["id"]

    def _show_toast(self, msg, sec=3.0):
        self.toast_msg = msg
        self.toast_until = time.time() + sec
        print(f"[TOAST] {msg}")

    def _draw_buttons(self, canvas):
        for b in self.buttons:
            draw_button(canvas, b, active=True, accent=b["color"],
                        destructive=b["id"] == "quit")

    # ---------- 业务 ----------
    def _clear_selection(self):
        self.roi_mode = False
        self.roi_drag = False
        self.roi_start = None
        self.roi_rect = None
        self.mark_mode = None
        self.mark_points = []

    def _start_selection(self, mode):
        self._clear_selection()
        self.roi_mode = True
        self.mark_mode = mode
        if mode == "rect":
            self.status = "Drag box on camera"
            self._show_toast("Drag box on camera image", 2.0)
        elif mode == "poly":
            self.status = "Tap polygon points, then Confirm"
            self._show_toast("Tap object outline points, then Confirm", 2.5)
        elif mode == "line":
            self.status = f"Tap cable line points, width {MARK_LINE_WIDTH_PX}px"
            self._show_toast("Tap along cable/device edge, then Confirm", 2.5)

    def undo(self):
        if not self.undo_stack:
            self.status = "Nothing to undo"
            return
        n = self.undo_stack.pop()
        self.danger = self.danger[:n]
        self.save_npz()
        self.status = f"Undone. danger points: {len(self.danger)}"

    def _selection_bbox(self, mode, rect, points):
        if mode == "rect":
            return rect
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
        if len(pts) == 0:
            return None
        pad = float(MARK_LINE_WIDTH_PX if mode == "line" else 4)
        x0, y0 = pts.min(axis=0) - pad
        x1, y1 = pts.max(axis=0) + pad
        x0 = max(0.0, min(float(self.proc_w - 1), x0))
        y0 = max(0.0, min(float(self.proc_h - 1), y0))
        x1 = max(0.0, min(float(self.proc_w), x1))
        y1 = max(0.0, min(float(self.proc_h), y1))
        return (float(x0), float(y0), float(x1 - x0), float(y1 - y0))

    def _selection_snapshot(self):
        mode = self.mark_mode or "rect"
        rect = self.roi_rect
        points = list(self.mark_points)
        if mode == "rect":
            if rect is None:
                return None, "Drag a box first"
            rx, ry, rw, rh = rect
            if rw < 8 or rh < 8:
                return None, "Drag a larger box first"
            bbox = (float(rx), float(ry), float(rw), float(rh))
        elif mode == "poly":
            if len(points) < 3:
                return None, "Tap at least 3 polygon points"
            bbox = self._selection_bbox(mode, rect, points)
        elif mode == "line":
            if len(points) < 2:
                return None, "Tap at least 2 line points"
            bbox = self._selection_bbox(mode, rect, points)
        else:
            return None, "Choose a mark mode first"
        if bbox is None or bbox[2] < 4 or bbox[3] < 4:
            return None, "Selection too small"
        return {
            "mode": mode,
            "rect": bbox,
            "points": np.asarray(points, dtype=np.float32).reshape(-1, 2),
        }, None

    def _selection_mask(self, pts_lidar, selection):
        """Points in lidar frame whose projection falls inside the mark shape."""
        if pts_lidar is None or len(pts_lidar) == 0:
            return None, 0
        pts_cam = apply_T(self.T_lidar_cam, pts_lidar)
        uv, Z = project_uv_pinhole(pts_cam, self.K, self.dist)
        mode = selection["mode"]
        rx, ry, rw, rh = selection["rect"]
        valid = ((Z > 0.05) & np.isfinite(uv).all(axis=1) &
                 (uv[:, 0] >= 0) & (uv[:, 0] < self.proc_w) &
                 (uv[:, 1] >= 0) & (uv[:, 1] < self.proc_h))
        if mode == "rect":
            inside = (valid &
                      (uv[:, 0] >= rx) & (uv[:, 0] <= rx + rw) &
                      (uv[:, 1] >= ry) & (uv[:, 1] <= ry + rh))
        else:
            mask = np.zeros((self.proc_h, self.proc_w), dtype=np.uint8)
            pts = np.round(selection["points"]).astype(np.int32).reshape(-1, 1, 2)
            if mode == "poly":
                cv2.fillPoly(mask, [pts], 1)
            elif mode == "line":
                cv2.polylines(mask, [pts], False, 1,
                              thickness=max(3, int(MARK_LINE_WIDTH_PX)),
                              lineType=cv2.LINE_AA)
            ui = np.round(uv[:, 0]).astype(np.int32)
            vi = np.round(uv[:, 1]).astype(np.int32)
            inside = np.zeros((len(pts_lidar),), dtype=bool)
            pix_ok = (valid &
                      (ui >= 0) & (ui < self.proc_w) &
                      (vi >= 0) & (vi < self.proc_h))
            if np.any(pix_ok):
                inside[pix_ok] = mask[vi[pix_ok], ui[pix_ok]] > 0
        n = int(inside.sum())
        if n == 0:
            return None, 0
        return inside, n

    def _pick_roi_points(self, pose, selection):
        """Return lidar-frame points whose projection falls inside the selection."""
        T_w_l = self._T_world_lidar(pose)
        T_l_w = inv_T(T_w_l)

        sources = []
        with self.lock:
            scan = self.latest_scan.copy()
            scan_pose = self.latest_scan_pose
            scan_pose_dt = self.latest_scan_pose_dt
        accum = self._accum_array(max_n=ROI_ACCUM_MAX)

        if len(scan) >= 5 and scan_pose is not None:
            sources.append(("scan", scan, scan_pose, scan_pose_dt))
        if len(accum) >= 5:
            src = accum
            if len(src) > ROI_ACCUM_SAMPLE:
                idx = np.linspace(0, len(src) - 1, ROI_ACCUM_SAMPLE, dtype=np.int32)
                src = src[idx]
            sources.append(("accum", apply_T(T_l_w, src), pose, 0.0))

        best_tag, best_pts, best_n, best_pose, best_pose_dt = None, None, -1, None, float("inf")
        cap = max(ROI_SAVE_MAX * 2, 8000)
        for tag, pts_lidar, src_pose, src_pose_dt in sources:
            inside, n = self._selection_mask(pts_lidar, selection)
            if inside is None or n <= best_n:
                continue
            idx = np.where(inside)[0]
            if len(idx) > cap:
                pick = np.linspace(0, len(idx) - 1, cap, dtype=np.int32)
                idx = idx[pick]
                n = len(idx)
            best_n = n
            best_tag = tag
            best_pts = pts_lidar[idx]
            best_pose = src_pose
            best_pose_dt = src_pose_dt
        if best_tag:
            print(f"[ROI] pick={best_tag} n={best_n} pose_dt={best_pose_dt:.3f}s")
        return best_tag, best_pts, best_n, best_pose, best_pose_dt

    def _merge_danger_points(self, chosen_g):
        """Voxel + cap; prevent OOM when far-range ROI hits many lidar returns."""
        if chosen_g is None or len(chosen_g) == 0:
            return 0
        chosen_g = voxel_downsample(chosen_g, voxel=ROI_VOXEL, max_pts=ROI_SAVE_MAX)
        if len(chosen_g) == 0:
            return 0
        if not self.danger:
            self.danger = chosen_g.tolist()
            return len(chosen_g)
        exist = np.asarray(self.danger, dtype=np.float32)
        merged = voxel_downsample(
            np.vstack([exist, chosen_g]), voxel=ROI_VOXEL, max_pts=DANGER_TOTAL_MAX)
        add = len(merged) - len(exist)
        if add < 0:
            add = 0
        self.danger = merged.tolist()
        return add

    def confirm_roi(self):
        selection, err = self._selection_snapshot()
        if selection is None:
            self.status = err
            self._show_toast(err)
            return

        try:
            with self.lock:
                pose = self.pose
                scan_n = len(self.latest_scan)
                frame = None if self.frame is None else self.frame.copy()
            accum_n = self._accum_total

            if pose is None:
                self.status = "NO SLAM POSE - wait for mapping"
                self._show_toast("NO SLAM POSE! Start Fastlivo2 and move device")
                print("[CONFIRM] blocked: /aft_mapped_to_init has no data")
                return

            if scan_n < 5 and accum_n < 5:
                self.status = "No lidar points - check radar network"
                self._show_toast("No lidar data. Check eth0 / Livox IP")
                print(f"[CONFIRM] blocked: scan={scan_n} accum={accum_n}")
                return

            src_tag, chosen, in_roi, save_pose, save_pose_dt = self._pick_roi_points(pose, selection)
            if chosen is None:
                chosen = np.empty((0, 3), np.float32)
            if save_pose is None:
                save_pose = pose
            if save_pose_dt > POSE_SYNC_MAX_DT:
                self.status = "Pose/lidar not synced - wait and confirm again"
                self._show_toast("Pose sync weak. Hold still and confirm again", 3.5)
                print(f"[CONFIRM] blocked: src={src_tag} pose_dt={save_pose_dt:.3f}s")
                return

            if len(chosen) > 0:
                r = np.linalg.norm(chosen, axis=1)
                chosen = chosen[r >= 0.2]
                pts_cam = apply_T(self.T_lidar_cam, chosen)
                cluster = front_depth_cluster(
                    pts_cam, z_gap=CLUSTER_Z_GAP, min_pts=5)
                if cluster is not None and len(cluster) < len(pts_cam):
                    z0, z1 = cluster[:, 2].min(), cluster[:, 2].max()
                    keep = (pts_cam[:, 2] >= z0 - 0.02) & (pts_cam[:, 2] <= z1 + 0.02)
                    chosen = chosen[keep]

            mode = selection["mode"]
            print(f"[CONFIRM] mode={mode} src={src_tag} in_shape={in_roi} kept={len(chosen)} "
                  f"bbox={tuple(round(v, 1) for v in selection['rect'])} "
                  f"range={LIDAR_RANGE_PRESETS[self.lidar_range_idx][1]}")

            if len(chosen) < 5:
                self.status = f"Too few points ({len(chosen)}), redraw"
                self._show_toast(f"Only {len(chosen)} pts in shape - aim at lidar hits")
                return

            T_w_l = self._T_world_lidar(save_pose)
            chosen_g = apply_T(T_w_l, chosen)
            raw_g_n = len(chosen_g)
            if ROI_DENSIFY:
                chosen_g = densify_planar_points(
                    chosen_g,
                    step=ROI_DENSIFY_STEP,
                    max_pts=ROI_DENSIFY_MAX,
                    max_thickness=ROI_DENSIFY_MAX_THICKNESS,
                )
                if len(chosen_g) > raw_g_n:
                    print(f"[CONFIRM] densify {raw_g_n}->{len(chosen_g)}")
            self.undo_stack.append(len(self.danger))
            add = self._merge_danger_points(chosen_g)
            self._set_anchor_from_frame(frame, selection["rect"])
            self.save_npz()
            self._clear_selection()
            self.status = f"Saved +{add}, total {len(self.danger)}"
            self._show_toast(f"OK: saved +{add} (total {len(self.danger)})", 2.5)
        except MemoryError:
            self.status = "Too many points - use smaller ROI or 10m range"
            self._show_toast("Out of memory: shrink ROI or set range 10m", 4.0)
            print("[CONFIRM] MemoryError")
        except Exception as e:
            self.status = f"Save failed: {e}"
            self._show_toast(f"Confirm failed: {e}", 4.0)
            print(f"[CONFIRM] error: {e}")
            import traceback
            traceback.print_exc()

    def _process_action(self, bid):
        if bid == "quit":
            self.quit = True
        elif bid == "undo":
            if self.roi_mode and self.mark_mode in ("poly", "line") and self.mark_points:
                self.mark_points.pop()
                self.status = f"{self.mark_mode}: {len(self.mark_points)} points"
                self._show_toast("Last point removed", 1.2)
            elif self.roi_mode and self.mark_mode == "rect":
                self.roi_rect = None
                self.status = "Box cleared"
                self._show_toast("Box cleared", 1.2)
            else:
                self.undo()
                self._show_toast("Undo done", 1.5)
        elif bid in ("box", "roi"):
            self._start_selection("rect")
        elif bid == "poly":
            self._start_selection("poly")
        elif bid == "line":
            self._start_selection("line")
        elif bid == "confirm":
            self.confirm_roi()
        elif bid == "cancel":
            self._clear_selection()
            self.status = "Selection cancelled"
            self._show_toast("Selection cancelled", 1.5)

    def on_mouse(self, event, x, y, flags, param):
        if event in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_LBUTTONUP):
            ridx = self._hit_range_button(x, y)
            if ridx is not None:
                if event == cv2.EVENT_LBUTTONUP:
                    self._set_lidar_range(ridx)
                return
            bid = self._hit_button(x, y)
            if bid:
                if event == cv2.EVENT_LBUTTONUP:
                    self.pending_action = bid
                return
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.roi_mode and y < CAM_PANEL_H:
                p = self._disp_to_proc(x, y)
                if p and self.mark_mode == "rect":
                    self.roi_drag = True
                    self.roi_start = p
                    self.roi_rect = (p[0], p[1], 0, 0)
                elif p and self.mark_mode in ("poly", "line"):
                    self.mark_points.append(p)
                    self.status = f"{self.mark_mode}: {len(self.mark_points)} points"
        elif event == cv2.EVENT_MOUSEMOVE and self.roi_drag and self.roi_mode:
            p = self._disp_to_proc(x, y)
            if p and self.roi_start and self.mark_mode == "rect":
                x0, y0 = self.roi_start
                self.roi_rect = (min(x0, p[0]), min(y0, p[1]),
                                 abs(p[0] - x0), abs(p[1] - y0))
        elif event == cv2.EVENT_LBUTTONUP and self.roi_drag:
            self.roi_drag = False

    def _draw_danger_on_cam(self, canvas, danger, pose):
        if pose is None or len(danger) == 0:
            return 0
        dpts = np.asarray(danger, np.float32)
        if len(dpts) > DANGER_DRAW:
            dpts = dpts[np.linspace(0, len(dpts) - 1, DANGER_DRAW, dtype=np.int32)]
        T_w_l = self._T_world_lidar(pose)
        pts_c = apply_T(self.T_lidar_cam, apply_T(inv_T(T_w_l), dpts))
        uv, Z = project_uv_pinhole(pts_c, self.K, self.dist)
        vis = Z > 0.05
        if not np.any(vis):
            return 0
        x0, y0, _, _, scale = self._cam_disp_rect()
        dx = (x0 + uv[vis, 0] * scale).astype(np.int32)
        dy = (y0 + uv[vis, 1] * scale).astype(np.int32)
        return stamp_bgr(canvas, dx, dy)

    def _draw_selection(self, canvas):
        if not self.roi_mode:
            return
        x0, y0, _, _, scale = self._cam_disp_rect()
        col = (0, 255, 0)
        if self.mark_mode == "rect" and self.roi_rect is not None:
            rx, ry, rw, rh = self.roi_rect
            d_x0, d_y0 = self._proc_to_disp(rx, ry)
            d_x1, d_y1 = self._proc_to_disp(rx + rw, ry + rh)
            cv2.rectangle(canvas, (d_x0, d_y0), (d_x1, d_y1), col, 2)
            return
        if self.mark_mode not in ("poly", "line") or not self.mark_points:
            return
        pts = []
        for px, py in self.mark_points:
            pts.append([int(x0 + px * scale), int(y0 + py * scale)])
        pts = np.asarray(pts, dtype=np.int32).reshape(-1, 1, 2)
        for p in pts.reshape(-1, 2):
            cv2.circle(canvas, tuple(p), 5, col, -1)
        if len(pts) >= 2:
            if self.mark_mode == "poly":
                cv2.polylines(canvas, [pts], len(pts) >= 3, col, 2, cv2.LINE_AA)
            else:
                thick = max(3, int(MARK_LINE_WIDTH_PX * scale))
                cv2.polylines(canvas, [pts], False, (0, 220, 255), thick, cv2.LINE_AA)
                cv2.polylines(canvas, [pts], False, col, 2, cv2.LINE_AA)

    def _render(self):
        if self._canvas is None:
            self._canvas = np.full((WIN_H, WIN_W, 3), UI_BG, np.uint8)
        canvas = self._canvas
        cv2.rectangle(canvas, (0, 0), (WIN_W - 1, WIN_H - 1), UI_BG, -1)
        self._build_buttons()
        self._build_range_buttons()

        with self.lock:
            frame = self.frame
            disp = self.disp_cache
            camera_stamp = self.camera_stamp
            pose = self.pose
            pose_stamp = self.pose_stamp
            frame_pose = self.frame_pose
            frame_pose_dt = self.frame_pose_dt
            scan_n = len(self.latest_scan)
            lidar_stamp = self.lidar_stamp
            danger = self.danger
            accum_n = self._accum_total
        now = time.time()
        overlay_pose = frame_pose if frame_pose is not None else pose
        cam_ok = (frame is not None and disp is not None and camera_stamp > 0 and
                  now - camera_stamp <= SENSOR_STALE_SEC)
        lidar_ok = (scan_n > 0 and lidar_stamp > 0 and
                    now - lidar_stamp <= SENSOR_STALE_SEC)
        sync_bad = (frame_pose_dt < float("inf") and frame_pose_dt > POSE_SYNC_MAX_DT)
        pose_ok = (overlay_pose is not None and pose_stamp > 0 and
                   now - pose_stamp <= SENSOR_STALE_SEC and not sync_bad)
        n_red = 0

        x0, y0, dw, dh, _ = self._cam_disp_rect()

        if not cam_ok:
            cv2.rectangle(canvas, (0, 0), (WIN_W - 1, CAM_PANEL_H - 1), UI_BG, -1)
        else:
            with self.lock:
                cv2.copyTo(self.disp_cache, None, canvas[y0:y0 + dh, x0:x0 + dw])
            if pose_ok:
                n_red = self._draw_danger_on_cam(canvas, danger, overlay_pose)
            self._draw_selection(canvas)

        self._draw_range_buttons(canvas)
        range_lbl = LIDAR_RANGE_PRESETS[self.lidar_range_idx][1]
        range_x = self.range_buttons[0]["x"] if self.range_buttons else WIN_W
        chips = [
            ("CAM", "ok" if cam_ok else "error", "LIVE" if cam_ok else "OFF"),
            ("LIDAR", "ok" if lidar_ok else "error", str(scan_n) if lidar_ok else "OFF"),
            ("POSE", "ok" if pose_ok else "error", "LIVE" if pose_ok else "OFF"),
            ("REGION", "ok" if danger else "idle", str(len(danger))),
        ]
        draw_status_strip(canvas, chips, x=10, y=10, max_x=range_x - 8)

        problems = []
        if not cam_ok:
            problems.append("CAMERA OFF")
        if not lidar_ok:
            problems.append("LIDAR OFF")
        if not pose_ok:
            problems.append("POSE NOT READY" if not sync_bad else "POSE NOT SYNCED")
        if problems:
            draw_banner(canvas, "MARKING PAUSED  |  " + "  /  ".join(problems),
                        level="error", y=52)

        cv2.rectangle(canvas, (0, CAM_PANEL_H), (WIN_W - 1, WIN_H - 1), UI_PANEL, -1)
        self._draw_buttons(canvas)
        mode = (self.mark_mode.upper() if self.roi_mode and self.mark_mode else "READY")
        summary = (f"{mode}  |  {self.status}  |  range {range_lbl}  |  "
                   f"scan {scan_n}  |  map {accum_n}  |  visible {n_red}")
        put_text_fit(canvas, summary, 12, WIN_H - 7, WIN_W - 24,
                     color=(210, 214, 218), base=0.46, minimum=0.34, thickness=1)

        if self.toast_msg and time.time() < self.toast_until:
            level = "ok" if self.toast_msg.startswith(("OK", "Saved")) else "warn"
            draw_banner(canvas, self.toast_msg, level=level, y=104)
        return canvas

    def run(self):
        rospy.init_node("danger_mark_fastlivo", anonymous=True)
        if not wait_ros_topics(["/left_camera/image", "/livox/lidar"]):
            print("[ERR] start Lite-SLAM-Base or Demo-Base first")
            return 1

        rospy.Subscriber("/aft_mapped_to_init", Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber("/left_camera/image", Image, self.on_image, queue_size=1)
        rospy.Subscriber("/livox/lidar", CustomMsg, self.on_lidar, queue_size=1)

        threading.Thread(target=rospy.spin, daemon=True).start()

        win = "Danger Region Marking"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, WIN_W, WIN_H)
        cv2.setMouseCallback(win, self.on_mouse)
        self.status = "Choose Box, Polygon, or Cable"

        print("[INFO] touch UI: use bottom buttons")

        try:
            while not rospy.is_shutdown() and not self.quit:
                if self.pending_action:
                    act = self.pending_action
                    self.pending_action = None
                    self._process_action(act)
                canvas = self._render()
                cv2.imshow(win, canvas)
                if cv2.waitKey(RENDER_MS) & 0xFF == 27:
                    break
        finally:
            try:
                if self.danger:
                    self.save_npz()
            except Exception as e:
                print(f"[WARN] exit save failed: {e}")
            cv2.destroyAllWindows()
        return 0


def main():
    from demo_log import setup_demo_log, log_exception
    log_path = setup_demo_log("stage1")
    print(f"[INFO] log file: {log_path}")
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", default=NPZ_DEFAULT)
    args = ap.parse_args()
    try:
        return DangerMarkApp(args.npz).run()
    except Exception:
        log_exception("STAGE1")
        return 1


if __name__ == "__main__":
    sys.exit(main())
