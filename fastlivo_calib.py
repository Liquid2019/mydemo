#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FAST-LIVO2 标定读取 + 坐标变换 + ROS 图像/点云工具"""

import os
import time
import numpy as np

try:
    from scipy.spatial import cKDTree
except Exception:
    cKDTree = None

DEFAULT_CAM_YAML = os.path.expanduser(
    "~/fastlivo2_ws/src/FAST-LIVO2/config/camera_pinhole.yaml"
)
DEFAULT_AVIA_YAML = os.path.expanduser(
    "~/fastlivo2_ws/src/FAST-LIVO2/config/avia.yaml"
)


def _parse_yaml_floats(lines, key):
    vals = []
    started = False
    for line in lines:
        s = line.strip()
        if not started:
            if not s.startswith(key):
                continue
            part = s.split(":", 1)[1].strip()
            if part.startswith("["):
                part = part.strip("[]")
            chunk = [float(x.strip().rstrip(",")) for x in part.split(",") if x.strip()]
            vals.extend(chunk)
            started = True
            if len(chunk) >= 3 and not s.endswith(","):
                break
            continue
        if not s or s.startswith("#"):
            continue
        if ":" in s and not s[0].isdigit() and not s.startswith("-"):
            break
        chunk = [float(x.strip().rstrip(",")) for x in s.replace("[", "").replace("]", "").split(",") if x.strip()]
        vals.extend(chunk)
        if len(vals) >= 9:
            break
    return vals if vals else None


def load_proc_image_size(scale, cam_yaml=DEFAULT_CAM_YAML):
    full_w, full_h = 2448, 2048
    with open(cam_yaml, "r", encoding="utf-8") as f:
        for line in f:
            if line.strip().startswith("cam_width:"):
                full_w = int(line.split(":")[1].strip())
            if line.strip().startswith("cam_height:"):
                full_h = int(line.split(":")[1].strip())
    return max(1, int(full_w * scale)), max(1, int(full_h * scale))


def load_pinhole_calib(cam_yaml=DEFAULT_CAM_YAML, avia_yaml=DEFAULT_AVIA_YAML):
    with open(cam_yaml, "r", encoding="utf-8") as f:
        cam_lines = f.readlines()

    scale = float(_parse_yaml_floats(cam_lines, "scale")[0])
    fx = float(_parse_yaml_floats(cam_lines, "cam_fx")[0]) * scale
    fy = float(_parse_yaml_floats(cam_lines, "cam_fy")[0]) * scale
    cx = float(_parse_yaml_floats(cam_lines, "cam_cx")[0]) * scale
    cy = float(_parse_yaml_floats(cam_lines, "cam_cy")[0]) * scale
    d0 = float(_parse_yaml_floats(cam_lines, "cam_d0")[0])
    d1 = float(_parse_yaml_floats(cam_lines, "cam_d1")[0])
    d2 = float(_parse_yaml_floats(cam_lines, "cam_d2")[0])
    d3 = float(_parse_yaml_floats(cam_lines, "cam_d3")[0])

    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    dist = np.array([[d0, d1, d2, d3]], dtype=np.float32)

    with open(avia_yaml, "r", encoding="utf-8") as f:
        avia_lines = f.readlines()

    ext_t = np.array(_parse_yaml_floats(avia_lines, "extrinsic_T"), dtype=np.float32).reshape(3)
    ext_r = np.array(_parse_yaml_floats(avia_lines, "extrinsic_R"), dtype=np.float32).reshape(3, 3)
    Rcl = np.array(_parse_yaml_floats(avia_lines, "Rcl"), dtype=np.float32).reshape(3, 3)
    Pcl = np.array(_parse_yaml_floats(avia_lines, "Pcl"), dtype=np.float32).reshape(3)

    Rli = ext_r.T.astype(np.float32)
    Pli = (-ext_r.T @ ext_t).astype(np.float32)
    T_imu_lidar = make_T(Rli, Pli)

    R_lc = (Rcl @ Rli).astype(np.float32)
    t_lc = (Rcl @ Pli + Pcl).astype(np.float32)
    T_lidar_cam = make_T(R_lc, t_lc)

    return K, dist, T_lidar_cam, T_imu_lidar, scale


def T_world_from_imu_pose(pos, quat, T_imu_lidar):
    return make_T(quat_to_R(quat), pos) @ T_imu_lidar


def quat_slerp(q0, q1, a):
    q0 = np.asarray(q0, dtype=np.float32).reshape(4)
    q1 = np.asarray(q1, dtype=np.float32).reshape(4)
    q0 = q0 / max(float(np.linalg.norm(q0)), 1e-9)
    q1 = q1 / max(float(np.linalg.norm(q1)), 1e-9)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    a = float(np.clip(a, 0.0, 1.0))
    if dot > 0.9995:
        q = q0 + a * (q1 - q0)
        return (q / max(float(np.linalg.norm(q)), 1e-9)).astype(np.float32)
    theta0 = np.arccos(np.clip(dot, -1.0, 1.0))
    theta = theta0 * a
    sin_t = np.sin(theta)
    sin_t0 = max(float(np.sin(theta0)), 1e-9)
    s0 = np.cos(theta) - dot * sin_t / sin_t0
    s1 = sin_t / sin_t0
    return (s0 * q0 + s1 * q1).astype(np.float32)


def interp_pose(pose0, pose1, a):
    p0, q0 = pose0
    p1, q1 = pose1
    a = float(np.clip(a, 0.0, 1.0))
    p = (np.asarray(p0, dtype=np.float32) * (1.0 - a) +
         np.asarray(p1, dtype=np.float32) * a)
    q = quat_slerp(q0, q1, a)
    return p.astype(np.float32), q.astype(np.float32)


def msg_stamp_sec(msg, fallback=None, max_wall_skew=None):
    fb = time.time() if fallback is None else float(fallback)
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        try:
            t = float(stamp.to_sec())
            if t > 0:
                if max_wall_skew is not None and abs(t - fb) > float(max_wall_skew):
                    return fb
                return t
        except Exception:
            pass
    return fb


def densify_planar_points(pts, step=0.03, max_pts=12000, max_thickness=0.10):
    pts = np.ascontiguousarray(np.asarray(pts, dtype=np.float32))
    if len(pts) < 20 or step <= 0 or max_pts <= 0:
        return pts
    center = pts.mean(axis=0)
    X = pts - center
    try:
        _, s, vt = np.linalg.svd(X.astype(np.float64), full_matrices=False)
    except Exception:
        return pts
    if len(s) < 3:
        return pts
    spread = s / max(np.sqrt(max(len(pts) - 1, 1)), 1.0)
    if spread[1] < 1e-3 or spread[2] > max(float(max_thickness), 0.35 * spread[1]):
        return pts

    axes = vt.astype(np.float32)
    uv = X @ axes[:2].T
    z = np.median(X @ axes[2])
    u0, v0 = uv.min(axis=0)
    u1, v1 = uv.max(axis=0)
    area = max((u1 - u0) * (v1 - v0), 0.0)
    use_step = float(step)
    if area / max(use_step * use_step, 1e-6) > max_pts * 2:
        use_step = float(np.sqrt(area / max(max_pts * 2, 1)))

    us = np.arange(u0, u1 + use_step, use_step, dtype=np.float32)
    vs = np.arange(v0, v1 + use_step, use_step, dtype=np.float32)
    if len(us) == 0 or len(vs) == 0 or len(us) * len(vs) < len(pts):
        return pts
    grid = np.stack(np.meshgrid(us, vs), axis=-1).reshape(-1, 2)

    try:
        from scipy.spatial import Delaunay
        tri = Delaunay(uv.astype(np.float64))
        inside = tri.find_simplex(grid.astype(np.float64)) >= 0
        grid = grid[inside]
    except Exception:
        pass
    if len(grid) == 0:
        return pts
    if len(grid) > max_pts:
        pick = np.linspace(0, len(grid) - 1, max_pts, dtype=np.int32)
        grid = grid[pick]

    dense = (center[None, :] + grid[:, 0:1] * axes[0][None, :] +
             grid[:, 1:2] * axes[1][None, :] + float(z) * axes[2][None, :])
    return np.vstack([pts, dense.astype(np.float32)])


def quat_to_R(q):
    qx, qy, qz, qw = [float(x) for x in q]
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n > 0:
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float32)


def make_T(R, t):
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = np.asarray(R, dtype=np.float32)
    T[:3, 3] = np.asarray(t, dtype=np.float32).reshape(3)
    return T


def inv_T(T):
    R = T[:3, :3]
    t = T[:3, 3:4]
    Ti = np.eye(4, dtype=np.float32)
    Ti[:3, :3] = R.T
    Ti[:3, 3:4] = -R.T @ t
    return Ti


def apply_T(T, pts):
    if pts is None or len(pts) == 0:
        return np.empty((0, 3), dtype=np.float32)
    P = np.asarray(pts, dtype=np.float32)
    ones = np.ones((len(P), 1), dtype=np.float32)
    return (T @ np.hstack([P, ones]).T).T[:, :3].astype(np.float32)


def project_uv_pinhole(pts_cam, K, dist):
    import cv2
    if pts_cam is None or len(pts_cam) == 0:
        return np.empty((0, 2), np.float32), np.empty((0,), np.float32)
    pts = np.asarray(pts_cam, dtype=np.float32)
    Z = pts[:, 2].copy()
    obj = pts.reshape(-1, 1, 3)
    rvec = np.zeros((3, 1), np.float32)
    tvec = np.zeros((3, 1), np.float32)
    uv, _ = cv2.projectPoints(obj, rvec, tvec, K.astype(np.float32), dist.astype(np.float32))
    return uv.reshape(-1, 2).astype(np.float32), Z


def min_dist_pts_to_danger(pts_cam, danger_cam):
    if pts_cam is None or len(pts_cam) == 0 or danger_cam is None or len(danger_cam) == 0:
        return float("inf"), -1
    pts = np.asarray(pts_cam, dtype=np.float32)
    danger = np.asarray(danger_cam, dtype=np.float32)
    ok_p = np.isfinite(pts).all(axis=1)
    ok_d = np.isfinite(danger).all(axis=1)
    pts = pts[ok_p]
    danger_idx = np.flatnonzero(ok_d)
    danger = danger[ok_d]
    if len(pts) == 0 or len(danger) == 0:
        return float("inf"), -1

    if cKDTree is not None:
        dist, local = cKDTree(danger).query(pts, k=1)
        pick = int(np.argmin(dist))
        return float(dist[pick]), int(danger_idx[int(local[pick])])

    best_d2 = float("inf")
    best_danger_idx = -1
    chunk = 512
    for start in range(0, len(pts), chunk):
        block = pts[start:start + chunk]
        diff = block[:, None, :] - danger[None, :, :]
        d2 = np.einsum("ijk,ijk->ij", diff, diff)
        flat_idx = int(np.argmin(d2))
        val = float(d2.flat[flat_idx])
        if val < best_d2:
            best_d2 = val
            best_danger_idx = int(danger_idx[flat_idx % len(danger)])
    return float(np.sqrt(best_d2)), best_danger_idx


def robust_dist_pts_to_danger(pts_cam, danger_cam, percentile=10.0):
    """Distance from object points to danger region using a low percentile.

    A plain minimum can be dominated by one bad lidar point. This keeps the
    result conservative while requiring support from several nearby points.
    """
    if pts_cam is None or len(pts_cam) == 0 or danger_cam is None or len(danger_cam) == 0:
        return float("inf"), -1
    pts = np.asarray(pts_cam, dtype=np.float32)
    danger = np.asarray(danger_cam, dtype=np.float32)
    ok_p = np.isfinite(pts).all(axis=1)
    ok_d = np.isfinite(danger).all(axis=1)
    pts = pts[ok_p]
    danger_idx = np.flatnonzero(ok_d)
    danger = danger[ok_d]
    if len(pts) == 0 or len(danger) == 0:
        return float("inf"), -1

    if cKDTree is not None:
        dist, local = cKDTree(danger).query(pts, k=1)
        pct = float(np.clip(percentile, 0.0, 50.0))
        rank = int(np.floor((pct / 100.0) * max(len(dist) - 1, 0)))
        order = np.argpartition(dist, rank)
        pick = int(order[rank])
        return float(dist[pick]), int(danger_idx[int(local[pick])])

    nearest_d2 = np.empty((len(pts),), dtype=np.float32)
    nearest_didx = np.empty((len(pts),), dtype=np.int32)
    chunk = 512
    for start in range(0, len(pts), chunk):
        block = pts[start:start + chunk]
        diff = block[:, None, :] - danger[None, :, :]
        d2 = np.einsum("ijk,ijk->ij", diff, diff)
        local = np.argmin(d2, axis=1)
        nearest_d2[start:start + len(block)] = d2[np.arange(len(block)), local]
        nearest_didx[start:start + len(block)] = danger_idx[local]

    pct = float(np.clip(percentile, 0.0, 50.0))
    rank = int(np.floor((pct / 100.0) * max(len(nearest_d2) - 1, 0)))
    order = np.argpartition(nearest_d2, rank)
    pick = int(order[rank])
    return float(np.sqrt(nearest_d2[pick])), int(nearest_didx[pick])


def front_depth_cluster(pts_cam, z_gap=0.15, min_pts=5):
    if pts_cam is None or len(pts_cam) < min_pts:
        return pts_cam
    z = pts_cam[:, 2]
    order = np.argsort(z)
    pts_sorted = pts_cam[order]
    z_sorted = z[order]
    start = 0
    for i in range(1, len(z_sorted)):
        if z_sorted[i] - z_sorted[i - 1] > z_gap:
            cluster = pts_sorted[start:i]
            if len(cluster) >= min_pts:
                return cluster
            start = i
    tail = pts_sorted[start:]
    return tail if len(tail) >= min_pts else pts_cam


def voxel_downsample(pts, voxel=0.02, max_pts=8000):
    """Grid dedup; cap output count to avoid OOM on Jetson."""
    pts = np.ascontiguousarray(np.asarray(pts, dtype=np.float32))
    if len(pts) == 0:
        return pts
    idx = np.ascontiguousarray(np.floor(pts / float(voxel)).astype(np.int32))
    keys = idx.view(np.dtype((np.void, idx.dtype.itemsize * idx.shape[1])))
    _, ui = np.unique(keys, return_index=True)
    out = pts[ui]
    if len(out) > max_pts:
        pick = np.linspace(0, len(out) - 1, max_pts, dtype=np.int32)
        out = out[pick]
    return np.ascontiguousarray(out, dtype=np.float32)


def parse_livox_msg(msg, max_pts=15000, r_min=0.10, r_max=10.0):
    n = min(len(msg.points), max_pts)
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)
    pts = np.empty((n, 3), dtype=np.float32)
    for i, p in enumerate(msg.points[:n]):
        pts[i, 0] = p.x
        pts[i, 1] = p.y
        pts[i, 2] = p.z
    r2 = np.sum(pts * pts, axis=1)
    return pts[(r2 >= r_min * r_min) & (r2 <= r_max * r_max)]


def stamp_bgr(canvas, xs, ys, color=(0, 0, 255), radius=0):
    if xs is None or len(xs) == 0:
        return 0
    h, w = canvas.shape[:2]
    ok = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
    if not np.any(ok):
        return 0
    xs0 = xs[ok].astype(np.int32)
    ys0 = ys[ok].astype(np.int32)
    r = max(0, int(radius))
    if r <= 0:
        canvas[ys0, xs0] = color
    else:
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                xx = xs0 + dx
                yy = ys0 + dy
                inside = (xx >= 0) & (xx < w) & (yy >= 0) & (yy < h)
                if np.any(inside):
                    canvas[yy[inside], xx[inside]] = color
    return int(ok.sum())


def ros_image_to_bgr(msg):
    import cv2
    if msg is None:
        return None
    h, w = msg.height, msg.width
    enc = (msg.encoding or "").lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if enc == "bgr8":
        return data.reshape(h, w, 3).copy()
    if enc == "rgb8":
        return cv2.cvtColor(data.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    if enc in ("mono8", "8uc1"):
        return cv2.cvtColor(data.reshape(h, w), cv2.COLOR_GRAY2BGR)
    if enc == "bgra8":
        return cv2.cvtColor(data.reshape(h, w, 4), cv2.COLOR_BGRA2BGR)
    return None


def view_rect(proc_w, proc_h, panel_w, panel_h):
    scale = min(panel_h / proc_h, panel_w / proc_w)
    dw = int(proc_w * scale)
    dh = int(proc_h * scale)
    x0 = (panel_w - dw) // 2
    y0 = (panel_h - dh) // 2
    return x0, y0, dw, dh, scale


def wait_ros_topics(required, timeout=30):
    import rospy
    t0 = time.time()
    while time.time() - t0 < timeout:
        topics = dict(rospy.get_published_topics())
        if all(t in topics for t in required):
            return True
        time.sleep(1)
    return False


def draw_bar_button(canvas, x, y, w, h, label, on, on_color, off_color=(70, 70, 70)):
    import cv2
    color = on_color if on else off_color
    cv2.rectangle(canvas, (x, y), (x + w, y + h), color, -1)
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (230, 230, 230), 3)
    text = f"{label} ON" if on else label
    fs = 0.75
    tw = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fs, 2)[0][0]
    tx = x + max(4, (w - tw) // 2)
    ty = y + (h + 18) // 2
    cv2.putText(canvas, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 255, 255), 2)
