#!/usr/bin/env python3
"""Small OpenCV UI primitives shared by the marking and distance apps."""

import cv2
import numpy as np


FONT = cv2.FONT_HERSHEY_SIMPLEX

BG = (28, 30, 32)
PANEL = (42, 45, 48)
PANEL_LIGHT = (58, 62, 66)
TEXT = (245, 245, 245)
TEXT_MUTED = (188, 194, 200)

GREEN = (80, 205, 90)
YELLOW = (40, 205, 245)
RED = (55, 70, 235)
BLUE = (220, 150, 55)
GRAY = (135, 140, 145)

STATE_COLORS = {
    "ok": GREEN,
    "warn": YELLOW,
    "error": RED,
    "idle": GRAY,
}


def _clip_rect(canvas, x, y, w, h):
    ch, cw = canvas.shape[:2]
    x1 = max(0, min(cw, int(x)))
    y1 = max(0, min(ch, int(y)))
    x2 = max(x1, min(cw, int(x + w)))
    y2 = max(y1, min(ch, int(y + h)))
    return x1, y1, x2, y2


def blend_rect(canvas, x, y, w, h, color=PANEL, alpha=0.88):
    x1, y1, x2, y2 = _clip_rect(canvas, x, y, w, h)
    if x2 <= x1 or y2 <= y1:
        return
    roi = canvas[y1:y2, x1:x2]
    if alpha >= 0.85:
        cv2.rectangle(canvas, (x1, y1), (x2 - 1, y2 - 1), color, -1)
        return
    overlay = np.empty_like(roi)
    cv2.rectangle(overlay, (0, 0), (overlay.shape[1] - 1, overlay.shape[0] - 1),
                  color, -1)
    cv2.addWeighted(
        overlay, float(alpha), roi, 1.0 - float(alpha), 0, dst=roi)


def fitted_text_scale(text, max_width, base=0.72, minimum=0.38, thickness=2):
    scale = float(base)
    while scale > minimum:
        width = cv2.getTextSize(str(text), FONT, scale, thickness)[0][0]
        if width <= max_width:
            break
        scale -= 0.04
    return max(float(minimum), scale)


def put_text_fit(canvas, text, x, baseline_y, max_width, color=TEXT,
                 base=0.72, minimum=0.38, thickness=2):
    scale = fitted_text_scale(text, max_width, base, minimum, thickness)
    cv2.putText(canvas, str(text), (int(x), int(baseline_y)), FONT, scale,
                color, thickness, cv2.LINE_AA)
    return scale


def layout_buttons(specs, x, y, width, height, gap=10):
    """Attach stable geometry to a list of button dictionaries."""
    if not specs:
        return []
    count = len(specs)
    usable = max(count, int(width) - int(gap) * (count - 1))
    base_w, extra = divmod(usable, count)
    out = []
    bx = int(x)
    for i, spec in enumerate(specs):
        bw = base_w + (1 if i < extra else 0)
        button = dict(spec)
        button.update({"x": bx, "y": int(y), "w": bw, "h": int(height)})
        out.append(button)
        bx += bw + int(gap)
    return out


def hit_button(buttons, x, y):
    """Use exact bounds so neighboring touch targets never overlap."""
    for button in buttons:
        bx, by = button["x"], button["y"]
        if bx <= x <= bx + button["w"] and by <= y <= by + button["h"]:
            return button
    return None


def _dark(color, amount=0.48):
    return tuple(int(max(0, min(255, c * amount))) for c in color)


def draw_button(canvas, button, active=True, accent=GREEN, state_text=None,
                destructive=False):
    x, y, w, h = (button[k] for k in ("x", "y", "w", "h"))
    accent = RED if destructive else tuple(accent)
    fill = _dark(accent, 0.48) if active else PANEL
    border = accent if active or destructive else (100, 105, 110)
    cv2.rectangle(canvas, (x, y), (x + w, y + h), fill, -1)
    cv2.rectangle(canvas, (x, y), (x + w, y + h), border, 2)

    label = button.get("label", "")
    if state_text is None:
        scale = fitted_text_scale(label, w - 24, base=0.78, minimum=0.46, thickness=2)
        size = cv2.getTextSize(label, FONT, scale, 2)[0]
        tx = x + max(10, (w - size[0]) // 2)
        ty = y + (h + size[1]) // 2
        cv2.putText(canvas, label, (tx, ty), FONT, scale, TEXT, 2, cv2.LINE_AA)
        return

    dot_color = accent if active else GRAY
    cv2.circle(canvas, (x + 18, y + 18), 6, dot_color, -1, cv2.LINE_AA)
    scale = fitted_text_scale(label, w - 28, base=0.68, minimum=0.42, thickness=2)
    size = cv2.getTextSize(label, FONT, scale, 2)[0]
    tx = x + max(10, (w - size[0]) // 2)
    ty = y + max(36, (h + size[1]) // 2 - 8)
    cv2.putText(canvas, label, (tx, ty), FONT, scale, TEXT, 2, cv2.LINE_AA)
    state = str(state_text).upper()
    state_scale = 0.44
    sw = cv2.getTextSize(state, FONT, state_scale, 1)[0][0]
    cv2.putText(canvas, state, (x + (w - sw) // 2, y + h - 16), FONT,
                state_scale, dot_color, 1, cv2.LINE_AA)


def draw_status_strip(canvas, chips, x=10, y=10, height=34, gap=8, max_x=None):
    """Draw compact sensor/model states and return the next x position."""
    right = canvas.shape[1] if max_x is None else int(max_x)
    cursor = int(x)
    for label, state, value in chips:
        text = str(label) if not value else f"{label} {value}"
        tw = cv2.getTextSize(text, FONT, 0.48, 1)[0][0]
        width = max(86, tw + 36)
        if cursor + width > right:
            break
        color = STATE_COLORS.get(state, GRAY)
        blend_rect(canvas, cursor, y, width, height, PANEL, 0.90)
        cv2.rectangle(canvas, (cursor, y), (cursor + width, y + height),
                      PANEL_LIGHT, 1)
        cv2.circle(canvas, (cursor + 14, y + height // 2), 5, color, -1, cv2.LINE_AA)
        cv2.putText(canvas, text, (cursor + 26, y + 22), FONT, 0.48,
                    TEXT, 1, cv2.LINE_AA)
        cursor += width + int(gap)
    return cursor


def draw_banner(canvas, text, level="warn", y=52, margin=10, height=42):
    color = STATE_COLORS.get(level, YELLOW)
    width = canvas.shape[1] - margin * 2
    blend_rect(canvas, margin, y, width, height, _dark(color, 0.28), 0.94)
    cv2.rectangle(canvas, (margin, y), (margin + width, y + height), color, 2)
    put_text_fit(canvas, text, margin + 14, y + 28, width - 28,
                 color=TEXT, base=0.66, minimum=0.42, thickness=2)


def draw_detection_label(canvas, x, y, title, value, detail, color,
                         max_width=340, min_y=50):
    main = f"{title}  {value}" if value else str(title)
    main_scale = fitted_text_scale(main, max_width - 24, base=0.76,
                                   minimum=0.48, thickness=2)
    main_w = cv2.getTextSize(main, FONT, main_scale, 2)[0][0]
    detail_w = cv2.getTextSize(str(detail), FONT, 0.42, 1)[0][0] if detail else 0
    width = min(max_width, max(190, main_w + 24, detail_w + 24))
    height = 58 if detail else 42
    x = max(4, min(int(x), canvas.shape[1] - width - 4))
    y = max(int(min_y), min(int(y), canvas.shape[0] - height - 4))
    blend_rect(canvas, x, y, width, height, (18, 20, 22), 0.92)
    cv2.rectangle(canvas, (x, y), (x + width, y + height), color, 2)
    cv2.putText(canvas, main, (x + 12, y + 27), FONT, main_scale,
                TEXT, 2, cv2.LINE_AA)
    if detail:
        put_text_fit(canvas, detail, x + 12, y + 49, width - 24,
                     color=TEXT_MUTED, base=0.42, minimum=0.34, thickness=1)
    return x, y, width, height


def friendly_reason(reason):
    raw = str(reason or "WAITING")
    mapping = {
        "NO_LIDAR": "LIDAR OFF",
        "STALE_LIDAR": "LIDAR STALE",
        "NO_POSE": "NO SLAM POSE",
        "STOP:POSE_SYNC": "POSE NOT SYNCED",
        "STOP:NO_POINTS": "NO LIDAR POINTS",
        "STOP:DET_LOST": "DETECTION LOST",
        "NO_REGION": "REGION NOT VISIBLE",
        "DETECTING": "MEASURING",
    }
    return mapping.get(raw, raw.replace("STOP:", "").replace("_", " "))
