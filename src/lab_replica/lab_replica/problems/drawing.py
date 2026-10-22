"""Top-down layout drawing of a problem (for setting it up on the real table)."""

import math

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from ..transforms import quat_to_matrix, rpy_to_matrix


def _font(size, bold=False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(f"/usr/share/fonts/truetype/dejavu/{name}", size)
    except OSError:
        return ImageFont.load_default()


def draw_layout(path, cell, items, prob, title="", px_per_m=900):
    """layout.png: top view of the table with a 10 cm grid, the basket and the
    items at their settled poses, and a legend with their coordinates."""
    t = cell["table"]
    x0, x1 = t["x_min"] - 0.05, t["x_max"] + 0.05
    y0, y1 = t["y_min"] - 0.05, t["y_max"] + 0.05
    margin = 70
    W = int((x1 - x0) * px_per_m) + 2 * margin
    H = int((y1 - y0) * px_per_m) + 2 * margin + 40 + 22 * 13

    def P(x, y):
        return (margin + (x - x0) * px_per_m, margin + 40 + (y1 - y) * px_per_m)

    img = Image.new("RGB", (W, H), (255, 255, 255))
    d = ImageDraw.Draw(img)
    f, fb, fs = _font(15), _font(18, True), _font(12)
    d.text((margin, 12), f"{title}  -  top view, world frame (cm)", font=fb, fill=(0, 0, 0))
    # Table top and the blue board.
    d.rectangle([P(t["x_min"], t["y_max"]), P(t["x_max"], t["y_min"])], fill=(225, 227, 230), outline=(120, 120, 120))
    d.rectangle([P(t["board_x"][0], t["board_y"][1]), P(t["board_x"][1], t["board_y"][0])], fill=(170, 210, 240),
                outline=(120, 120, 120))
    # Grid every 10 cm.
    for gx in np.arange(math.ceil(x0 * 10) / 10, x1, 0.1):
        d.line([P(gx, y0), P(gx, y1)], fill=(215, 215, 215))
        d.text((P(gx, y0)[0] - 10, P(gx, y0)[1] + 4), f"{gx * 100:.0f}", font=fs, fill=(90, 90, 90))
    for gy in np.arange(math.ceil(y0 * 10) / 10, y1, 0.1):
        d.line([P(x0, gy), P(x1, gy)], fill=(215, 215, 215))
        d.text((P(x0, gy)[0] - 40, P(x0, gy)[1] - 7), f"{gy * 100:.0f}", font=fs, fill=(90, 90, 90))
    d.text((P(x1, y0)[0] - 120, P(x1, y0)[1] + 20), "x ->   (front of the table below)", font=fs, fill=(60, 60, 60))
    # Robot base.
    bx, by = cell["robot"]["base_xyz"][:2]
    r = 0.075
    d.ellipse([P(bx - r, by + r), P(bx + r, by - r)], fill=(150, 150, 155), outline=(60, 60, 60))
    d.text((P(bx, by)[0] - 22, P(bx, by)[1] - 8), "UR", font=fb, fill=(255, 255, 255))
    # Basket.
    cx, cy = prob["basket_xy"]
    for rr, col in ((_basket_radius(cell, "top"), (160, 120, 70)), (_basket_radius(cell, "bottom"), (200, 170, 120))):
        d.ellipse([P(cx - rr, cy + rr), P(cx + rr, cy - rr)], outline=col, width=3)
    d.text((P(cx, cy)[0] - 60, P(cx, cy + _basket_radius(cell, "top"))[1] - 20),
           f"basket ({cx * 100:.0f}, {cy * 100:.0f})", font=f, fill=(120, 80, 40))
    # Objects.
    legend = []
    for k, p in enumerate(sorted(prob["placements"], key=lambda p: p.item)):
        obj = items[p.item]
        xyz, q = prob["settled"][p.item]
        R = quat_to_matrix(q)
        R_rest = rpy_to_matrix(*obj.poses[p.rest].rpy)
        Rz = R @ R_rest.T
        yaw = math.atan2(Rz[1, 0], Rz[0, 0])
        fp = obj.footprint_world(p.rest, xyz[0], xyz[1], yaw)
        col = tuple(int(255 * c) for c in obj.spec["color"])
        d.polygon([P(*v) for v in fp], fill=col, outline=(0, 0, 0))
        n = k + 1
        cxp, cyp = P(xyz[0], xyz[1])
        d.ellipse([cxp - 10, cyp - 10, cxp + 10, cyp + 10], fill=(255, 255, 255), outline=(0, 0, 0))
        d.text((cxp - (4 if n < 10 else 8), cyp - 8), str(n), font=fs, fill=(0, 0, 0))
        where = "in basket" if p.support == "basket" else "table"
        if prob.get("held") == p.item:
            where = "table, picked up by the setup step (starts in the gripper)"
        legend.append(f"{n:2d}. {p.item:28s} x={xyz[0] * 100:6.1f}  y={xyz[1] * 100:6.1f}  z={xyz[2] * 100:5.1f} cm  "
                      f"yaw={math.degrees(yaw):6.1f} deg  {p.rest:7s} {where}")
    y = H - 22 * 13 + 4
    for line in legend:
        d.text((margin, y), line, font=_mono(14), fill=(0, 0, 0))
        y += 22
    img.save(path)


def _mono(size):
    try:
        return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", size)
    except OSError:
        return ImageFont.load_default()


def _basket_radius(cell, which):
    b = cell["basket"]
    return (b["top_diameter"] if which == "top" else b["bottom_diameter"]) / 2
