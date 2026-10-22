"""The lab table (slatted aluminium top with the blue board on it) with the
robot's mounting plate.

cell.yaml sections: `table` (top and board extents, height, board thickness,
slat pitch, frame profile) and `robot` (plate_size; the plate is base_xyz[2]
thick).
The frame rails and legs are approximate; nothing in the task touches them.
"""

from ..gazebo import sdf

BLUE = (0.07, 0.45, 0.78)
ALUMINIUM = (0.70, 0.72, 0.74)
PLATE = (0.62, 0.63, 0.65)


def table_sdf(cell):
    """model.sdf text of the static table model."""
    t, r = cell["table"], cell["robot"]
    p = t["frame_profile"]
    x0, x1, y0, y1 = t["x_min"], t["x_max"], t["y_min"], t["y_max"]
    (bx0, bx1), (by0, by1) = t["board_x"], t["board_y"]
    th = t["board_thickness"]
    H = t["height"]
    parts = [sdf.box("blue_board", (bx1 - bx0, by1 - by0, th), ((bx0 + bx1) / 2, (by0 + by1) / 2, -th / 2), BLUE,
                     specular=0.05)]
    # Slatted top: extrusions along x, their tops under the board.
    pitch, slat_h = t["slat_pitch"], 0.012
    n = int(round((y1 - y0) / pitch))
    for k in range(n):
        yc = y0 + (k + 0.5) * (y1 - y0) / n
        parts.append(sdf.box(f"slat{k}", (x1 - x0, 0.8 * pitch, slat_h), ((x0 + x1) / 2, yc, -th - slat_h / 2),
                             ALUMINIUM, specular=0.5))
    th += slat_h
    # Frame under the top.
    zf = -th - p / 2
    rail = dict(specular=0.5)
    parts += [
        sdf.box("rail_front", (x1 - x0 + 0.06, p, p), ((x0 + x1) / 2, y0 - p / 2 + 0.01, zf), ALUMINIUM, **rail),
        sdf.box("rail_back", (x1 - x0 + 0.06, p, p), ((x0 + x1) / 2, y1 + p / 2, -th + p / 2 - 0.004), ALUMINIUM, **rail),
        sdf.box("rail_left", (p, y1 - y0, p), (x0 - p / 2, (y0 + y1) / 2, zf - 0.02), ALUMINIUM, **rail),
        sdf.box("rail_right", (p, y1 - y0, p), (x1 + p / 2, (y0 + y1) / 2, zf), ALUMINIUM, **rail),
    ]
    for k, (lx, ly) in enumerate([(x0, y0), (x1, y0), (x0, y1), (x1, y1)]):
        parts.append(sdf.box(f"leg{k}", (p, p, H - th), (lx, ly, -th - (H - th) / 2), ALUMINIUM, **rail))
    # Lower shelf rail, visible under the front edge in the photos.
    parts.append(sdf.box("rail_low", (x1 - x0, p, p), ((x0 + x1) / 2, y0, -H + 0.15), ALUMINIUM, **rail))
    # Robot mounting plate: fills the gap between the table and the robot's mounting face.
    ps, pt = r["plate_size"], max(r["base_xyz"][2], 0.001)
    parts.append(sdf.box("robot_plate", (ps[0], ps[1], pt), (r["base_xyz"][0], r["base_xyz"][1], pt / 2), PLATE,
                         specular=0.6))
    return sdf.static_model("lab_table", "".join(parts))
