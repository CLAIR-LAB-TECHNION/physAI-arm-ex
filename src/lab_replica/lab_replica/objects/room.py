"""Background behind the table: floor, wall with cable trunking and sockets,
window sill, window and the frame uprights. Seen by the camera only; only
the floor collides.

cell.yaml sections: `room` (wall_y, trunking_height, socket_x, sill_z,
sill_depth, window_height, uprights_x, crossbar_z) and `table` (height).
"""

from ..gazebo import sdf

WALL = (0.86, 0.86, 0.83)
ALUMINIUM = (0.70, 0.72, 0.74)
SOCKET_COLORS = [(0.92, 0.92, 0.90), (0.92, 0.92, 0.90), (0.12, 0.25, 0.65)]   # one per socket plate


def room_sdf(cell):
    """model.sdf text of the static room model."""
    rm = cell["room"]
    wy = rm["wall_y"]
    H = cell["table"]["height"]
    sill_z = rm["sill_z"]
    sd = rm["sill_depth"]
    parts = [sdf.box("floor", (8, 8, 0.02), (0.5, 0, -H - 0.01), (0.55, 0.55, 0.53))]
    # Lower wall up to the sill, with cable trunking and sockets on it.
    parts.append(sdf.box("wall_low", (8, 0.1, H + sill_z), (0.5, wy + 0.05, (sill_z - H) / 2), WALL, collide=False))
    th = rm["trunking_height"]
    parts.append(sdf.box("trunking", (6, 0.05, th), (0.5, wy - 0.025, th / 2 - 0.005), (0.88, 0.88, 0.86),
                         collide=False))
    for k, (sx, accent) in enumerate(zip(rm["socket_x"], SOCKET_COLORS)):
        parts.append(sdf.box(f"socket_plate{k}", (0.12, 0.012, 0.09), (sx, wy - 0.055, 0.075), (0.96, 0.96, 0.95),
                             collide=False))
        for j in (-1, 1):
            parts.append(sdf.box(f"socket{k}_{j}", (0.035, 0.006, 0.04), (sx + j * 0.025, wy - 0.063, 0.075), accent,
                                 collide=False))
    # Window sill (grey stone) in a niche, then the window.
    parts.append(sdf.box("sill", (8, sd + 0.02, 0.03), (0.5, wy + sd / 2, sill_z + 0.015), (0.52, 0.53, 0.55),
                         specular=0.3, collide=False))
    parts.append(sdf.box("window_frame", (8, 0.06, 0.06), (0.5, wy + sd + 0.03, sill_z + 0.06), (0.25, 0.26, 0.28),
                         collide=False))
    parts.append(sdf.box("window", (8, 0.02, rm["window_height"]),
                         (0.5, wy + sd + 0.07, sill_z + 0.09 + rm["window_height"] / 2),
                         (0.80, 0.86, 0.90), collide=False, emissive=(0.55, 0.60, 0.65)))
    parts.append(sdf.box("wall_top", (8, 0.1, 1.5), (0.5, wy + 0.05, sill_z + 0.09 + rm["window_height"] + 0.75), WALL,
                         collide=False))
    # Frame uprights at the back of the table, joined at the top.
    for k, px in enumerate(rm["uprights_x"]):
        parts.append(sdf.box(f"upright{k}", (0.045, 0.045, 1.4), (px, wy - 0.03, 0.70), ALUMINIUM, specular=0.5,
                             collide=False))
    ux0, ux1 = min(rm["uprights_x"]), max(rm["uprights_x"])
    parts.append(sdf.box("crossbar", (ux1 - ux0 + 0.045, 0.045, 0.045), ((ux0 + ux1) / 2, wy - 0.03, rm["crossbar_z"]),
                         ALUMINIUM, specular=0.5, collide=False))
    return sdf.static_model("lab_room", "".join(parts))
