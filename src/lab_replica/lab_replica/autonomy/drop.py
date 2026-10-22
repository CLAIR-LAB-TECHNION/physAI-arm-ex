"""Where to drop an item into the basket: its emptiest spot.

Items are never taken out of the basket again, so an item put in is simply
dropped from above the rim onto whatever is there. The spot is chosen from
the depth seen in the basket (a height map, so items nobody recognised
count too; with a depth_world.DepthWorld its height map, where unseen
space counts as piled up to the bound of the camera rays): for every
position and gripper yaw, the item's footprint is laid over the map; the
lowest pile under it wins, then the most free room around it.
"""

import math
from dataclasses import dataclass

import cv2
import numpy as np

from .obstacles import CELL, height_map

PILE_WEIGHT = 1.0       # cost per metre of pile under the item
ROOM_WEIGHT = 0.5       # cost per metre of free room around the footprint (more room is better)
MAX_ROOM = 0.05         # more room than this does not count
OCCUPIED = 0.010        # a cell is occupied this much above the basket floor
WALL_MARGIN = 0.005     # footprint kept this far inside the floor's edge


@dataclass
class DropSpots:
    spots: list            # [(cost, x, y, yaw, z_pile)], best first
    heights: np.ndarray    # (ny, nx) pile height above the basket floor [m], NaN outside the basket floor
    x0: float
    y0: float
    z_floor: float         # basket floor, world z


def drop_spots(frames, cell, basket, basket_xy, footprint, yaws, stack=(), step=0.01, depth=None, items=()):
    """Candidate drop spots in the basket, best first. footprint: the
    item's outline (N, 2) in its rested frame; yaws: the item yaws that
    can be dropped (one per gripper yaw); stack: [(footprint polygon, top
    z)] of what is known to rest in the basket. depth: the DepthWorld to take
    the heights from instead of the frames (its cells of `items` and the
    unrecognised ones)."""
    bx, by = basket_xy
    z_floor = cell["table"]["surface_z"] + basket.floor
    r_floor = basket.inner_radius(0.0)
    r_top = basket.inner_radius(basket.height)
    x_range, y_range = (bx - r_top - 0.02, bx + r_top + 0.02), (by - r_top - 0.02, by + r_top + 0.02)
    if depth is None:
        H, x0, y0 = height_map(frames, cell, x_range=x_range, y_range=y_range)
        ny, nx = H.shape
        X, Y = np.meshgrid(x0 + (np.arange(nx) + 0.5) * CELL, y0 + (np.arange(ny) + 0.5) * CELL)
    else:
        x0, y0 = x_range[0], y_range[0]
        nx, ny = int(np.ceil((x_range[1] - x0) / CELL)), int(np.ceil((y_range[1] - y0) / CELL))
        X, Y = np.meshgrid(x0 + (np.arange(nx) + 0.5) * CELL, y0 + (np.arange(ny) + 0.5) * CELL)
        H = depth.heights_at(X, Y, radius=CELL / 2, items=items)
    R = np.hypot(X - bx, Y - by)
    # Inside the floor's radius the camera sees the floor or what lies on
    # it (unseen counts as floor); farther out it sees the sloping wall.
    inside = R < r_floor - 0.012
    pile = np.where(np.isfinite(H), np.maximum(H - z_floor, 0.0), 0.0)
    for poly, top in stack:
        m = np.zeros(H.shape, np.uint8)
        cv2.fillConvexPoly(m, _cells(poly, x0, y0), 1)
        pile = np.where(m > 0, np.maximum(pile, top - z_floor), pile)
    pile = np.where(inside, pile, np.nan)
    free = inside & (pile <= OCCUPIED)
    room = cv2.distanceTransform(free.astype(np.uint8), cv2.DIST_L2, 3) * CELL

    fp = np.asarray(footprint, float)
    spots = []
    # The item should land on the floor (or what is on it), not lean on the
    # wall; an item too big for that is dropped as far in as it goes.
    for limit in (r_floor - WALL_MARGIN, r_top - WALL_MARGIN, math.inf):
        for yaw in yaws:
            c, s = math.cos(yaw), math.sin(yaw)
            rot = fp @ np.array([[c, s], [-s, c]])
            reach = float(np.max(np.hypot(rot[:, 0], rot[:, 1])))
            r_max = max(limit - reach, 0.0) if math.isfinite(limit) else 0.0
            n = int(r_max / step)
            for i in range(-n, n + 1):
                for j in range(-n, n + 1):
                    dx, dy = i * step, j * step
                    if dx * dx + dy * dy > r_max * r_max + 1e-12:
                        continue
                    poly = rot + (bx + dx, by + dy)
                    if math.isfinite(limit) and np.max(np.hypot(poly[:, 0] - bx, poly[:, 1] - by)) > limit:
                        continue
                    m = np.zeros(H.shape, np.uint8)
                    cv2.fillConvexPoly(m, _cells(poly, x0, y0), 1)
                    under = (m > 0) & inside
                    z = float(np.nanmax(pile[under])) if under.any() else 0.0
                    free_room = float(np.min(room[m > 0])) if (m > 0).any() else 0.0
                    cost = PILE_WEIGHT * z - ROOM_WEIGHT * min(free_room, MAX_ROOM)
                    spots.append((cost, bx + dx, by + dy, yaw, z_floor + z))
        if spots:
            break
    spots.sort(key=lambda t: t[0])
    return DropSpots(spots, np.where(inside, pile, np.nan), x0, y0, z_floor)


def spread(spots, n=12, min_dist=0.03):
    """Up to n spots, best first, at least min_dist apart (when one cannot
    be reached, its neighbours usually cannot either)."""
    out = []
    for s in spots:
        if all(math.hypot(s[1] - o[1], s[2] - o[2]) >= min_dist for o in out):
            out.append(s)
            if len(out) >= n:
                break
    return out


def _cells(poly, x0, y0):
    """Polygon in height-map cell coordinates (for cv2 rasterisation)."""
    return np.round((np.asarray(poly, float) - (x0, y0)) / CELL - 0.5).astype(np.int32)
