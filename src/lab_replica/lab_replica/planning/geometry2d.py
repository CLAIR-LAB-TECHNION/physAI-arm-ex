"""2D convex polygon tests used for footprints."""

import numpy as np


def _axes(poly):
    e = np.roll(poly, -1, axis=0) - poly
    n = np.stack([-e[:, 1], e[:, 0]], axis=1)
    norms = np.linalg.norm(n, axis=1, keepdims=True)
    return n / np.where(norms == 0, 1, norms)


def polygons_intersect(a, b, margin=0.0):
    """Separating-axis test for convex polygons, with an optional margin
    (polygons closer than `margin` count as intersecting)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    for ax in np.vstack([_axes(a), _axes(b)]):
        pa, pb = a @ ax, b @ ax
        if pa.max() + margin < pb.min() or pb.max() + margin < pa.min():
            return False
    return True


def polygon_inside_circle(poly, cx, cy, r):
    poly = np.asarray(poly, float)
    return bool(np.all(np.hypot(poly[:, 0] - cx, poly[:, 1] - cy) <= r))


def polygon_inside_rect(poly, x0, x1, y0, y1):
    poly = np.asarray(poly, float)
    return bool(np.all((poly[:, 0] >= x0) & (poly[:, 0] <= x1) & (poly[:, 1] >= y0) & (poly[:, 1] <= y1)))
