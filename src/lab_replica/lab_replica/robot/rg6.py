"""OnRobot RG6 kinematics (matches description/urdf/rg6.xacro).

q is the finger_joint angle: -0.628 fully open, +0.628 closed. Width is the
distance between the two pad faces. Depth is measured along the gripper axis
from the RG6 mounting face (rg6_base_link origin) to the fingertip ends.
Because the fingers move on a parallelogram, the fingertips reach further out
(deeper) as the gripper closes: 0.228 m when fully open, 0.283 m when closed.
"""

import math

Q_MIN, Q_MAX = -0.628319, 0.628319
KNUCKLE_PIVOT = (-0.024112, 0.136813)   # (y, z) of the left knuckle pivot
KNUCKLE_VEC = (-0.047335, 0.064495)     # pivot -> fingertip joint, at q = 0
PAD_FACE_Y = 0.0246                     # pad face offset in the finger frame
TIP_Z = 0.0664                          # fingertip end in the finger frame
PAD_LENGTH = 0.050                      # contact length of the pads
FINGER_HALF_WIDTH = 0.0125              # pads are 25 mm wide
FINGER_THICKNESS = 0.031                # finger depth behind the pad face

MAX_WIDTH = 0.151

# Collision boxes of the knuckle bars (description/urdf/rg6.xacro), in the
# left knuckle frames: (pivot (y, z) in rg6_base_link, s range along the bar,
# t range across it). The bar direction at q = 0 is KNUCKLE_VEC; t is along
# (-u_z, u_y). Both knuckles turn by q about -x.
KNUCKLE_BOXES = [
    ((-0.024112, 0.136813), (0.012, 0.086), (-0.0062, 0.0065)),   # outer
    ((-0.012720, 0.159500), (0.004, 0.057), (-0.0086, 0.0017)),   # inner, bar
    ((-0.012720, 0.159500), (0.057, 0.071), (-0.0053, 0.0017)),   # inner, neck
    ((-0.012720, 0.159500), (0.071, 0.086), (-0.0078, 0.0041)),   # inner, eye
]
# Body collision box (y half width, z range) below the knuckle pivots.
BODY_BOX = (0.042, (0.025, 0.167))
# Joints of the simulated gripper (see description/urdf/rg6.xacro), in the
# order used by rg6_controller.
JOINT_ORDER = [
    "left_finger_slide",
    "right_finger_slide",
    "left_finger_lift",
    "right_finger_lift",
    "finger_joint",
    "left_inner_knuckle_joint",
    "right_outer_knuckle_joint",
    "right_inner_knuckle_joint",
]


def _finger_origin(q):
    a, b = KNUCKLE_VEC
    y = KNUCKLE_PIVOT[0] + a * math.cos(q) + b * math.sin(q)
    z = KNUCKLE_PIVOT[1] - a * math.sin(q) + b * math.cos(q)
    return y, z


def width_from_q(q):
    y, _ = _finger_origin(q)
    return -2.0 * (y + PAD_FACE_Y)


def depth_from_q(q):
    _, z = _finger_origin(q)
    return z + TIP_Z


def q_from_width(w):
    """Inverse of width_from_q, clamped to the joint range."""
    a, b = KNUCKLE_VEC
    # w = -2 (p + a cos q + b sin q + f) -> A cos q - B sin q = C
    A, B = -2 * a, 2 * b
    C = w + 2 * (KNUCKLE_PIVOT[0] + PAD_FACE_Y)
    R = math.hypot(A, B)
    d = math.atan2(B, A)
    c = max(-1.0, min(1.0, C / R))
    q = math.acos(c) - d
    return max(Q_MIN, min(Q_MAX, q))


def depth_from_width(w):
    return depth_from_q(q_from_width(w))


def _knuckle_polygons(q):
    """(y, z) corners of the left knuckle bars at knuckle angle q."""
    a, b = KNUCKLE_VEC
    L = math.hypot(a, b)
    u = (a / L, b / L)
    v = (-u[1], u[0])
    c, s = math.cos(q), math.sin(q)
    out = []
    for (py, pz), (s0, s1), (t0, t1) in KNUCKLE_BOXES:
        poly = []
        for sv, tv in ((s0, t0), (s1, t0), (s1, t1), (s0, t1)):
            y = sv * u[0] + tv * v[0]
            z = sv * u[1] + tv * v[1]
            poly.append((py + y * c + z * s, pz - y * s + z * c))
        out.append(poly)
    return out


def free_height(width):
    """Height above the fingertip ends up to which the space between the
    pads is free when they are `width` apart.

    Above it the inner knuckles (about 6 cm up) and then the outer knuckles
    and the body reach into the gap, so a held object must stay below it."""
    q = q_from_width(width)
    depth = depth_from_q(q)
    y_face = -width / 2   # left pad face; the gap is y > y_face
    by, (bz0, bz1) = BODY_BOX
    polys = _knuckle_polygons(q) + [[(-by, bz0), (by, bz0), (by, bz1), (-by, bz1)]]
    z_low = -math.inf
    for poly in polys:
        # Clip the polygon to the half plane y >= y_face; its lowest point
        # there (largest z) is a corner or an edge crossing.
        for (y0, z0), (y1, z1) in zip(poly, poly[1:] + poly[:1]):
            if y0 >= y_face:
                z_low = max(z_low, z0)
            if (y0 - y_face) * (y1 - y_face) < 0:
                z_low = max(z_low, z0 + (z1 - z0) * (y_face - y0) / (y1 - y0))
    return depth - z_low


def _ref():
    return _finger_origin(0.0)


def slide_from_q(q):
    """Prismatic slide of one finger (positive closes) for knuckle angle q."""
    return _finger_origin(q)[0] - _ref()[0]


def lift_from_q(q):
    """Prismatic lift (along the gripper axis) for knuckle angle q."""
    return _finger_origin(q)[1] - _ref()[1]


def q_from_slide(s):
    """Knuckle angle of one finger from its slide (half of the width)."""
    half = -(_ref()[0] + s + PAD_FACE_Y)
    return q_from_width(2 * half)


def width_from_slides(s_left, s_right):
    half = lambda s: -(_ref()[0] + s + PAD_FACE_Y)  # noqa: E731
    return half(s_left) + half(s_right)


def joint_command(q_left, q_right=None, s_left=None, s_right=None):
    """Positions for JOINT_ORDER. The slides default to the linkage values of
    q; pass s_* to command the slides separately (when squeezing)."""
    if q_right is None:
        q_right = q_left
    sl = slide_from_q(q_left) if s_left is None else s_left
    sr = slide_from_q(q_right) if s_right is None else s_right
    return [sl, sr, lift_from_q(q_left), lift_from_q(q_right), q_left, -q_left, -q_right, -q_right]
