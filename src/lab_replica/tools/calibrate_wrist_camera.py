#!/usr/bin/env python3
"""Hand-eye calibration of the wrist RealSense from the scene itself (no target).

    calibrate_wrist_camera.py capture <dir> [--speed-scale S]
        Moves the arm through VIEWS (all high above the table, so the items
        cannot be hit), and saves a colour/depth frame and the joint angles at
        each. Needs the robot (or the simulation) and the camera running.
    calibrate_wrist_camera.py solve <dir>
        Estimates the pose of the colour optical frame in rg6_base_link and
        prints the cell.yaml values (wrist_camera optical_xyz, optical_rpy).

The solver needs no calibration target. The scene must stay still during
capture. It finds the transform that makes all views agree:
  * the blue board seen in every view must be level, at one height;
  * features matched between two views (SIFT on the colour images, filtered
    with RANSAC on the epipolar geometry) must land on the same world point
    when lifted with their depth.
The arm's kinematics are trusted. The start value is the one in cell.yaml.
"""

import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

from lab_replica import config  # noqa: E402

# Camera views (eye, target, up) in the world frame, around the middle of the
# blue board. The camera stays 0.55 m or more above the table, so the
# fingertips (about 0.24 m below the camera) clear the tallest item (0.27 m).
VIEWS = [
    ((0.46, 0.00, 0.78), (0.515, 0.00, 0.0), (0, -1, 0)),
    ((0.43, 0.00, 0.62), (0.43, 0.00, 0.0), (0, -1, 0)),
    ((0.43, 0.00, 0.62), (0.43, 0.00, 0.0), (1, 0, 0)),
    ((0.43, 0.00, 0.62), (0.43, 0.00, 0.0), (1, -1, 0)),
    ((0.30, -0.10, 0.62), (0.45, 0.00, 0.0), (0, -1, 0)),
    ((0.55, 0.10, 0.62), (0.45, -0.05, 0.0), (1, 0, 0)),
    ((0.35, 0.10, 0.58), (0.42, -0.05, 0.0), (-1, 0, 0)),
    ((0.55, -0.15, 0.58), (0.45, 0.05, 0.0), (0, -1, 0)),
    ((0.43, 0.00, 0.72), (0.43, 0.00, 0.0), (-1, 0, 0)),
    ((0.38, -0.05, 0.56), (0.38, -0.05, 0.0), (0, -1, 0)),
]
MIN_TIP_Z = 0.30   # lowest fingertip height on the way between views [m]
# Every view's arm pose is the IK solution nearest to the first view's, so the
# wrist turns at most about 125 deg from there (the camera cable runs along the arm).


# ---- capture -------------------------------------------------------------------------

def capture(out_dir, speed_scale):
    import cv2
    import rclpy

    from lab_replica import objects
    from lab_replica.autonomy.views import ViewPlanner, look_at
    from lab_replica.execution.executor import Executor, active_trajectory_controller
    from lab_replica.perception.rgbd import RGBDCamera
    from lab_replica.planning.motion import PlanError
    from lab_replica.planning.skills import Skills
    from lab_replica.robot import rg6

    os.makedirs(out_dir, exist_ok=True)
    cell = config.load_cell()
    sk = Skills(cell, objects.load_items(), objects.load_basket(cell))
    vp = ViewPlanner(sk, cell["robot"]["ur_type"])
    width = rg6.width_from_q(-0.3)

    rclpy.init()
    probe = rclpy.create_node("calibrate_probe")
    controller = active_trajectory_controller(probe, timeout=30.0)
    probe.destroy_node()
    node = Executor(controller=controller, speed_scale=speed_scale)
    while node.current() is None:
        pass
    if not node.traj.wait_for_server(timeout_sec=30.0):
        raise SystemExit(f"no {controller} action server")
    cam = RGBDCamera(node, cell["robot"]["ur_type"])

    def check(path):
        for q in path:
            if sk.robot.world_points(q, width)[1][:, 2].min() < MIN_TIP_Z:
                raise PlanError("fingertips too low")
        sk.check_path(path, width, {}, label="calibration view")

    q_ref = None
    for k, (eye, target, up) in enumerate(VIEWS):
        q0 = node.current()
        q1 = vp.ik_view(look_at(eye, target, up), q0 if q_ref is None else q_ref)
        if q_ref is None:
            q_ref = q1
        if q1 is None:
            print(f"view {k}: out of reach, skipped")
            continue
        path = sk.P.joint_move(q0, q1, step=0.02)
        try:
            check(path)
        except PlanError:
            # Rise first: camera straight up to 0.75 m, then over.
            T = vp.camera_pose(q0)
            q_up = vp.ik_view(look_at((T[0, 3], T[1, 3], 0.75), (T[0, 3], T[1, 3], 0.0), up), q0)
            path = sk.P.joint_move(q0, q_up, step=0.02) + sk.P.joint_move(q_up, q1, step=0.02)[1:]
            try:
                check(path)
            except PlanError as e:
                print(f"view {k}: no safe motion ({e}), skipped")
                continue
        print(f"view {k}: moving")
        node.run_traj(path, sk.P.time_path(path), label=f"view {k}")
        t = node.get_clock().now().nanoseconds * 1e-9 + 0.5
        f = cam.capture(after=t)
        q = node.current()
        cv2.imwrite(os.path.join(out_dir, f"{k:02d}_color.png"), cv2.cvtColor(f.rgb, cv2.COLOR_RGB2BGR))
        np.save(os.path.join(out_dir, f"{k:02d}_depth.npy"), f.depth.astype(np.float32))
        with open(os.path.join(out_dir, f"{k:02d}.json"), "w") as fh:
            json.dump({"q": [float(v) for v in q], "K": f.K.tolist()}, fh)
        print(f"view {k}: saved")
    node.destroy_node()
    rclpy.shutdown()


# ---- solve ---------------------------------------------------------------------------

def load_views(d):
    import cv2
    views = []
    for name in sorted(os.listdir(d)):
        if not name.endswith(".json"):
            continue
        k = name[:-5]
        meta = json.load(open(os.path.join(d, name)))
        z = np.load(os.path.join(d, f"{k}_depth.npy")).astype(float)
        z[~np.isfinite(z) | (z <= 0)] = np.nan
        views.append(dict(name=k, q=np.array(meta["q"]), K=np.array(meta["K"]), depth=z,
                          bgr=cv2.imread(os.path.join(d, f"{k}_color.png"))))
    return views


def board_pixels(v, n=1500, rng=None):
    """Pixels on the blue board, away from its edges and the items."""
    import cv2
    hsv = cv2.cvtColor(v["bgr"], cv2.COLOR_BGR2HSV)
    m = (hsv[..., 0] > 90) & (hsv[..., 0] < 112) & (hsv[..., 1] > 90) & np.isfinite(v["depth"])
    m = cv2.erode(m.astype(np.uint8), np.ones((21, 21), np.uint8)).astype(bool)
    vv, uu = np.nonzero(m)
    i = (rng or np.random.default_rng(0)).choice(len(uu), min(n, len(uu)), replace=False)
    return uu[i], vv[i]


def depth_at(z, u, v, r=2):
    h, w = z.shape
    out = np.full(len(u), np.nan)
    for i, (a, b) in enumerate(zip(np.round(u).astype(int), np.round(v).astype(int))):
        if r <= a < w - r and r <= b < h - r:
            patch = z[b - r:b + r + 1, a - r:a + r + 1]
            ok = patch[np.isfinite(patch)]
            if ok.size >= 5 and ok.max() - ok.min() < 0.01:   # no depth edge
                out[i] = np.median(ok)
    return out


def lift(K, u, v, z):
    return np.c_[(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z]


def matches(a, b):
    """Pixel pairs (ua, va, ub, vb) of features matched between two views."""
    import cv2
    ka, da = a["kp"]
    kb, db = b["kp"]
    if da is None or db is None or len(ka) < 8 or len(kb) < 8:
        return np.zeros((0, 4))
    m = cv2.BFMatcher().knnMatch(da, db, k=2)
    good = [x[0] for x in m if len(x) == 2 and x[0].distance < 0.7 * x[1].distance]
    if len(good) < 12:
        return np.zeros((0, 4))
    pa = np.float32([ka[g.queryIdx].pt for g in good])
    pb = np.float32([kb[g.trainIdx].pt for g in good])
    _, inl = cv2.findFundamentalMat(pa, pb, cv2.FM_RANSAC, 1.5, 0.999)
    if inl is None:
        return np.zeros((0, 4))
    inl = inl.ravel().astype(bool)
    return np.c_[pa[inl], pb[inl]]


def solve(d):
    import cv2
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation

    from lab_replica.robot.kinematics import Chain, robot_urdf
    from lab_replica.transforms import make_tf, rpy_to_matrix

    cell = config.load_cell()
    wc = cell["wrist_camera"]
    urdf = robot_urdf(cell["robot"]["ur_type"])
    grip = Chain(urdf, "world", "rg6_base_link")
    views = load_views(d)
    if len(views) < 3:
        raise SystemExit(f"{d}: need 3 or more views, found {len(views)}")
    sift = cv2.SIFT_create(3000)
    rng = np.random.default_rng(0)
    for v in views:
        v["Tg"] = grip.fk(v["q"])
        v["kp"] = sift.detectAndCompute(cv2.cvtColor(v["bgr"], cv2.COLOR_BGR2GRAY), None)
        u, vv = board_pixels(v, rng=rng)
        v["plane"] = lift(v["K"], u, vv, v["depth"][vv, u])

    pairs = []
    for i in range(len(views)):
        for j in range(i + 1, len(views)):
            m = matches(views[i], views[j])
            if len(m) == 0:
                continue
            za = depth_at(views[i]["depth"], m[:, 0], m[:, 1])
            zb = depth_at(views[j]["depth"], m[:, 2], m[:, 3])
            ok = np.isfinite(za) & np.isfinite(zb)
            if ok.sum() >= 8:
                pa = lift(views[i]["K"], m[ok, 0], m[ok, 1], za[ok])
                pb = lift(views[j]["K"], m[ok, 2], m[ok, 3], zb[ok])
                pairs.append((i, j, pa, pb))
    n_feat = sum(len(p[2]) for p in pairs)
    print(f"{len(views)} views, {sum(len(v['plane']) for v in views)} board points, "
          f"{n_feat} matched features in {len(pairs)} view pairs")

    T0 = make_tf(rpy_to_matrix(*wc["optical_rpy"]), wc["optical_xyz"])
    x0 = np.r_[Rotation.from_matrix(T0[:3, :3]).as_rotvec(), T0[:3, 3], 0.0]

    def T_gc(x):
        return make_tf(Rotation.from_rotvec(x[:3]).as_matrix(), x[3:6])

    def to_world(v, T, P):
        Tw = v["Tg"] @ T
        return P @ Tw[:3, :3].T + Tw[:3, 3]

    def residuals(x, split=False):
        T = T_gc(x)
        plane = np.concatenate([to_world(v, T, v["plane"])[:, 2] - x[6] for v in views])
        feat = np.concatenate([(to_world(views[i], T, pa) - to_world(views[j], T, pb)).ravel()
                               for i, j, pa, pb in pairs]) if pairs else np.zeros(0)
        if split:
            return plane, feat
        # One board point per pixel is plentiful: weight the two terms alike.
        w_plane = math.sqrt(max(n_feat, 1) * 3 / max(len(plane), 1))
        return np.r_[w_plane * plane, feat]

    def report(x, title):
        plane, feat = residuals(x, split=True)
        f = np.linalg.norm(feat.reshape(-1, 3), axis=1) if len(feat) else np.zeros(1)
        print(f"{title}: board height {x[6] * 1000:+.1f} mm, board rms {np.sqrt(np.mean(plane ** 2)) * 1000:.1f} mm, "
              f"feature distance median {np.median(f) * 1000:.1f} mm, 90% {np.percentile(f, 90) * 1000:.1f} mm")

    report(x0, "cell.yaml")
    sol = least_squares(residuals, x0, loss="soft_l1", f_scale=0.01, x_scale=[0.02] * 3 + [0.01] * 3 + [0.005])
    x = sol.x
    report(x, "solution ")
    T = T_gc(x)
    rpy = Rotation.from_matrix(T[:3, :3]).as_euler("xyz")
    assert np.allclose(rpy_to_matrix(*rpy), T[:3, :3], atol=1e-9)
    tilt = math.degrees(math.acos(np.clip(T[2, 2], -1, 1)))
    print(f"optical axis {tilt:.1f} deg from the gripper axis; lens {np.hypot(*T[:2, 3]) * 1000:.0f} mm off it")
    print("cell.yaml, section wrist_camera:")
    print(f"  optical_xyz: [{T[0, 3]:.4f}, {T[1, 3]:.4f}, {T[2, 3]:.4f}]")
    print(f"  optical_rpy: [{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}]")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture", help="move through the views and save frames")
    c.add_argument("dir")
    c.add_argument("--speed-scale", type=float, default=0.5)
    s = sub.add_parser("solve", help="estimate the hand-eye transform from saved frames")
    s.add_argument("dir")
    args = ap.parse_args()
    if args.cmd == "capture":
        capture(args.dir, args.speed_scale)
    else:
        solve(args.dir)


if __name__ == "__main__":
    main()
