"""Small procedural mesh toolkit: surfaces of revolution, lofts, boxes, swept
tubes, and a Wavefront OBJ/MTL writer that Gazebo (ogre2) loads directly."""

import math
import os

import numpy as np


class Mesh:
    """Triangle mesh with per-vertex normals and UVs and one material."""

    def __init__(self, v, n, uv, f, material):
        self.v = np.asarray(v, float)
        self.n = np.asarray(n, float)
        self.uv = np.asarray(uv, float)
        self.f = np.asarray(f, int)
        self.material = material

    def transformed(self, R=None, t=(0, 0, 0)):
        R = np.eye(3) if R is None else np.asarray(R, float)
        return Mesh(self.v @ R.T + np.asarray(t), self.n @ R.T, self.uv, self.f, self.material)


class Material:
    def __init__(self, name, color, texture=None, specular=0.15, shininess=20.0):
        self.name = name
        self.color = tuple(color)
        self.texture = texture  # file name relative to the OBJ
        self.specular = specular
        self.shininess = shininess


def _normalize(a):
    n = np.linalg.norm(a, axis=-1, keepdims=True)
    n[n == 0] = 1.0
    return a / n


def lathe(profile, material, segments=48, v_range=(0.0, 1.0), flip=False):
    """Surface of revolution about z.

    profile: list of (r, z) going along the surface. With flip=False the
    normals point outwards for a profile that runs bottom to top on the outer
    side. Sharp corners should be split into separate lathe calls.
    """
    p = np.asarray(profile, float)
    m = len(p)
    seg = np.diff(p, axis=0)
    length = np.concatenate([[0], np.cumsum(np.linalg.norm(seg, axis=1))])
    vcoord = v_range[0] + (v_range[1] - v_range[0]) * length / max(length[-1], 1e-9)
    # Profile tangent -> 2D normal (dr, dz) rotated by -90 deg.
    tang = np.zeros_like(p)
    tang[1:-1] = p[2:] - p[:-2]
    tang[0] = p[1] - p[0]
    tang[-1] = p[-1] - p[-2]
    tang = _normalize(tang)
    nrm2 = np.stack([tang[:, 1], -tang[:, 0]], axis=1)  # (nr, nz)
    if flip:
        nrm2 = -nrm2
    verts, norms, uvs = [], [], []
    for j in range(segments + 1):
        a = 2 * math.pi * j / segments
        c, s = math.cos(a), math.sin(a)
        for i in range(m):
            r, z = p[i]
            verts.append((r * c, r * s, z))
            norms.append((nrm2[i, 0] * c, nrm2[i, 0] * s, nrm2[i, 1]))
            uvs.append((j / segments, vcoord[i]))
    faces = []
    for j in range(segments):
        for i in range(m - 1):
            a = j * m + i
            b = (j + 1) * m + i
            if flip:
                faces += [(a, a + 1, b), (b, a + 1, b + 1)]
            else:
                faces += [(a, b, a + 1), (b, b + 1, a + 1)]
    return Mesh(verts, norms, uvs, faces, material)


def disk(r_in, r_out, z, material, segments=48, up=True):
    """Flat annulus (or disk when r_in == 0) at height z facing up or down."""
    prof = [(r_in, z), (r_out, z)] if not up else [(r_out, z), (r_in, z)]
    m = lathe(prof, material, segments)
    # Planar UVs look better on caps.
    m.uv = 0.5 + m.v[:, :2] / (2 * max(r_out, 1e-9))
    return m


def loft(sections, material, flip=False):
    """Skin through closed cross-sections of equal point count.

    sections: list of (N, 3) arrays, bottom to top, counter-clockwise seen
    from above. Normals are computed from the surface.
    """
    S = [np.asarray(s, float) for s in sections]
    n = len(S[0])
    m = len(S)
    verts = np.zeros(((n + 1) * m, 3))
    uvs = np.zeros(((n + 1) * m, 2))
    for k, s in enumerate(S):
        ring = np.vstack([s, s[:1]])
        verts[k * (n + 1):(k + 1) * (n + 1)] = ring
        uvs[k * (n + 1):(k + 1) * (n + 1), 0] = np.linspace(0, 1, n + 1)
        uvs[k * (n + 1):(k + 1) * (n + 1), 1] = k / max(m - 1, 1)
    faces = []
    for k in range(m - 1):
        for i in range(n):
            a = k * (n + 1) + i
            b = a + 1
            c = a + (n + 1)
            d = c + 1
            if flip:
                faces += [(a, c, b), (b, c, d)]
            else:
                faces += [(a, b, c), (b, d, c)]
    faces = np.array(faces)
    norms = vertex_normals(verts, faces)
    # Seam vertices share a position; average their normals.
    for k in range(m):
        i0, i1 = k * (n + 1), k * (n + 1) + n
        avg = _normalize(norms[i0] + norms[i1])
        norms[i0] = norms[i1] = avg
    return Mesh(verts, norms, uvs, faces, material)


def cap(section, material, up=True):
    """Fan-triangulated planar cap for a convex closed section."""
    s = np.asarray(section, float)
    c = s.mean(axis=0)
    verts = np.vstack([c, s])
    n = len(s)
    faces = []
    for i in range(n):
        a, b = 1 + i, 1 + (i + 1) % n
        faces.append((0, a, b) if up else (0, b, a))
    nz = 1.0 if up else -1.0
    norms = np.tile([0, 0, nz], (n + 1, 1))
    span = max(np.ptp(s[:, 0]), np.ptp(s[:, 1]), 1e-9)
    uvs = 0.5 + (verts[:, :2] - c[:2]) / span
    return Mesh(verts, norms, uvs, faces, material)


def vertex_normals(v, f):
    v = np.asarray(v, float)
    f = np.asarray(f, int)
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    vn = np.zeros_like(v)
    for k in range(3):
        np.add.at(vn, f[:, k], fn)
    return _normalize(vn)


def box(size, material, center=(0, 0, 0), uv_scale=None):
    """Axis-aligned box with flat-shaded faces. uv_scale maps metres to UV."""
    sx, sy, sz = (s / 2 for s in size)
    cx, cy, cz = center
    faces_def = [
        ((1, 0, 0), [(sx, -sy, -sz), (sx, sy, -sz), (sx, sy, sz), (sx, -sy, sz)]),
        ((-1, 0, 0), [(-sx, sy, -sz), (-sx, -sy, -sz), (-sx, -sy, sz), (-sx, sy, sz)]),
        ((0, 1, 0), [(sx, sy, -sz), (-sx, sy, -sz), (-sx, sy, sz), (sx, sy, sz)]),
        ((0, -1, 0), [(-sx, -sy, -sz), (sx, -sy, -sz), (sx, -sy, sz), (-sx, -sy, sz)]),
        ((0, 0, 1), [(-sx, -sy, sz), (sx, -sy, sz), (sx, sy, sz), (-sx, sy, sz)]),
        ((0, 0, -1), [(-sx, sy, -sz), (sx, sy, -sz), (sx, -sy, -sz), (-sx, -sy, -sz)]),
    ]
    verts, norms, uvs, faces = [], [], [], []
    for normal, quad in faces_def:
        base = len(verts)
        q = np.array(quad)
        # Choose the two in-plane axes for UVs.
        ax = [i for i in range(3) if normal[i] == 0]
        for corner in q:
            verts.append((corner[0] + cx, corner[1] + cy, corner[2] + cz))
            norms.append(normal)
            if uv_scale:
                uvs.append((corner[ax[0]] * uv_scale + 0.5, corner[ax[1]] * uv_scale + 0.5))
        if not uv_scale:
            uvs += [(0, 0), (1, 0), (1, 1), (0, 1)]
        faces += [(base, base + 1, base + 2), (base, base + 2, base + 3)]
    return Mesh(verts, norms, uvs, faces, material)


def tube(path, radius, material, sides=16):
    """Sweep a circle of the given radius along a 3D polyline."""
    P = np.asarray(path, float)
    sections = []
    prev_n = None
    for i in range(len(P)):
        t = P[min(i + 1, len(P) - 1)] - P[max(i - 1, 0)]
        t = t / np.linalg.norm(t)
        if prev_n is None:
            ref = np.array([0, 0, 1.0]) if abs(t[2]) < 0.9 else np.array([1.0, 0, 0])
            nrm = np.cross(t, ref)
        else:
            nrm = prev_n - np.dot(prev_n, t) * t
        nrm = nrm / np.linalg.norm(nrm)
        bin_ = np.cross(t, nrm)
        prev_n = nrm
        ang = np.linspace(0, 2 * math.pi, sides, endpoint=False)
        ring = P[i] + radius * (np.outer(np.cos(ang), nrm) + np.outer(np.sin(ang), bin_))
        sections.append(ring)
    return loft(sections, material)


def superellipse(a, b, z, n=4.0, points=48):
    """Rounded-rectangle cross-section with half-axes a (x) and b (y)."""
    t = np.linspace(0, 2 * math.pi, points, endpoint=False)
    c, s = np.cos(t), np.sin(t)
    x = a * np.sign(c) * np.abs(c) ** (2.0 / n)
    y = b * np.sign(s) * np.abs(s) ** (2.0 / n)
    return np.stack([x, y, np.full_like(x, z)], axis=1)


def write_obj(path, meshes):
    """Write meshes into one OBJ with a sibling MTL. Returns the OBJ path."""
    base = os.path.splitext(path)[0]
    mtl_name = os.path.basename(base) + ".mtl"
    mats = {}
    for m in meshes:
        mats[m.material.name] = m.material
    with open(base + ".mtl", "w") as fm:
        for mat in mats.values():
            r, g, b = mat.color
            fm.write(f"newmtl {mat.name}\n")
            fm.write(f"Ka {r:.4f} {g:.4f} {b:.4f}\nKd {r:.4f} {g:.4f} {b:.4f}\n")
            fm.write(f"Ks {mat.specular:.3f} {mat.specular:.3f} {mat.specular:.3f}\n")
            fm.write(f"Ns {mat.shininess:.1f}\nd 1.0\nillum 2\n")
            if mat.texture:
                fm.write(f"map_Kd {mat.texture}\n")
            fm.write("\n")
    with open(path, "w") as fo:
        fo.write(f"mtllib {mtl_name}\n")
        vo = 1
        for k, m in enumerate(meshes):
            fo.write(f"o part{k}\nusemtl {m.material.name}\n")
            for p in m.v:
                fo.write(f"v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
            for p in m.uv:
                fo.write(f"vt {p[0]:.5f} {p[1]:.5f}\n")
            for p in m.n:
                fo.write(f"vn {p[0]:.5f} {p[1]:.5f} {p[2]:.5f}\n")
            for a, b, c in m.f + vo:
                fo.write(f"f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}\n")
            vo += len(m.v)
    return path
