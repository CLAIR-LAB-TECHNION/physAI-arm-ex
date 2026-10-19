"""The woven basket: a frustum open at the top.

cell.yaml section `basket`: center_xy, top_diameter, bottom_diameter, height,
wall_thickness, bottom_thickness, mass.

Model frame: origin at the centre of the bottom, on the table surface.
"""

import math
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

from ..gazebo import meshgen as mg
from ..gazebo.textures import noise
from .base import Shape, ring_boxes

FRICTION = 0.8


class Basket:
    def __init__(self, spec):
        self.spec = spec
        self.r_top = spec["top_diameter"] / 2
        self.r_bottom = spec["bottom_diameter"] / 2
        self.height = spec["height"]
        self.wall = spec["wall_thickness"]
        self.floor = spec["bottom_thickness"]
        self.mass = spec["mass"]
        self.slope = (self.r_top - self.r_bottom) / self.height   # radius change per metre of height
        # The floor disk carries the basket; the wall boxes start 1 cm up so
        # they never touch the table (contacts there cost solver time).
        self.shapes = [Shape("cylinder", (self.r_bottom, self.floor), (0, 0, self.floor / 2))]
        lift = 0.01
        self.shapes += ring_boxes(self.r_bottom - self.wall + self.slope * lift, self.r_bottom + self.slope * lift,
                                  lift, self.height, n=18, tilt_out=self.slope)
        # Rough mass properties of a thin-walled bowl.
        self.inertia = np.diag([1.0, 1.0, 1.6]) * self.mass * self.r_top ** 2 / 2.5
        self.com = np.array([0, 0, self.height * 0.4])

    def inner_radius(self, z):
        """Inner wall radius at height z above the basket bottom."""
        return self.r_bottom - self.wall + self.slope * max(0.0, z)

    def visual(self, texture_dir):
        rt, rb, H = self.r_top, self.r_bottom, self.height
        w, fl = self.wall, self.floor
        _weave_texture(os.path.join(texture_dir, "weave.png"), seed=3)
        _coil_texture(os.path.join(texture_dir, "coil.png"), seed=4)
        weave = mg.Material("basket_weave", (1, 1, 1), texture="weave.png", specular=0.05, shininess=5)
        weave_in = mg.Material("basket_weave_in", (0.92, 0.92, 0.92), texture="weave.png", specular=0.03, shininess=5)
        coil = mg.Material("basket_coil", (1, 1, 1), texture="coil.png", specular=0.03, shininess=5)
        rim = mg.Material("basket_rim", (0.66, 0.53, 0.38), specular=0.05)
        return [
            mg.lathe([(rb, 0.0), (rt, H)], weave, segments=96),
            mg.lathe([(rt, H), (rt - w * 0.3, H + 0.005), (rt - w, H)], rim, segments=96),
            mg.lathe([(rt - w, H), (rb - w, fl)], weave_in, segments=96),
            mg.disk(0.0, rb - w, fl, coil, segments=72, up=True),
            mg.disk(0.0, rb, 0.0, coil, segments=72, up=False),
        ]


def _weave_texture(path, seed, w=1024, h=256, rows=14, cols=96):
    """Herringbone-like seagrass weave of the wall."""
    rng = np.random.default_rng(seed)
    img = Image.new("RGB", (w, h), (176, 140, 100))
    d = ImageDraw.Draw(img)
    rh = h / rows
    cw = w / cols
    for r in range(rows):
        for c in range(cols):
            x0, y0 = c * cw, r * rh
            tone = rng.normal(0, 10)
            base = np.array([196, 160, 116]) + tone
            col = tuple(int(v) for v in np.clip(base, 0, 255))
            dark = tuple(int(v) for v in np.clip(base - 45, 0, 255))
            if (r + c) % 2 == 0:
                poly = [(x0, y0 + rh), (x0 + cw * 0.55, y0), (x0 + cw * 1.05, y0), (x0 + cw * 0.5, y0 + rh)]
            else:
                poly = [(x0, y0), (x0 + cw * 0.5, y0), (x0 + cw * 1.05, y0 + rh), (x0 + cw * 0.55, y0 + rh)]
            d.polygon(poly, fill=col, outline=dark)
    img = img.filter(ImageFilter.GaussianBlur(0.6))
    img.save(path)


def _coil_texture(path, seed, size=512):
    """Spiral coil weave of the floor."""
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:size, 0:size] / size - 0.5
    r = np.hypot(xx, yy)
    a = np.arctan2(yy, xx)
    rings = 0.5 + 0.5 * np.sin(2 * math.pi * (r * 22 + a / (2 * math.pi)))
    stitches = 0.5 + 0.5 * np.sin(a * 60 + r * 40)
    n = noise(size, size, 16, rng)
    shade = 0.78 + 0.14 * rings + 0.06 * stitches + 0.05 * (n - 0.5)
    base = np.array([190, 152, 108], float)
    img = np.clip(base[None, None] * shade[..., None], 0, 255).astype(np.uint8)
    Image.fromarray(img).save(path)
