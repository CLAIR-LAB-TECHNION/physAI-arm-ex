"""Drink can (red-can).

items.yaml keys: diameter, height, mass, color, stable_poses (upright, lying).
"""

import math
import os

from PIL import Image, ImageDraw

from ..gazebo import meshgen as mg
from ..gazebo.textures import font, rgb
from .base import EIGHT_YAWS, TIP_CLEARANCE, Grasp, ItemModel, RestPose, grasp_height, round_prism, \
    round_footprint, rect


class Can(ItemModel):
    def build(self, s):
        d, h = s["diameter"], s["height"]
        self.shapes = round_prism(d, h)
        self.poses["upright"] = RestPose(
            "upright", (0, 0, 0), h / 2, h, round_footprint(d),
            [Grasp(d, grasp_height(h, d), yaws=EIGHT_YAWS)])
        # Lying on its side: the can axis ends up along the rested-frame y
        # axis and the pads close across it.
        self.poses["lying"] = RestPose(
            "lying", (math.pi / 2, 0, 0), d / 2, d, rect(d, h),
            [Grasp(d, TIP_CLEARANCE, yaws=[0.0, math.pi])])

    def visual(self, texture_dir):
        s = self.spec
        r, h = s["diameter"] / 2, s["height"]
        z0, z1 = -h / 2, h / 2
        tex = f"{self.model}_label.png"
        _label_texture(os.path.join(texture_dir, tex), s["color"])
        label = mg.Material(self.model + "_label", (1, 1, 1), texture=tex, specular=0.5, shininess=50)
        metal = mg.Material(self.model + "_silver", (0.72, 0.73, 0.75), specular=0.6, shininess=60)
        return [
            mg.lathe([(0.0, z0 + 0.007), (r * 0.62, z0 + 0.003), (r * 0.82, z0), (r * 0.97, z0 + 0.004),
                      (r, z0 + 0.008)], metal),
            mg.lathe([(r, z0 + 0.008), (r, z1 - 0.016)], label),
            mg.lathe([(r, z1 - 0.016), (r * 0.93, z1 - 0.007), (r * 0.86, z1 - 0.001), (r * 0.85, z1)], metal),
            mg.lathe([(r * 0.85, z1), (r * 0.80, z1 - 0.004), (0.0, z1 - 0.004)], metal),
            mg.box((0.018, 0.010, 0.0015), metal, center=(r * 0.25, 0, z1 - 0.003)),  # pull tab
        ]


def _label_texture(path, color):
    W, H = 1024, 512
    img = Image.new("RGB", (W, H), rgb(color))
    d = ImageDraw.Draw(img)
    # A white ribbon wave running around the can.
    pts = [(x, H * 0.70 + 28 * math.sin(2 * math.pi * x / W * 2)) for x in range(0, W + 1, 8)]
    pts2 = [(x, y + 18) for x, y in pts[::-1]]
    d.polygon(pts + pts2, fill=(245, 245, 245))
    script = font(["DejaVuSerif-BoldItalic.ttf", "DejaVuSerifCondensed-BoldItalic.ttf"], 120)
    small = font(["DejaVuSans-Bold.ttf", "FreeSansBold.ttf"], 42)
    for x0 in (60, 60 + W // 2):
        d.text((x0, H * 0.28), "Coca-Cola", font=script, fill=(250, 250, 250))
        d.text((x0 + 40, H * 0.12), "CLASSIC", font=small, fill=(250, 250, 250))
    img.save(path)
