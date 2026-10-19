"""Wooden cube (yellow-, orange-, blue-, red-, green-block).

items.yaml keys: size (edge length), mass, color, stable_poses (flat).
"""

import math
import os
import zlib

import numpy as np
from PIL import Image

from ..gazebo import meshgen as mg
from ..gazebo.textures import noise, rgb
from .base import TIP_CLEARANCE, Grasp, ItemModel, RestPose, Shape, rect


class Block(ItemModel):
    def build(self, s):
        a = s["size"]
        self.shapes = [Shape("box", (a, a, a))]
        self.poses["flat"] = RestPose(
            "flat", (0, 0, 0), a / 2, a, rect(a, a),
            [Grasp(a, TIP_CLEARANCE, yaws=[0.0, math.pi / 2, math.pi, -math.pi / 2])])

    def visual(self, texture_dir):
        a = self.spec["size"]
        tex = f"{self.model}_wood.png"
        _wood_texture(os.path.join(texture_dir, tex), self.spec["color"], seed=zlib.crc32(self.model.encode()) % 1000)
        wood = mg.Material(self.model + "_wood", (1, 1, 1), texture=tex, specular=0.08, shininess=8)
        return [mg.box((a, a, a), wood)]


def _wood_texture(path, color, seed):
    rng = np.random.default_rng(seed)
    H = W = 256
    base = np.array(rgb(color), float)
    y = np.linspace(0, 1, H)[:, None]
    grain = 0.5 + 0.5 * np.sin(2 * math.pi * (y * 18 + 0.6 * noise(H, W, 32, rng)))
    n = noise(H, W, 8, rng)
    shade = 0.86 + 0.10 * grain + 0.08 * (n - 0.5)
    img = np.clip(base[None, None, :] * shade[..., None], 0, 255).astype(np.uint8)
    Image.fromarray(img).save(path)
