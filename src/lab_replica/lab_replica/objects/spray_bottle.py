"""Spray bottle with a trigger head (red-spray-bottle).

items.yaml keys: body_width, body_depth, body_height (the body tapers towards
the neck), neck_diameter, neck_height, head_width, head_length, head_height,
head_offset (head centre ahead of the body axis, towards the nozzle), mass,
color, head_color, stable_poses (upright).

The item frame's x axis points along the nozzle. The bottle is grasped
across its trigger head, as in the lab photos.
"""

import math

import numpy as np

from ..gazebo import meshgen as mg
from .base import Grasp, ItemModel, RestPose, Shape, rect, top_grasp_height


class SprayBottle(ItemModel):
    def build(self, s):
        bw, bd, bh = s["body_width"], s["body_depth"], s["body_height"]
        nd, nh = s["neck_diameter"], s["neck_height"]
        hw, hl, hh = s["head_width"], s["head_length"], s["head_height"]
        ho = s["head_offset"]
        H = bh + nh + hh
        z0 = -H / 2
        # The tapering body as three stacked boxes, then neck and head.
        self.shapes = [
            Shape("box", (bw * 0.97, bd, bh * 0.45), (0, 0, z0 + bh * 0.225)),
            Shape("box", (bw * 0.80, bd * 0.92, bh * 0.38), (0, 0, z0 + bh * 0.64)),
            Shape("box", (bw * 0.45, bd * 0.60, bh * 0.18), (0, 0, z0 + bh * 0.91)),
            Shape("cylinder", (nd / 2, nh + 0.004), (0, 0, z0 + bh + nh / 2)),
            Shape("box", (hl, hw, hh), (ho, 0, z0 + bh + nh + hh / 2)),
        ]
        self.poses["upright"] = RestPose(
            "upright", (0, 0, 0), H / 2, H, rect(bw, bd),
            # Grip the trigger head across its narrow side.
            [Grasp(hw, top_grasp_height(H, hw), yaws=[math.pi / 2, -math.pi / 2], center=(ho, 0.0))])

    def visual(self, texture_dir):
        s = self.spec
        bw, bd, bh = s["body_width"], s["body_depth"], s["body_height"]
        nd, nh = s["neck_diameter"], s["neck_height"]
        hw, hl, hh = s["head_width"], s["head_length"], s["head_height"]
        ho = s["head_offset"]
        H = bh + nh + hh
        z0 = -H / 2
        body = mg.Material(self.model + "_body", s["color"], specular=0.35, shininess=30)
        head = mg.Material(self.model + "_head", s["head_color"], specular=0.25, shininess=15)
        black = mg.Material(self.model + "_black", (0.05, 0.05, 0.05))
        # Body cross-sections: (height fraction, width scale, depth scale).
        profile = [(0.0, 0.97, 0.97), (0.04, 1.00, 1.00), (0.30, 0.96, 0.98), (0.55, 0.86, 0.94),
                   (0.75, 0.76, 0.88), (0.87, 0.58, 0.72), (0.95, 0.38, 0.52), (1.00, 0.20, 0.36)]
        sections = []
        for t, ka, kb in profile:
            n = 4.0 if t < 0.85 else 2.4
            sections.append(mg.superellipse(bw / 2 * ka, bd / 2 * kb, z0 + bh * t, n=n, points=48))
        sections[-1] = mg.superellipse(nd / 2, nd / 2, z0 + bh, n=2.0, points=48)
        meshes = [mg.loft(sections, body), mg.cap(sections[0], body, up=False)]
        zn = z0 + bh
        meshes.append(mg.lathe([(nd / 2, zn), (nd / 2, zn + nh)], head))
        zh = zn + nh
        meshes.append(mg.box((hl, hw, hh), head, center=(ho, 0, zh + hh / 2)))
        meshes.append(mg.box((0.012, 0.014, 0.012), black, center=(ho + hl / 2 + 0.005, 0, zh + hh * 0.72)))  # nozzle
        # Trigger hanging below the front of the head.
        trigger = mg.box((0.010, 0.020, 0.045), black)
        a = 0.35
        R = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
        meshes.append(trigger.transformed(R, (ho + hl / 2 - 0.016, 0, zh - 0.012)))
        return meshes
