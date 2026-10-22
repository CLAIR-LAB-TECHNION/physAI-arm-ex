"""Mug with a handle (green-mug).

items.yaml keys: top_diameter, bottom_diameter, height, wall, handle_reach
(how far the handle sticks out), handle_height, handle_thickness, mass,
color, handle_color, stable_poses (upright).

The item frame's x axis points towards the handle. The mug is grasped
across the body, perpendicular to the handle.
"""

import math

import numpy as np

from ..gazebo import meshgen as mg
from .base import Grasp, ItemModel, RestPose, Shape, circle, convex_hull, grasp_height, round_prism

HANDLE_Z = 0.004   # handle centre above the mug's mid-height


class Mug(ItemModel):
    def build(self, s):
        rt, rb, h = s["top_diameter"] / 2, s["bottom_diameter"] / 2, s["height"]
        reach, hh, ht = s["handle_reach"], s["handle_height"], s["handle_thickness"]
        r = (rt + rb) / 2
        self.shapes = round_prism(2 * r, h) + [
            Shape("box", (reach + 0.004, ht, hh), (r + reach / 2 - 0.002, 0, HANDLE_Z)),
        ]
        footprint = np.vstack([circle(rt, 20), [[rt + reach, -ht], [rt + reach, ht]]])
        self.poses["upright"] = RestPose(
            "upright", (0, 0, 0), h / 2, h, convex_hull(footprint),
            [Grasp(2 * r, grasp_height(h, 2 * r), yaws=[math.pi / 2, -math.pi / 2])])

    def visual(self, texture_dir):
        s = self.spec
        rt, rb, h = s["top_diameter"] / 2, s["bottom_diameter"] / 2, s["height"]
        wall = s["wall"]
        reach, hh, ht = s["handle_reach"], s["handle_height"], s["handle_thickness"]
        z0, z1 = -h / 2, h / 2
        body = mg.Material(self.model + "_body", s["color"], specular=0.3, shininess=30)
        handle = mg.Material(self.model + "_handle", s["handle_color"], specular=0.4, shininess=40)
        meshes = [
            mg.lathe([(0.0, z0), (rb, z0)], body),
            mg.lathe([(rb, z0), (rt, z1)], body),
            mg.lathe([(rt, z1), (rt - wall, z1)], body),
            mg.lathe([(rt - wall, z1), (rb - wall, z0 + 0.006)], body),
            mg.lathe([(rb - wall, z0 + 0.006), (0.0, z0 + 0.006)], body),
        ]
        # Handle: a half ellipse swept with a round cross-section.
        r = (rt + rb) / 2
        t = np.linspace(math.pi / 2, -math.pi / 2, 24)
        path = np.stack([r - 0.004 + reach * np.cos(t), np.zeros_like(t), HANDLE_Z + hh / 2 * np.sin(t)], axis=1)
        meshes.append(mg.tube(path, ht / 2, handle, sides=14))
        return meshes
