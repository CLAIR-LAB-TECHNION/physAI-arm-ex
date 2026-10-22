"""An item added in the action console (`new-item:`) for one session,
modelled by its outline measured with the wrist camera (SAM 3 mask + depth):
a box, or a round prism for round items. It is grasped with the same force
as every item (cell.yaml gripper.grip_force).

Its spec (made by autonomy/executive.py new_item; never written to a file):
  kind: measured
  shape: box | round
  size: [length, width, height] (box, length >= width) or [diameter, height] (round)
  mass, color, stable_poses ([flat] for a box, [upright] for a round item)
  prompts: SAM 3 text prompts (the first is the main name)
"""

import math

from ..gazebo import meshgen as mg
from ..robot import rg6
from .base import EIGHT_YAWS, Grasp, ItemModel, RestPose, Shape, grasp_height, rect, round_footprint, round_prism

GRASP_MARGIN = 0.01   # the planner skips grasps wider than rg6.MAX_WIDTH - this


class Measured(ItemModel):
    def build(self, s):
        if s["shape"] == "round":
            d, h = s["size"]
            self.shapes = round_prism(d, h)
            self.poses["upright"] = RestPose("upright", (0, 0, 0), h / 2, h, round_footprint(d),
                                             [Grasp(d, grasp_height(h, d), yaws=EIGHT_YAWS)])
            return
        length, width, h = s["size"]
        self.shapes = [Shape("box", (length, width, h))]
        # Across the short side (closing along y), and across the long side
        # when the gripper opens wide enough for it.
        grasps = [Grasp(width, grasp_height(h, width), yaws=[math.pi / 2, -math.pi / 2])]
        if length - width > 0.005 and length < rg6.MAX_WIDTH - GRASP_MARGIN:
            grasps.append(Grasp(length, grasp_height(h, length), yaws=[0.0, math.pi]))
        self.poses["flat"] = RestPose("flat", (0, 0, 0), h / 2, h, rect(length, width), grasps)

    def visual(self, texture_dir):
        s = self.spec
        mat = mg.Material(self.model + "_plain", s["color"], specular=0.1, shininess=10)
        if s["shape"] == "round":
            d, h = s["size"]
            r = d / 2
            return [mg.disk(0.0, r, -h / 2, mat, up=False), mg.lathe([(r, -h / 2), (r, h / 2)], mat),
                    mg.disk(0.0, r, h / 2, mat)]
        return [mg.box(tuple(s["size"]), mat)]
