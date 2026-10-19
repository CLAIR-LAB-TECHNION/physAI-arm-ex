"""Roll of tape lying flat (roll-of-black-tape, roll-of-green-tape, roll-of-blue-tape).

items.yaml keys: outer_diameter, inner_diameter, width (the roll's height
lying flat), core (optional: 'printed' for the duct tape, else a cardboard
core), mass, color, stable_poses (flat).
"""

import os

from PIL import Image, ImageDraw

from ..gazebo import meshgen as mg
from ..gazebo.textures import font, rgb
from .base import EIGHT_YAWS, TIP_CLEARANCE, Grasp, ItemModel, RestPose, round_prism, round_footprint


class TapeRoll(ItemModel):
    def build(self, s):
        d, w = s["outer_diameter"], s["width"]
        # Solid collision: nothing is ever put through the hole, and a ring of
        # boxes makes DART an order of magnitude slower.
        self.shapes = round_prism(d, w)
        self.poses["flat"] = RestPose(
            "flat", (0, 0, 0), w / 2, w, round_footprint(d),
            [Grasp(d, TIP_CLEARANCE, yaws=EIGHT_YAWS)])

    def visual(self, texture_dir):
        s = self.spec
        ro, ri, w = s["outer_diameter"] / 2, s["inner_diameter"] / 2, s["width"]
        rc = ri + 0.004   # cardboard core thickness
        col = s["color"]
        outer = mg.Material(self.model + "_outer", col, specular=0.25, shininess=25)
        side = mg.Material(self.model + "_side", tuple(min(1, c * 1.25 + 0.02) for c in col), specular=0.1)
        core_tex = f"{self.model}_core.png"
        if s.get("core") == "printed":
            _printed_core_texture(os.path.join(texture_dir, core_tex), col)
        else:
            _cardboard_core_texture(os.path.join(texture_dir, core_tex), col)
        core_in = mg.Material(self.model + "_core", (1, 1, 1), texture=core_tex, specular=0.05)
        core_edge = mg.Material(self.model + "_coreedge", (0.80, 0.78, 0.74), specular=0.05)
        return [
            mg.lathe([(ro, -w / 2), (ro, w / 2)], outer, segments=64),
            mg.disk(rc, ro, w / 2, side, segments=64, up=True),
            mg.disk(rc, ro, -w / 2, side, segments=64, up=False),
            mg.disk(ri, rc, w / 2, core_edge, segments=64, up=True),
            mg.disk(ri, rc, -w / 2, core_edge, segments=64, up=False),
            mg.lathe([(ri, w / 2), (ri, -w / 2)], core_in, segments=64),
        ]


def _cardboard_core_texture(path, accent):
    """White cardboard core with a repeated coloured label."""
    W, H = 512, 64
    img = Image.new("RGB", (W, H), (236, 236, 232))
    d = ImageDraw.Draw(img)
    f = font(["DejaVuSans-Bold.ttf"], 18)
    for k in range(8):
        x = k * 64
        d.rectangle([x + 4, 10, x + 26, H - 10], fill=rgb(accent))
        d.rectangle([x + 30, 14, x + 58, H - 14], fill=(30, 110, 170))
        d.text((x + 33, 20), "TP", font=f, fill=(240, 240, 240))
    img.save(path)


def _printed_core_texture(path, color):
    """Inside of the duct tape roll: its colour with white logo prints."""
    W, H = 512, 128
    img = Image.new("RGB", (W, H), rgb(color))
    d = ImageDraw.Draw(img)
    for k in range(10):
        x = 20 + k * 50
        y = 30 if k % 2 else 70
        d.polygon([(x, y), (x + 22, y - 6), (x + 18, y + 14), (x + 6, y + 18)], fill=(225, 230, 240))
    img.save(path)
