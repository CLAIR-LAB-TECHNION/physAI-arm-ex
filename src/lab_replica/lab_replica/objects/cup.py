"""Cup: straight lower part, flared upper part (blue-cup-with-yellow-stripe).

items.yaml keys: top_diameter, waist_diameter (straight lower part),
waist_height (where the flare starts), height, stripe (height range of the
coloured band), wall, mass, color, stripe_color, stable_poses (upright).
"""

from ..gazebo import meshgen as mg
from .base import EIGHT_YAWS, Grasp, ItemModel, RestPose, round_prism, round_footprint, top_grasp_height


class Cup(ItemModel):
    def build(self, s):
        rt, rw = s["top_diameter"] / 2, s["waist_diameter"] / 2
        zw, h = s["waist_height"], s["height"]
        z0 = -h / 2
        # Straight lower part, then the flare as two stepped solid prisms
        # (radii at 1/4 and 3/4 of the flare).
        hu = (h - zw) / 2
        r1 = rw + (rt - rw) * 0.25
        r2 = rw + (rt - rw) * 0.75
        self.shapes = (round_prism(2 * rw, zw, z0 + zw / 2)
                       + round_prism(2 * r1, hu, z0 + zw + hu / 2, base_disk=False)
                       + round_prism(2 * r2, hu, z0 + zw + 1.5 * hu, base_disk=False))
        self.poses["upright"] = RestPose(
            "upright", (0, 0, 0), h / 2, h, round_footprint(s["top_diameter"]),
            # The cup is taller than the RG6 fingers are long, so it is held
            # near the top, where it is widest.
            [Grasp(2 * r2, top_grasp_height(h, 2 * r2), yaws=EIGHT_YAWS)])

    def visual(self, texture_dir):
        s = self.spec
        rt, rw = s["top_diameter"] / 2, s["waist_diameter"] / 2
        zw, h, wall = s["waist_height"], s["height"], s["wall"]
        st0, st1 = s["stripe"]
        z0 = -h / 2
        body = mg.Material(self.model + "_body", s["color"], specular=0.3, shininess=30)
        stripe = mg.Material(self.model + "_stripe", s["stripe_color"], specular=0.2)
        return [
            mg.lathe([(0.0, z0), (rw, z0)], body),
            mg.lathe([(rw, z0), (rw, z0 + st0)], body),
            mg.lathe([(rw + 0.0005, z0 + st0), (rw + 0.0005, z0 + st1)], stripe),
            mg.lathe([(rw, z0 + st1), (rw, z0 + zw)], body),
            mg.lathe([(rw, z0 + zw), (rt, z0 + h)], body),
            mg.lathe([(rt, z0 + h), (rt - wall, z0 + h)], body),
            mg.lathe([(rt - wall, z0 + h), (rw - wall, z0 + zw), (rw - wall, z0 + 0.006)], body),
            mg.lathe([(rw - wall, z0 + 0.006), (0.0, z0 + 0.006)], body),
        ]
