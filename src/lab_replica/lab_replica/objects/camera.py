"""The lab's RGB camera: its Gazebo sensor and a pinhole model of it.

cell.yaml section `camera`: xyz, rpy (Gazebo camera frame: x forward, y left,
z up), width, height, horizontal_fov, update_rate. The simulated camera
publishes /lab_camera/image; it has no lens distortion.
"""

import math

import numpy as np

from ..gazebo import sdf
from ..transforms import rpy_to_matrix

TOPIC = "/lab_camera/image"


def camera_sdf(cell, save_dir=None, name="lab_camera"):
    """<model> text of the static camera. save_dir: also write every frame
    there as PNG (used for rendering previews)."""
    c = cell["camera"]
    save = f'<save enabled="true"><path>{save_dir}</path></save>' if save_dir else ""
    return f"""
    <model name="{name}">
      <static>true</static>
      {sdf.pose(c["xyz"], c["rpy"])}
      <link name="link">
        <sensor name="rgb" type="camera">
          <always_on>1</always_on>
          <update_rate>{c["update_rate"]}</update_rate>
          <topic>{TOPIC}</topic>
          <camera>
            <horizontal_fov>{c["horizontal_fov"]}</horizontal_fov>
            <image><width>{c["width"]}</width><height>{c["height"]}</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>20</far></clip>
            {save}
          </camera>
        </sensor>
      </link>
    </model>"""


class Camera:
    """Pinhole model: which world points the camera sees."""

    def __init__(self, cell):
        c = cell["camera"]
        self.W, self.H = c["width"], c["height"]
        self.f = (self.W / 2) / math.tan(c["horizontal_fov"] / 2)
        self.cx, self.cy = self.W / 2, self.H / 2
        self.t = np.array(c["xyz"], float)
        self.R = rpy_to_matrix(*c["rpy"])

    def project(self, p):
        """World point(s) -> pixel (u, v) and depth along the optical axis."""
        p = np.atleast_2d(p) - self.t
        q = p @ self.R  # into the camera frame
        z = q[:, 0]
        u = self.cx - self.f * q[:, 1] / z
        v = self.cy - self.f * q[:, 2] / z
        return np.stack([u, v], axis=1), z

    def visible(self, p, margin=10):
        """True if all points are in front of the camera and inside the image."""
        uv, z = self.project(p)
        return bool(np.all((z > 0) & (uv[:, 0] > margin) & (uv[:, 0] < self.W - margin)
                           & (uv[:, 1] > margin) & (uv[:, 1] < self.H - margin)))
