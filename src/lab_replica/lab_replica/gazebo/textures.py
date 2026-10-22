"""Small helpers for the procedural textures of the object models."""

import os

import numpy as np
from PIL import Image, ImageFont

FONT_DIR = "/usr/share/fonts/truetype"


def font(names, size):
    """The first of the TrueType fonts `names` found on the system."""
    for n in names:
        for root, _, files in os.walk(FONT_DIR):
            if n in files:
                return ImageFont.truetype(os.path.join(root, n), size)
    return ImageFont.load_default()


def rgb(color):
    """[0, 1] colour -> 0-255 tuple."""
    return tuple(int(round(255 * v)) for v in color[:3])


def noise(h, w, scale, rng):
    """Smooth value noise in [0, 1]."""
    small = rng.random((max(2, h // scale), max(2, w // scale)))
    img = Image.fromarray((small * 255).astype(np.uint8)).resize((w, h), Image.BICUBIC)
    return np.asarray(img, float) / 255.0
