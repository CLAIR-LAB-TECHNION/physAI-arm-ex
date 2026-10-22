"""Perception with the wrist RealSense: what is where on the table.

    rgbd.py      frames from the RealSense topics; pixels to world points
    sam3.py      SAM 3: item names in, instance masks out
    estimate.py  an item's placement (support, resting pose, x, y, yaw) from
                 its 3D points, fitted with the item's model; the basket
    scene.py     all items in one or several frames (assignment of masks to
                 items, fusion of views); drawing the detections

Settings (topics, views, prompts, thresholds): config/perception.yaml.
"""
