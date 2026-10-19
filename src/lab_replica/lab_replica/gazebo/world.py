"""Gazebo world files: the static cell (room, table, camera, lights, physics)
plus the basket and the items at given poses."""

import numpy as np

from ..objects.camera import camera_sdf
from ..planning.placement import placement_pose
from ..transforms import quat_to_matrix
from . import sdf

WORLD_NAME = "lab"

_FLOATING = ('<ignition-gui><property key="state" type="string">floating</property><property key="width" '
             'type="double">5</property><property key="height" type="double">5</property><property '
             'key="showTitleBar" type="bool">false</property></ignition-gui>')
GUI_PLUGINS = "".join(f"""
      <plugin filename="{f}" name="{n}">
        {_FLOATING}
      </plugin>""" for f, n in [
    ("EntityContextMenuPlugin", "Entity context menu"), ("GzSceneManager", "Scene Manager"),
    ("InteractiveViewControl", "Interactive view control"), ("CameraTracking", "Camera Tracking"),
    ("MarkerManager", "Marker manager"), ("SelectEntities", "Select Entities"),
    ("VisualizationCapabilities", "Visualization Capabilities")]) + """
      <plugin filename="WorldControl" name="World control">
        <ignition-gui><title>World control</title><property type="bool" key="showTitleBar">false</property><property type="bool" key="resizable">false</property><property type="double" key="height">72</property><property type="double" key="width">121</property><property type="double" key="z">1</property><property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="left" target="left"/><line own="bottom" target="bottom"/></anchors></ignition-gui>
        <play_pause>true</play_pause><step>true</step><start_paused>false</start_paused>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <ignition-gui><title>World stats</title><property type="bool" key="showTitleBar">false</property><property type="bool" key="resizable">false</property><property type="double" key="height">110</property><property type="double" key="width">290</property><property type="double" key="z">1</property><property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="right" target="right"/><line own="bottom" target="bottom"/></anchors></ignition-gui>
        <sim_time>true</sim_time><real_time>true</real_time><real_time_factor>true</real_time_factor><iterations>true</iterations>
      </plugin>"""


def world_sdf(cell, body, world_name=WORLD_NAME, step=0.001, rtf=1.0, sensors=True, save_dir=None, gui=True):
    """A complete world: physics, lights, room, table, camera and `body`
    (the <include>s of the basket and the items).

    rtf: real-time factor (0: as fast as possible). sensors: render the
    camera. save_dir: the camera also saves its frames there. gui: the GUI
    starts with the lab camera's view."""
    c = cell["camera"]
    sensors_plugin = (
        '<plugin filename="ignition-gazebo-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>'
        if sensors
        else ""
    )
    camera = camera_sdf(cell, save_dir) if sensors else ""
    gui_block = ""
    if gui:
        gui_block = f"""
    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <ignition-gui><title>3D View</title><property type="bool" key="showTitleBar">false</property><property type="string" key="state">docked</property></ignition-gui>
        <engine>ogre2</engine><scene>scene</scene>
        <ambient_light>0.5 0.5 0.5</ambient_light><background_color>0.85 0.88 0.9</background_color>
        <camera_pose>{sdf.fmt(*c["xyz"], *c["rpy"])}</camera_pose>
        <camera_clip><near>0.03</near><far>100</far></camera_clip>
      </plugin>{GUI_PLUGINS}
    </gui>"""
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="{world_name}">
    <physics name="1ms" type="ignored">
      <max_step_size>{step}</max_step_size>
      <real_time_factor>{rtf}</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    {sensors_plugin}
    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>0.42 0.42 0.42 1</ambient>
      <background>0.85 0.88 0.9 1</background>
      <shadows>true</shadows>
    </scene>{gui_block}
    <light type="directional" name="ceiling">
      <pose>0.3 -0.5 3 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <diffuse>0.55 0.55 0.53 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>0.15 0.35 -0.92</direction>
    </light>
    <light type="point" name="fill">
      <pose>0.4 -1.2 1.4 0 0 0</pose>
      <cast_shadows>false</cast_shadows>
      <diffuse>0.25 0.25 0.25 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <attenuation><range>8</range><constant>0.6</constant><linear>0.05</linear><quadratic>0.0</quadratic></attenuation>
    </light>
    {sdf.include("lab_room", "lab_room", (0, 0, 0), static=True)}
    {sdf.include("lab_table", "lab_table", (0, 0, 0), static=True)}
    {camera}
{body}
  </world>
</sdf>
"""


def basket_include(cell, xy=None):
    x, y = xy if xy is not None else cell["basket"]["center_xy"]
    return sdf.include("basket", "basket", (x, y, cell["table"]["surface_z"]))


def problem_world(cell, items, placements=None, poses=None, basket_xy=None, lift=0.0005, **kwargs):
    """World text with the basket and the items.

    placements: list of planning.placement.Placement (ideal resting poses,
                lifted by `lift` so they drop into contact), or
    poses:      {item: (xyz, quat_xyzw)} exact poses (e.g. after settling).
    Other arguments go to world_sdf."""
    body = basket_include(cell, basket_xy)
    for p in placements or []:
        item = items[p.item]
        xyz, R = placement_pose(item, p)
        body += sdf.include(item.model, item.model, xyz + np.array([0, 0, lift]), R=R)
    for name, (xyz, q) in (poses or {}).items():
        item = items[name]
        body += sdf.include(item.model, item.model, np.asarray(xyz), R=quat_to_matrix(q))
    return world_sdf(cell, body, **kwargs)
