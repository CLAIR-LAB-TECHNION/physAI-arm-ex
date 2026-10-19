"""A static, visual-only copy of the robot in a given configuration, for
rendering problem previews without running the controllers."""

import xml.etree.ElementTree as ET

from ..robot.collision import RobotPoints, resolve_uri
from ..transforms import make_tf, matrix_to_rpy, rpy_to_matrix


def robot_visual_sdf(urdf_text, q_arm, width, name="robot_preview"):
    rp = RobotPoints(urdf_text, max_mesh_points=1)
    T = rp.frames(q_arm, width)
    root = ET.fromstring(urdf_text)
    mats = {m.get("name"): m.find("color").get("rgba") for m in root.iter("material")
            if m.find("color") is not None and m.get("name")}
    vis = []
    for link in root.findall("link"):
        lname = link.get("name")
        if lname not in T:
            continue
        for k, v in enumerate(link.findall("visual")):
            o = v.find("origin")
            xyz = [float(a) for a in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(a) for a in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            M = T[lname] @ make_tf(rpy_to_matrix(*rpy), xyz)
            g = v.find("geometry")[0]
            if g.tag == "mesh":
                geo = f"<mesh><uri>file://{resolve_uri(g.get('filename'))}</uri></mesh>"
            elif g.tag == "cylinder":
                geo = f"<cylinder><radius>{g.get('radius')}</radius><length>{g.get('length')}</length></cylinder>"
            elif g.tag == "box":
                geo = f"<box><size>{g.get('size')}</size></box>"
            else:
                continue
            mat = ""
            m = v.find("material")
            rgba = None
            if m is not None:
                c = m.find("color")
                rgba = c.get("rgba") if c is not None else mats.get(m.get("name"))
            if rgba and not (g.tag == "mesh" and g.get("filename").endswith(".dae")):
                r, gg, b, _ = rgba.split()
                mat = f"<material><ambient>{r} {gg} {b} 1</ambient><diffuse>{r} {gg} {b} 1</diffuse><specular>0.3 0.3 0.3 1</specular></material>"
            p = list(M[:3, 3]) + list(matrix_to_rpy(M[:3, :3]))
            vis.append(f'<visual name="{lname}_{k}"><pose>{" ".join(f"{x:.6f}" for x in p)}</pose>'
                       f"<geometry>{geo}</geometry>{mat}</visual>")
    return (f'<model name="{name}"><static>true</static><link name="link">' + "".join(vis) + "</link></model>")

