"""SDF snippets shared by the model and world files."""

from ..transforms import matrix_to_rpy

# Contact properties. DART (Gazebo's default engine) uses the smaller friction
# coefficient of the two touching surfaces.
OBJECT_MU = 0.9
TABLE_MU = 0.6


def fmt(*vals):
    return " ".join(f"{v:.6g}" for v in vals)


def pose(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    return f"<pose>{fmt(*xyz, *rpy)}</pose>"


def geometry(shape):
    """<box> or <cylinder> of an objects.base.Shape."""
    if shape.kind == "box":
        return f"<box><size>{fmt(*shape.size)}</size></box>"
    r, h = shape.size
    return f"<cylinder><radius>{r:.6g}</radius><length>{h:.6g}</length></cylinder>"


def surface(mu):
    return (
        "<surface><friction><ode>"
        f"<mu>{mu}</mu><mu2>{mu}</mu2>"
        "</ode></friction>"
        "<contact><ode><min_depth>0.0005</min_depth><max_vel>0.1</max_vel></ode></contact>"
        "</surface>"
    )


def material(color, specular=0.1):
    r, g, b = color[:3]
    return (
        f"<material><ambient>{fmt(r, g, b)} 1</ambient><diffuse>{fmt(r, g, b)} 1</diffuse>"
        f"<specular>{fmt(specular, specular, specular)} 1</specular></material>"
    )


def inertial(mass, com, inertia):
    I = inertia
    return (
        f"<inertial>{pose(com)}<mass>{mass:.6g}</mass><inertia>"
        f"<ixx>{I[0, 0]:.6g}</ixx><ixy>{I[0, 1]:.6g}</ixy><ixz>{I[0, 2]:.6g}</ixz>"
        f"<iyy>{I[1, 1]:.6g}</iyy><iyz>{I[1, 2]:.6g}</iyz><izz>{I[2, 2]:.6g}</izz>"
        "</inertia></inertial>"
    )


def dynamic_model(model_name, shapes, mass, com, inertia, mesh_uri, mu=OBJECT_MU):
    """model.sdf of a movable object: one link, collision shapes and a visual mesh."""
    cols = "\n".join(
        f'      <collision name="c{k}">{pose(s.xyz, s.rpy)}<geometry>{geometry(s)}</geometry>{surface(mu)}</collision>'
        for k, s in enumerate(shapes)
    )
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <model name="{model_name}">
    <link name="link">
      {inertial(mass, com, inertia)}
{cols}
      <visual name="visual">
        <geometry><mesh><uri>{mesh_uri}</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


def static_model(model_name, body):
    """model.sdf of a static model whose link holds `body`."""
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
{body}    </link>
  </model>
</sdf>
"""


def box(name, size, xyz, color, collide=True, specular=0.1, mu=TABLE_MU, emissive=None):
    """A box visual (and collision) for a static model's link."""
    vis_mat = material(color, specular)
    if emissive is not None:
        vis_mat = vis_mat.replace("</material>", f"<emissive>{fmt(*emissive)} 1</emissive></material>")
    col = (
        f'<collision name="{name}_c"><pose>{fmt(*xyz)} 0 0 0</pose><geometry><box><size>{fmt(*size)}</size></box></geometry>{surface(mu)}</collision>'
        if collide
        else ""
    )
    return (
        f'{col}<visual name="{name}_v"><pose>{fmt(*xyz)} 0 0 0</pose>'
        f"<geometry><box><size>{fmt(*size)}</size></box></geometry>{vis_mat}</visual>\n"
    )


def model_config(name, description):
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author><name>lab_replica</name></author>
  <description>{description}</description>
</model>
"""


def include(model, name, xyz, R=None, rpy=None, static=False):
    """<include> of model://<model> at a pose (rotation matrix R or rpy)."""
    if R is not None:
        rpy = matrix_to_rpy(R)
    rpy = rpy or (0, 0, 0)
    st = "<static>true</static>" if static else ""
    return f"""
    <include>
      <uri>model://{model}</uri>
      <name>{name}</name>
      {pose(xyz, rpy)}{st}
    </include>"""
