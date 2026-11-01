"""Write the Gazebo models (meshes, textures, model.sdf) from the object
models, and the empty-cell world."""

import os

from .. import config, objects
from ..objects.basket import FRICTION as BASKET_FRICTION
from ..objects.room import room_sdf
from ..objects.table import table_sdf
from . import meshgen, sdf
from .world import basket_include, world_sdf


def _write_model(root, name, sdf_text, description):
    d = os.path.join(root, name)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "model.sdf"), "w") as f:
        f.write(sdf_text)
    with open(os.path.join(d, "model.config"), "w") as f:
        f.write(sdf.model_config(name, description))


def _write_dynamic(root, model, shapes, mass, com, inertia, meshes_fn, description, **kw):
    mesh_dir = os.path.join(root, model, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)
    meshgen.write_obj(os.path.join(mesh_dir, model + ".obj"), meshes_fn(mesh_dir))
    _write_model(root, model, sdf.dynamic_model(model, shapes, mass, com, inertia,
                                                f"model://{model}/meshes/{model}.obj", **kw), description)


def build_models(out=None, log=print):
    """Rebuild every model in `out` (default: the package's models/) and
    worlds/lab_empty.sdf."""
    out = out or config.path("models")
    cell = config.load_cell()
    os.makedirs(out, exist_ok=True)
    for name, item in objects.load_items().items():
        _write_dynamic(out, item.model, item.shapes, item.mass, item.com, item.inertia, item.visual,
                       f"PDDL item {name}")
        log(f"built {item.model}: {len(item.shapes)} collision shapes, mass {item.mass} kg")
    basket = objects.load_basket(cell)
    _write_dynamic(out, "basket", basket.shapes, basket.mass, basket.com, basket.inertia, basket.visual,
                   "Woven seagrass basket", mu=BASKET_FRICTION)
    log(f"built basket: {len(basket.shapes)} collision shapes")
    _write_model(out, "lab_table", table_sdf(cell), "Lab table with its white board")
    _write_model(out, "lab_room", room_sdf(cell), "Wall, window sill and floor behind the table")
    log("built lab_table, lab_room")
    worlds = config.path("worlds")
    os.makedirs(worlds, exist_ok=True)
    with open(os.path.join(worlds, "lab_empty.sdf"), "w") as f:
        f.write(world_sdf(cell, basket_include(cell)))
    log("wrote worlds/lab_empty.sdf")
