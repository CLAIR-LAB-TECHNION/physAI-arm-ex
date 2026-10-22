"""Models of every physical object in the cell, one file each.

    can.py, spray_bottle.py, tape_roll.py,   the twelve items, by `kind` in
    cup.py, mug.py, block.py                 config/items.yaml: collision
                                             shapes, resting poses, grasps,
                                             visual mesh
    measured.py                              items added in the action
                                             console for one session
    basket.py                                the basket
    table.py, room.py                        the table (with the robot's
                                             mounting plate) and the background
    camera.py                                the lab camera
    base.py                                  what the item kinds share

The numbers come from config/items.yaml and config/cell.yaml; these files
turn them into shapes. gazebo/build.py writes the Gazebo models from them.
The robot and gripper are described in description/urdf/.
"""

from .. import config
from .basket import Basket
from .block import Block
from .can import Can
from .cup import Cup
from .measured import Measured
from .mug import Mug
from .spray_bottle import SprayBottle
from .tape_roll import TapeRoll

# items.yaml `kind` -> model class
KINDS = {
    "can": Can,
    "spray_bottle": SprayBottle,
    "tape_roll": TapeRoll,
    "cup": Cup,
    "mug": Mug,
    "block": Block,
    "measured": Measured,
}


def make_item(name, spec):
    """The item model of one spec (an items.yaml entry)."""
    return KINDS[spec["kind"]](name, spec)


def load_items():
    """{PDDL item name: item model} for every item in config/items.yaml."""
    return {name: make_item(name, spec) for name, spec in config.load_item_specs().items()}


def load_basket(cell=None):
    return Basket((cell or config.load_cell())["basket"])
