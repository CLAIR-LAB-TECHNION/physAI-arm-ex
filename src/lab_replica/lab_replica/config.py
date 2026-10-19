"""Where the package's files are, and loading the configuration files.

    config/cell.yaml        measurements of the cell (table, robot mount, basket, camera, wall)
    config/items.yaml       measurements of the twelve items
    config/real_robot.yaml  the real robot's address and driver settings
"""

import os

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_PACKAGE = os.path.dirname(_HERE)   # src/lab_replica when running from the source tree


def package_root():
    """The package directory holding config/, models/, description/ ...

    The installed share directory when the package is built, else the
    source tree, so the scripts also run straight from src/.
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("lab_replica")
        if os.path.isdir(os.path.join(share, "config")):
            return share
    except Exception:
        pass
    return _SRC_PACKAGE


def path(*parts):
    """Path of a file in the package, e.g. path("config", "cell.yaml")."""
    return os.path.join(package_root(), *parts)


def workspace_root():
    """The lab-replica directory (it holds src/ and problems/)."""
    if os.environ.get("LAB_REPLICA_WS"):
        return os.environ["LAB_REPLICA_WS"]
    for pkg in (_SRC_PACKAGE, os.path.realpath(package_root())):
        # <ws>/src/lab_replica, or <ws>/install/lab_replica/share/lab_replica
        for ws in (os.path.dirname(os.path.dirname(pkg)), os.path.join(pkg, "..", "..", "..", "..")):
            ws = os.path.abspath(ws)
            if os.path.isdir(os.path.join(ws, "src", "lab_replica")):
                return ws
    return os.getcwd()


PROBLEM_DIRS = ("problems", "problems_tamp", "problems_gen")


def problem_dir(name_or_path):
    """A problem directory, given its name (looked up in <workspace>/problems,
    problems_tamp (tasks of solve.py), problems_gen) or a path."""
    p = os.path.abspath(os.path.expanduser(name_or_path))
    if os.path.isdir(p):
        return p
    for d in PROBLEM_DIRS:
        p = os.path.join(workspace_root(), d, name_or_path)
        if os.path.isdir(p):
            return p
    raise FileNotFoundError(f"no problem directory {name_or_path!r} (looked in "
                            f"{', '.join(os.path.join(workspace_root(), d) for d in PROBLEM_DIRS)})")


def _load(name):
    with open(path("config", name)) as f:
        return yaml.safe_load(f)


def load_cell():
    """config/cell.yaml as a dict (sections table, robot, gripper, basket, camera, room, placement)."""
    return _load("cell.yaml")


def load_item_specs():
    """config/items.yaml as a dict {PDDL item name: measurements}."""
    return _load("items.yaml")


def load_real_robot():
    """config/real_robot.yaml as a dict."""
    return _load("real_robot.yaml")


def model_name(item):
    """Gazebo model name of a PDDL item (red-can -> red_can)."""
    return item.replace("-", "_")
