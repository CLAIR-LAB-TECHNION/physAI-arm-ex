"""Gazebo: model and world files, and headless Gazebo runs.

    build.py          writes the models in models/ from lab_replica.objects
    world.py          world files (the cell + basket + items at given poses)
    sdf.py            SDF snippets
    meshgen.py        procedural meshes and the OBJ writer
    textures.py       helpers for the procedural textures
    server.py         a headless Gazebo server (settling, rendering)
    robot_preview.py  a static copy of the robot, for rendered previews
"""
