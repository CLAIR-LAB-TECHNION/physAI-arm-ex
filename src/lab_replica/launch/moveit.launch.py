"""MoveIt's move_group for the lab robot: planning only (the trajectories are
executed by execution.executor.Executor, as before).

    ros2 launch lab_replica moveit.launch.py [use_sim_time:=true] [rviz:=true]

The consoles start it themselves (moveit_client.py, MoveIt.start) when no
move_group answers. The planning scene starts with the table and the wall
(cell.yaml); the consoles add the rest (exercise/motion_planning.py scene_obstacles).

Services used (moveit_msgs): /plan_kinematic_path, /compute_cartesian_path,
/compute_ik, /apply_planning_scene, /check_state_validity.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from lab_replica import config


def _yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _setup(context):
    import xacro

    share = get_package_share_directory("lab_replica")
    moveit = os.path.join(share, "config", "moveit")
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    sim_time = arg("use_sim_time").lower() == "true"
    description = xacro.process_file(os.path.join(share, "description", "urdf", "lab_robot.urdf.xacro"),
                                     mappings={"ur_type": config.load_cell()["robot"]["ur_type"]}).toxml()
    with open(os.path.join(moveit, "lab_robot.srdf")) as f:
        semantic = f.read()
    params = {
        "robot_description": description,
        "robot_description_semantic": semantic,
        "robot_description_kinematics": _yaml(os.path.join(moveit, "kinematics.yaml")),
        "robot_description_planning": _yaml(os.path.join(moveit, "joint_limits.yaml")),
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": " ".join([
                "default_planner_request_adapters/AddTimeOptimalParameterization",
                "default_planner_request_adapters/FixWorkspaceBounds",
                "default_planner_request_adapters/FixStartStateBounds",
                "default_planner_request_adapters/FixStartStateCollision",
                "default_planner_request_adapters/FixStartStatePathConstraints"]),
            "start_state_max_bounds_error": 0.1,
            **_yaml(os.path.join(moveit, "ompl_planning.yaml")),
        },
        # Planning only: no controllers (the consoles execute the trajectories).
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_manage_controllers": False,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "use_sim_time": sim_time,
    }
    nodes = [Node(package="moveit_ros_move_group", executable="move_group", output="screen", parameters=[params])]
    nodes.append(Node(package="rviz2", executable="rviz2", output="log", condition=IfCondition(arg("rviz")),
                      arguments=["-d", os.path.join(share, "config", "lab.rviz")],
                      parameters=[{k: params[k] for k in ("robot_description", "robot_description_semantic",
                                                          "robot_description_kinematics")},
                                  {"use_sim_time": sim_time}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
