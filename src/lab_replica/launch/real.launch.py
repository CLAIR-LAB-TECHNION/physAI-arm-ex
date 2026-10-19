"""The real cell: the UR arm through ur_robot_driver, the RG6 through the
OnRobot URCap already installed on the robot.

    ros2 launch lab_replica real.launch.py

Settings come from config/real_robot.yaml (robot IP, headless mode,
controller). With headless_mode false, press Play on the pendant's External
Control program once the driver is up. Then run a solution exactly as in
simulation:
    ros2 run lab_replica run_solution.py all_on_table --speed-scale 0.3 --step

Arguments:
  robot_ip           override real_robot.yaml's ip for this run
  rviz               start RViz (default false)
  use_fake_hardware  true: a mock arm, no robot connection, for testing the
                     commands offline. Use it with robot_ip:=127.0.0.1
                     gripper_port:=41499 and, in another terminal,
                     python3 src/lab_replica/tools/fake_onrobot_xmlrpc.py --port 41499
  gripper_port       override real_robot.yaml's gripper_port
  ur_driver          true: also start the UR driver here; false: only the RG6
                     driver, the UR driver runs on another computer
                     (default: real_robot.yaml's ur_driver)

Set the payload (RG6 + Quick Changer, 1.31 kg on the lab robot) in the
robot's installation on the pendant.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from lab_replica import config


def _setup(context):
    real = config.load_real_robot()
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    robot_ip = arg("robot_ip") or str(real["ip"])
    fake = arg("use_fake_hardware").lower() == "true"
    # The mock hardware has no passthrough interface.
    controller = "scaled_joint_trajectory_controller" if fake else real["controller"]
    ur_type = config.load_cell()["robot"]["ur_type"]
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ur_robot_driver"), "launch", "ur_control.launch.py")),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "headless_mode": str(real["headless_mode"]).lower(),
            "initial_joint_controller": controller,
            "verify_robot_model": "true",
            "launch_rviz": arg("rviz"),
            "use_fake_hardware": str(fake).lower(),
        }.items(),
    )
    gripper = Node(
        package="lab_replica", executable="rg6_driver.py", output="screen",
        parameters=[{"backend": "onrobot", "ip": robot_ip, "port": int(arg("gripper_port") or real["gripper_port"]),
                     "tool_index": 0}],
        # The UR driver owns /joint_states; publish the finger joints next to it.
        remappings=[("/joint_states", "/rg6/joint_states")],
    )
    ur_driver = arg("ur_driver") or str(real.get("ur_driver", True))
    return [driver, gripper] if ur_driver.lower() == "true" else [gripper]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value=""),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("gripper_port", default_value=""),
        DeclareLaunchArgument("ur_driver", default_value=""),
        OpaqueFunction(function=_setup),
    ])
