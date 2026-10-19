"""The wrist RealSense: colour, and depth aligned to colour, on the topics of
config/perception.yaml (the realsense2_camera driver's names).

    ros2 launch lab_replica camera.launch.py               # the real camera (USB)
    ros2 launch lab_replica camera.launch.py sim:=true     # the simulated one (needs sim.launch.py wrist_camera:=false)

Arguments (real camera):
  color_profile  width,height,fps of the colour stream (default 1280,720,15)
  depth_profile  width,height,fps of the depth stream (default 1280,720,15)
  serial_no      which camera, when several are plugged in (default: any)

The skills take frames only while the arm stands still, so 15 fps is plenty
and keeps the USB link and the alignment light. The depth is aligned to the
colour image by the driver (align_depth.enable); the skills need that.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    if arg("sim").lower() == "true":
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "lab_sim_launch", os.path.join(os.path.dirname(os.path.realpath(__file__)), "sim.launch.py"))
        sim = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(sim)
        return [sim.wrist_camera_bridge(output="screen")]
    try:
        get_package_share_directory("realsense2_camera")
    except Exception as e:  # noqa: BLE001
        raise RuntimeError("the RealSense driver is not installed: "
                           "sudo apt install ros-humble-realsense2-camera") from e
    # The driver's node as its rs_launch.py starts it (that file would warn
    # about this file's own arguments, which it does not know).
    params = {
        "align_depth.enable": True,
        "enable_color": True,
        "enable_depth": True,
        "rgb_camera.color_profile": arg("color_profile"),
        "depth_module.depth_profile": arg("depth_profile"),
        "pointcloud.enable": False,
    }
    if arg("serial_no"):
        params["serial_no"] = ParameterValue(arg("serial_no"), value_type=str)
    return [Node(package="realsense2_camera", executable="realsense2_camera_node", namespace="camera",
                 name="camera", parameters=[params], output="screen", emulate_tty=True)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("color_profile", default_value="1280,720,15"),
        DeclareLaunchArgument("depth_profile", default_value="1280,720,15"),
        DeclareLaunchArgument("serial_no", default_value=""),
        OpaqueFunction(function=_setup),
    ])
