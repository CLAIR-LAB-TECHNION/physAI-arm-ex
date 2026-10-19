"""The simulated cell: Gazebo, the UR arm with the RG6, controllers, camera.

    ros2 launch lab_replica sim.launch.py problem:=all_on_table
    ros2 launch lab_replica sim.launch.py                  # the empty cell

Arguments:
  problem        a problem name in problems/ or a problem directory (loads its world.sdf)
  world          any world file instead (default: the empty cell)
  gui            start the Gazebo GUI (default true; skipped without a display)
  rviz           start RViz (default false)
  vgl            run the GUI and RViz through VirtualGL on the GPU (default
                 auto: when /opt/VirtualGL/bin/vglrun exists). Over a remote
                 desktop (xrdp) the display only has software OpenGL, which
                 costs about two CPU cores for the GUI and slows the physics.
  camera_bridge  bridge /lab_camera/image to ROS (default true)
  wrist_camera   bridge the wrist RealSense's images to ROS (default true;
                 false: start them separately with camera.launch.py sim:=true,
                 as run/1_robot.sh and run/2_camera.sh do)
  wrist_camera_rate  update rate of the wrist RealSense [Hz] (default: cell.yaml)

Once up (the same interface as real.launch.py):
  /joint_trajectory_controller/follow_joint_trajectory   arm
  /rg6/gripper_command                                    gripper (control_msgs/GripperCommand)
  /lab_camera/image                                       the lab camera (fixed, in front of the table)
  /camera/camera/color/image_raw                          wrist RealSense colour (rgb8)
  /camera/camera/aligned_depth_to_color/image_raw         wrist RealSense depth aligned to colour
                                                          (32FC1 [m] here; the real driver sends 16UC1 [mm])
  /camera/camera/color/camera_info                        wrist RealSense intrinsics
  /lab/model_poses                                        object poses (tf2_msgs/TFMessage)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from lab_replica import config

VGLRUN = "/opt/VirtualGL/bin/vglrun"


def wrist_camera_bridge(output="log"):
    """The simulated wrist RealSense under the topic names of the realsense2_camera driver."""
    rs = "/camera/camera"
    return Node(package="ros_gz_bridge", executable="parameter_bridge", name="wrist_camera_bridge", output=output,
                arguments=["/wrist_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                           "/wrist_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
                           "/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo"],
                remappings=[("/wrist_camera/image", f"{rs}/color/image_raw"),
                            ("/wrist_camera/depth_image", f"{rs}/aligned_depth_to_color/image_raw"),
                            ("/wrist_camera/camera_info", f"{rs}/color/camera_info")],
                parameters=[{"use_sim_time": True}])


def _setup(context):
    share = get_package_share_directory("lab_replica")
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    if arg("problem"):
        world = os.path.join(config.problem_dir(arg("problem")), "world.sdf")
    else:
        world = os.path.abspath(os.path.expanduser(arg("world") or config.path("worlds", "lab_empty.sdf")))
    if not os.path.isfile(world):
        raise RuntimeError(f"world file not found: {world}")
    vgl = arg("vgl").lower()
    vgl = vgl == "true" or (vgl == "auto" and os.access(VGLRUN, os.X_OK))
    gl_prefix = [VGLRUN, "-d", "egl"] if vgl else []

    import xacro
    robot_description = xacro.process_file(
        os.path.join(share, "description", "urdf", "lab_robot.urdf.xacro"),
        mappings={"ur_type": config.load_cell()["robot"]["ur_type"], "sim_ignition": "true",
                  "simulation_controllers": os.path.join(share, "config", "controllers.yaml"),
                  "wrist_camera_rate": arg("wrist_camera_rate") or "0"}).toxml()

    # Gazebo finds the models in the package, the UR meshes in the ROS share,
    # and ign_ros2_control on LD_LIBRARY_PATH.
    ros_share = os.path.dirname(get_package_share_directory("ur_description"))
    resource = [config.path("models"), ros_share, os.path.dirname(share), os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")]
    plugins = [os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", ""), os.environ.get("LD_LIBRARY_PATH", "")]

    def spawner(controller):
        return Node(package="controller_manager", executable="spawner", output="log",
                    arguments=[controller, "-c", "/controller_manager", "--controller-manager-timeout", "120"])

    joint_states, arm, gripper = spawner("joint_state_broadcaster"), spawner("joint_trajectory_controller"), \
        spawner("rg6_controller")
    gripper_driver = Node(package="lab_replica", executable="rg6_driver.py", output="screen",
                          parameters=[{"use_sim_time": True, "backend": "sim"}])
    actions = [
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", ":".join(p for p in resource if p)),
        SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", ":".join(p for p in plugins if p)),
        # The server always renders the camera through EGL on the GPU.
        ExecuteProcess(cmd=["ign", "gazebo", "-s", "-r", "-v", "2", "--headless-rendering", world], output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="log",
             parameters=[{"use_sim_time": True, "robot_description": robot_description}]),
        Node(package="ros_gz_sim", executable="create", output="log",
             arguments=["-string", robot_description, "-name", "ur", "-allow_renaming", "true"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="lab_bridge", output="log",
             arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                        "/world/lab/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"],
             remappings=[("/world/lab/dynamic_pose/info", "/lab/model_poses")]),
        joint_states,
        RegisterEventHandler(OnProcessExit(target_action=joint_states, on_exit=[arm, gripper])),
        RegisterEventHandler(OnProcessExit(target_action=gripper, on_exit=[gripper_driver])),
        Node(package="rviz2", executable="rviz2", output="log", condition=IfCondition(arg("rviz")),
             prefix=" ".join(gl_prefix) or None,
             arguments=["-d", os.path.join(share, "config", "lab.rviz")], parameters=[{"use_sim_time": True}]),
    ]
    if arg("wrist_camera").lower() == "true":
        actions.append(wrist_camera_bridge())
    if arg("camera_bridge").lower() == "true":
        actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge", name="camera_bridge",
                            output="log", arguments=["/lab_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image"]))
    if arg("gui").lower() == "true" and os.environ.get("DISPLAY"):
        # A separate GUI process; it starts with the lab camera's view (set in the world file).
        actions.append(TimerAction(period=3.0, actions=[
            ExecuteProcess(cmd=gl_prefix + ["ign", "gazebo", "-g", "-v", "1"], output="log")]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("problem", default_value=""),
        DeclareLaunchArgument("world", default_value=""),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("vgl", default_value="auto"),
        DeclareLaunchArgument("camera_bridge", default_value="true"),
        DeclareLaunchArgument("wrist_camera", default_value="true"),
        DeclareLaunchArgument("wrist_camera_rate", default_value=""),
        OpaqueFunction(function=_setup),
    ])
