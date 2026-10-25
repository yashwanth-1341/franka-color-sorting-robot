"""
Full colour pick-and-place demo in Gazebo:

    ros2 launch panda_bringup simulated_robot.launch.py

Every launch generates a new random layout of the red/green/blue boxes and the
obstacles on the table. Then pick a colour in the web UI at
http://localhost:8765, or from a terminal:

    ros2 topic pub --once /target_color std_msgs/String "{data: red}"

Arguments:
    seed:=<int>          reproduce a specific random layout
    num_obstacles:=<n>   number of obstacles on the table (default 6)
    target_color:=red    start picking this colour as soon as the robot is ready
    headless:=true|false run Gazebo without GUI (default: true when $DISPLAY is unset)
    rviz:=true|false     start RViz (default: false when headless)
    ui_port:=8765        port of the colour selection web UI
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from panda_pick_place.scene import write_scene


def running_simulations():
    """PIDs of Gazebo servers that are already running (e.g. an earlier launch
    in another terminal). Two simulations share the ROS topics and fight over
    the robot, so only one may run at a time."""
    pids = []
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmdline = f.read().replace(b"\0", b" ")
        except OSError:
            continue
        # gz is a ruby script that renames itself to "gz sim <world> ..."
        if cmdline.startswith(b"gz sim "):
            pids.append(int(pid))
    return pids


def launch_setup(context):
    running = running_simulations()
    if running:
        raise RuntimeError(
            "Another Gazebo simulation is already running (pid "
            + ", ".join(map(str, running))
            + "). Stop it first (Ctrl+C in its terminal, or `pkill -f 'gz sim'`), "
            "because two simulations share the same ROS topics and fight over the robot.")

    seed_arg = LaunchConfiguration("seed").perform(context)
    seed = int(seed_arg) if seed_arg else None
    num_obstacles = int(LaunchConfiguration("num_obstacles").perform(context))
    headless = LaunchConfiguration("headless").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context) or (
        "false" if headless.lower() == "true" else "true")

    # ------------------- Random scene -------------------
    base_world = os.path.join(get_package_share_directory("panda_description"), "worlds", "table.world")
    output_dir = os.path.join(tempfile.gettempdir(), "panda_pick_place")
    world_path, scene_path, scene = write_scene(base_world, output_dir, seed=seed, num_obstacles=num_obstacles)

    layout = ", ".join(
        f"{box['color']} ({box['position'][0]:.2f}, {box['position'][1]:.2f})"
        for box in scene["boxes"].values())

    # ------------------- Gazebo -------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_description"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={"world_path": world_path, "headless": headless}.items()
    )

    # ------------------- Controllers -------------------
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # ------------------- MoveIt -------------------
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_moveit"),
                "launch",
                "moveit.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True", "rviz": rviz}.items()
    )

    # ------------------- Vision Node -------------------
    vision_node = Node(
        package="panda_vision",
        executable="color_detector",
        name="color_detector",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # ------------------- Pick and Place Node -------------------
    pick_and_place_node = Node(
        package="panda_pick_place",
        executable="pick_and_place",
        name="pick_and_place",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "scene_file": scene_path,
            "target_color": LaunchConfiguration("target_color").perform(context),
        }]
    )

    # ------------------- Colour Selection UI -------------------
    color_selector_node = Node(
        package="panda_pick_place",
        executable="color_selector",
        name="color_selector",
        output="screen",
        parameters=[{"port": int(LaunchConfiguration("ui_port").perform(context))}]
    )

    return [
        LogInfo(msg=f"Random scene (seed={seed}): {layout}; world: {world_path}"),
        gazebo,
        controller,
        moveit,
        vision_node,
        pick_and_place_node,
        color_selector_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("seed", default_value="",
                              description="Random seed for the box/obstacle layout (empty = new layout)"),
        DeclareLaunchArgument("num_obstacles", default_value="6"),
        DeclareLaunchArgument("target_color", default_value="",
                              description="Optional colour (red/green/blue) to pick right away"),
        DeclareLaunchArgument("headless", default_value="false" if os.environ.get("DISPLAY") else "true"),
        DeclareLaunchArgument("rviz", default_value=""),
        DeclareLaunchArgument("ui_port", default_value="8765"),
        OpaqueFunction(function=launch_setup),
    ])
