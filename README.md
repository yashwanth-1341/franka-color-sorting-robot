# Franka Panda colour pick & place (ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2)

The Panda picks a box of the colour you choose (red, green or blue) from a table
and drops it into the yellow crate (black rim). It plans around the obstacles on
the table. Every launch, and every click on **New random layout** in the web
page, generates a new random layout of boxes and obstacles.

**New to the project? Read [DOCUMENTATION.md](DOCUMENTATION.md)**: installation,
every file explained, the colour detection and the coordinate transformations.

https://github.com/user-attachments/assets/8f789570-0bba-4835-8808-e1d1e641c410

> **At a glance:** a Franka Emika Panda (7 joints) in Gazebo Harmonic sorts red,
> green and blue boxes into a crate. OpenCV HSV segmentation finds each box in
> an overhead camera image, and TF projects its pixel position onto the table in
> the world frame. MoveIt 2 plans a collision-free path around randomly placed
> obstacles through pymoveit2, and ros2_control executes the arm and gripper
> trajectories. Everything runs on ROS 2 Jazzy in a Docker dev container, with a
> web page for choosing the colour and watching the robot.

## Setup

The dev container (`.devcontainer/Dockerfile`) installs all dependencies. For an
existing container:

```bash
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch panda_bringup simulated_robot.launch.py
```

Then open **http://localhost:8765** (VS Code forwards the port) and click a colour.
The page shows a live video of the robot in Gazebo, the overhead camera with the
detected boxes and the robot status. No X display is needed.

Only one simulation can run at a time; the launch refuses to start while another
Gazebo server is running. Stop the old one with Ctrl+C first.

You can also pick, or ask for a new layout, from a terminal:

```bash
ros2 topic pub --once /target_color std_msgs/msg/String "{data: red}"
ros2 topic pub --once /pick_place/new_layout std_msgs/msg/String "{data: ''}"
```

Launch arguments:

| argument | default | meaning |
| --- | --- | --- |
| `seed` | *(empty)* | Random seed. Empty gives a new layout every launch; set it to reproduce a layout |
| `num_obstacles` | `6` | Number of obstacles on the table |
| `target_color` | *(empty)* | Start picking this colour as soon as the robot is ready |
| `headless` | `true` if `$DISPLAY` is unset | Run Gazebo server-only with offscreen rendering |
| `rviz` | `false` when headless | Start RViz with the MoveIt MotionPlanning plugin |
| `ui_port` | `8765` | Port of the colour selection web UI |

To see the Gazebo GUI and RViz, the container needs an X display (for example
XQuartz on macOS with `DISPLAY` set). Then run with `headless:=false`.

## How it works

| package | role |
| --- | --- |
| `panda_description` | URDF/xacro, overhead camera, base table world (`worlds/table.world`) |
| `panda_controller` | ros2_control config and controller spawners |
| `panda_moveit` | MoveIt 2 config and `move_group` launch |
| `panda_vision` | `color_detector`: HSV segmentation of the camera image; projects each box onto the plane of the box tops using TF and the camera intrinsics (correcting for the visible side faces), then publishes `R/G/B,x,y,z,yaw` on `/color_coordinates` in the `world` frame |
| `panda_pick_place` | `scene.py` generates a random layout and writes the Gazebo world and `scene.json`. `pick_and_place` loads the table, crate, obstacles and the other boxes into the MoveIt planning scene so OMPL plans around them, then runs pre-grasp → Cartesian descent → grasp → lift → crate → release. It checks the finger opening after grasping, after lifting and above the crate, and checks with the camera that the box has left the table; a failed pick is retried up to 3 times with a fresh detection. `color_selector` is the web UI with the live Gazebo view (`/scene_camera/image`) |
| `panda_bringup` | `simulated_robot.launch.py` starts everything |

Generated files for the current run go to `/tmp/panda_pick_place/` (`pick_place.world`, `scene.json`).

Topics:
- `/target_color` (`std_msgs/String`): colour request
- `/pick_place/status` (`std_msgs/String`, JSON): robot status
- `/color_coordinates`: detections
- `/color_detector/debug_image`: annotated camera image
- `/pick_place/new_layout` (`std_msgs/String`): request a new random layout
- `/scene_camera/image`: live view of the robot shown in the web page
