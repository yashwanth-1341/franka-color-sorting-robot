#!/usr/bin/env python3
"""
Obstacle-aware colour pick and place for the simulated Panda.

The node waits for a colour request on /target_color (std_msgs/String, one of
R/G/B or red/green/blue). It then:

  1. loads the randomized scene (table, obstacles, drop crate, boxes) into the
     MoveIt planning scene so OMPL plans around the obstacles,
  2. takes the box position from the colour detector (/color_coordinates),
     falling back to the spawn position from scene.json if it is not visible,
  3. picks the box with a top-down grasp and drops it into the crate.

Progress is published as JSON on /pick_place/status.

    ros2 topic pub --once /target_color std_msgs/String "{data: red}"
"""

import json
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda

from panda_pick_place.scene import apply_scene_to_gazebo, load_scene, relayout_scene, save_scene

COLOR_ALIASES = {
    "R": "R", "RED": "R",
    "G": "G", "GREEN": "G",
    "B": "B", "BLUE": "B",
}
COLOR_NAMES = {"R": "red", "G": "green", "B": "blue"}

# Arm turned away from the camera so it does not hide the boxes.
READY_JOINTS = [math.pi / 2, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4]

HAND_LINKS = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]

# x offset of each colour's slot from the crate centre (crate inside is 20 cm, boxes 5 cm)
CRATE_SLOT_X = {"R": -0.06, "G": 0.0, "B": 0.06}


def top_down_quat(yaw):
    """Hand pointing straight down (Rx(pi)), rotated by `yaw` around world z."""
    return [math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0]


def yaw_quat(yaw):
    return [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        self.declare_parameter("scene_file", "")
        self.declare_parameter("target_color", "")  # optional: start a pick right away
        self.declare_parameter("detection_timeout", 5.0)
        self.declare_parameter("max_velocity", 0.25)
        self.declare_parameter("max_acceleration", 0.25)
        # panda_hand origin -> fingertip distance
        self.declare_parameter("tcp_offset", 0.1034)
        # How far below the box top the fingertips go when grasping
        self.declare_parameter("grasp_depth", 0.035)
        self.declare_parameter("pregrasp_height", 0.12)
        # Finger position per finger when closed on a 5 cm box (slightly squeezing)
        self.declare_parameter("grasp_finger_position", 0.015)
        # Sum of both finger positions above which the gripper holds the box:
        # ~0.05 around a 5 cm box, 2 * grasp_finger_position = 0.03 when empty.
        self.declare_parameter("holding_min_finger_sum", 0.04)
        # Attempts per requested colour; failed picks are retried with a fresh detection
        self.declare_parameter("max_attempts", 3)
        # Layouts in which MoveIt cannot reach every box are replaced, up to this many times
        self.declare_parameter("max_layout_attempts", 10)
        self.declare_parameter("world_name", "pick_place_world")

        self.holding_min_finger_sum = float(self.get_parameter("holding_min_finger_sum").value)
        self.max_attempts = int(self.get_parameter("max_attempts").value)
        self.max_layout_attempts = int(self.get_parameter("max_layout_attempts").value)
        self.world_name = self.get_parameter("world_name").value
        self.tcp_offset = float(self.get_parameter("tcp_offset").value)
        self.grasp_depth = float(self.get_parameter("grasp_depth").value)
        self.pregrasp_height = float(self.get_parameter("pregrasp_height").value)
        self.grasp_finger_position = float(self.get_parameter("grasp_finger_position").value)
        self.detection_timeout = float(self.get_parameter("detection_timeout").value)

        self.scene_file = self.get_parameter("scene_file").value
        if not self.scene_file:
            raise RuntimeError("Parameter 'scene_file' is required (written by the bringup launch)")
        self.scene = load_scene(self.scene_file)
        self.frame = self.scene["frame_id"]

        self.callback_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=self.frame,
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )
        self.moveit2.max_velocity = float(self.get_parameter("max_velocity").value)
        self.moveit2.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.moveit2.planner_id = "RRTConnect"
        self.moveit2.allowed_planning_time = 5.0
        self.moveit2.num_planning_attempts = 5
        # Plan from the current scene state with only the joints overridden, so a
        # box attached to the hand is kept in collision checking.
        self.moveit2._MoveIt2__move_action_goal.request.start_state.is_diff = True
        # Straight-line moves (descend, lift, lower into the crate) must not swing
        # the elbow into an obstacle; MoveIt does not check this unless asked to
        self.moveit2.cartesian_avoid_collisions = True
        self.hand_target = None  # last commanded hand pose, used to back off after a failure

        self.execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory", callback_group=self.callback_group)
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory",
            callback_group=self.callback_group)

        self.detections = {}  # color_id -> (x, y, z, yaw, stamp)
        self.picked = []
        self.busy = False
        self.lock = threading.Lock()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "/pick_place/status", latched)
        self.create_subscription(String, "/color_coordinates", self.coords_callback, 10,
                                 callback_group=self.callback_group)
        self.create_subscription(String, "/target_color", self.target_callback, 10,
                                 callback_group=self.callback_group)
        self.create_subscription(String, "/pick_place/new_layout", self.new_layout_callback, 10,
                                 callback_group=self.callback_group)

        self.publish_status("starting", message="Waiting for MoveIt and the controllers")
        threading.Thread(target=self.startup, daemon=True).start()

    # ------------------------------------------------------------------ status

    def publish_status(self, state, color=None, message=""):
        status = {
            "state": state,
            "color": COLOR_NAMES.get(color, color),
            "message": message,
            "picked": [COLOR_NAMES[c] for c in self.picked],
            "available": [COLOR_NAMES[c] for c in self.scene["boxes"] if c not in self.picked],
        }
        self.status_pub.publish(String(data=json.dumps(status)))
        # rclpy requires every logging call site to keep the same severity
        if state == "error":
            self.get_logger().error(f"[{state}] {message}")
        else:
            self.get_logger().info(f"[{state}] {message}")

    # --------------------------------------------------------------- callbacks

    def coords_callback(self, msg):
        try:
            fields = msg.data.split(",")
            color_id = fields[0].strip().upper()
            x, y, z = (float(v) for v in fields[1:4])
            yaw = float(fields[4]) if len(fields) > 4 else 0.0
        except (ValueError, IndexError):
            self.get_logger().warn(f"Ignoring malformed /color_coordinates message: '{msg.data}'")
            return
        self.detections[color_id] = (x, y, z, yaw, time.monotonic())

    def target_callback(self, msg):
        self.request_pick(msg.data)

    def request_pick(self, color):
        color_id = COLOR_ALIASES.get(color.strip().upper())
        if color_id is None:
            self.publish_status("error", message=f"Unknown colour '{color}', use red, green or blue")
            return
        with self.lock:
            if self.busy:
                self.get_logger().warn("Already executing a pick, ignoring new request")
                return
            if color_id in self.picked:
                self.publish_status("idle", color_id, f"The {COLOR_NAMES[color_id]} box is already in the crate")
                return
            self.busy = True
        threading.Thread(target=self.run_pick, args=(color_id,), daemon=True).start()

    # ----------------------------------------------------------------- startup

    def startup(self):
        with self.lock:
            self.busy = True
        while rclpy.ok() and not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("Waiting for /gripper_controller ...", throttle_duration_sec=10.0)
        while rclpy.ok() and self.moveit2.joint_state is None:
            time.sleep(0.5)
        # Give move_group time to come up before publishing the planning scene
        while rclpy.ok() and not self.execute_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("Waiting for move_group ...", throttle_duration_sec=10.0)
        try:
            self.update_planning_scene()
            self.move_gripper(0.04)
            self.move_joints(READY_JOINTS)
            feasible = self.ensure_feasible_layout()
            message = "No layout found in which every box is reachable. Try fewer obstacles (num_obstacles:=...)"
        except Exception as e:
            feasible, message = False, f"Startup failed: {e}"
        finally:
            with self.lock:
                self.busy = False
        if not feasible:
            self.publish_status("error", message=message)
            return
        self.publish_status("idle", message="Ready. Select a colour to pick.")

        initial = self.get_parameter("target_color").value
        if initial:
            self.request_pick(initial)

    # ------------------------------------------------------------------ layout

    def new_layout_callback(self, msg):
        with self.lock:
            if self.busy:
                self.get_logger().warn("Busy, ignoring the request for a new layout")
                return
            self.busy = True
        threading.Thread(target=self.run_new_layout, daemon=True).start()

    def run_new_layout(self):
        try:
            self.move_gripper(0.04)
            self.moveit2.detach_all_collision_objects()
            if not self.move_joints(READY_JOINTS):
                self.back_off_upwards()
            feasible = self.ensure_feasible_layout(shuffle_first=True)
            message = "" if feasible else "No layout found in which every box is reachable"
        except Exception as e:
            feasible, message = False, f"Could not change the layout: {e}"
        finally:
            with self.lock:
                self.busy = False
        if feasible:
            self.publish_status("idle", message="New layout ready. Select a colour to pick.")
        else:
            self.publish_status("error", message=message)

    def ensure_feasible_layout(self, shuffle_first=False):
        """Make sure MoveIt can reach every box around the obstacles. Layouts
        where it cannot are replaced by a new random layout in Gazebo."""
        for attempt in range(1, self.max_layout_attempts + 1):
            if shuffle_first or attempt > 1:
                self.publish_status("busy", message="Placing the boxes and obstacles at new random positions")
                self.shuffle_layout()
                # The reachability check plans from the ready pose
                self.move_joints(READY_JOINTS)
            self.publish_status("busy", message="Checking that every box can be reached around the obstacles")
            unreachable = self.unreachable_boxes()
            if not unreachable:
                return True
            self.get_logger().warn(
                "Layout rejected, cannot reach the " + ", ".join(COLOR_NAMES[c] for c in unreachable) + " box")
        return False

    def shuffle_layout(self):
        """New random positions for the boxes and obstacles, in Gazebo, in the
        planning scene and in scene.json. Boxes in the crate go back on the table."""
        new_scene = relayout_scene(self.scene)
        apply_scene_to_gazebo(self.scene, new_scene, self.world_name)
        self.scene = new_scene
        save_scene(new_scene, self.scene_file)
        self.picked = []
        self.detections.clear()  # old detections would point at the old positions
        self.update_planning_scene()
        layout = ", ".join(f"{b['color']} ({b['position'][0]:.2f}, {b['position'][1]:.2f})"
                           for b in new_scene["boxes"].values())
        self.get_logger().info(f"New layout: {layout}")
        time.sleep(1.0)  # let the boxes settle and the camera see them

    def unreachable_boxes(self):
        """Colours of the boxes that MoveIt cannot reach: no collision-free path
        from the ready pose to above the box, or no straight descent onto it."""
        unreachable = []
        for color_id, box in self.scene["boxes"].items():
            x, y, z = box["position"]
            approach, _ = self.plan_grasp(color_id, (x, y, z), box["yaw"])
            if approach is None:
                unreachable.append(color_id)
        return unreachable

    def plan_grasp(self, color_id, box_center, box_yaw):
        """Find a top-down grasp rotation for which both the path to the
        pre-grasp pose (around the obstacles) and the straight descent from there
        can be planned. Returns (approach trajectory, quaternion) or (None, None).

        The box is square, so any of the four 90 deg rotations is a valid grasp;
        the closest one is tried first. Some rotations reach the pre-grasp pose
        with the wrist at its limits, from where the descent is impossible."""
        box = self.scene["boxes"][color_id]
        x, y, z = box_center
        grasp_z = z + box["size"][2] / 2 + self.tcp_offset - self.grasp_depth
        for offset in (0.0, math.pi / 2, -math.pi / 2, math.pi):
            quat = top_down_quat(box_yaw + offset)
            approach = self.plan(position=(x, y, grasp_z + self.pregrasp_height), quat_xyzw=quat)
            if approach is None:
                continue
            # The fingers end up around the box, so check the descent without it
            self.moveit2.remove_collision_object(box["name"])
            time.sleep(0.3)
            end = dict(zip(approach.joint_names, approach.points[-1].positions))
            descent = self.plan(cartesian=True, position=(x, y, grasp_z), quat_xyzw=quat,
                                start_joint_state=[end[n] for n in panda.joint_names()])
            self.moveit2.add_collision_box(
                id=box["name"], size=box["size"], position=box_center, quat_xyzw=yaw_quat(box_yaw))
            time.sleep(0.3)
            if descent is not None:
                return approach, quat
        return None, None

    # ---------------------------------------------------------- planning scene

    def update_planning_scene(self, exclude=()):
        """(Re)publish table, crate, obstacles and the remaining boxes as collision objects."""
        table = self.scene["table"]
        (x0, y0), (x1, y1) = table["min_xy"], table["max_xy"]
        # Slightly below the real table top so panda_link0 (z=0.35) is not in collision
        self.moveit2.add_collision_box(
            id="table", size=(x1 - x0, y1 - y0, 0.03),
            position=((x0 + x1) / 2, (y0 + y1) / 2, 0.345 - 0.015), quat_xyzw=yaw_quat(0.0))

        for part in self.scene["crate"]["parts"]:
            self.moveit2.add_collision_box(
                id=part["name"], size=part["size"], position=part["position"], quat_xyzw=yaw_quat(0.0))

        for obstacle in self.scene["obstacles"]:
            self.moveit2.add_collision_box(
                id=obstacle["name"], size=obstacle["size"], position=obstacle["position"],
                quat_xyzw=yaw_quat(obstacle["yaw"]))

        for color_id, box in self.scene["boxes"].items():
            if color_id in self.picked or color_id in exclude:
                continue
            x, y, z, yaw = self.box_pose(color_id, wait=False)
            self.moveit2.add_collision_box(
                id=box["name"], size=box["size"], position=(x, y, z), quat_xyzw=yaw_quat(yaw))
        time.sleep(0.5)

    def box_pose(self, color_id, wait=True, fallback=True):
        """Box centre (x, y, z) and yaw, from vision if available, else from the scene
        file (or None if `fallback` is False)."""
        box = self.scene["boxes"][color_id]
        deadline = time.monotonic() + (self.detection_timeout if wait else 0.0)
        while True:
            detection = self.detections.get(color_id)
            if detection is not None and time.monotonic() - detection[4] < 2.0:
                x, y, z_top, yaw, _ = detection
                return x, y, z_top - box["size"][2] / 2, yaw
            if time.monotonic() >= deadline:
                break
            time.sleep(0.2)
        if not fallback:
            return None
        if wait:
            self.get_logger().warn(
                f"{COLOR_NAMES[color_id]} box not detected by the camera, using its spawn position")
        x, y, z = box["position"]
        return x, y, z, box["yaw"]

    def detected_since(self, color_id, since, timeout=2.0):
        """True if the camera sees the box in a frame taken after `since`."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            detection = self.detections.get(color_id)
            if detection is not None and detection[4] > since:
                return True
            time.sleep(0.1)
        return False

    # ----------------------------------------------------------------- motion

    # pymoveit2's blocking plan()/wait_until_executed() call rclpy.spin_once() on
    # this node, which pulls it out of the multi-threaded executor. Instead we
    # only use the async API here and wait on futures the executor completes.

    @staticmethod
    def wait_for(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.02)
        return True

    def plan(self, cartesian=False, **kwargs):
        future = self.moveit2.plan_async(cartesian=cartesian, **kwargs)
        if future is None or not self.wait_for(future, 30.0):
            return None
        return self.moveit2.get_trajectory(
            future, cartesian=cartesian, cartesian_fraction_threshold=0.95 if cartesian else 0.0)

    def send_action_goal(self, client, goal, timeout):
        """Send an action goal and block until it finishes. Returns True on success."""
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Action server '{client._action_name}' not available")
            return False
        goal_future = client.send_goal_async(goal)
        if not self.wait_for(goal_future, 10.0) or not goal_future.result().accepted:
            return False
        result_future = goal_future.result().get_result_async()
        if not self.wait_for(result_future, timeout):
            return False
        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def execute(self, trajectory):
        if trajectory is None:
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = trajectory
        duration = trajectory.points[-1].time_from_start
        ok = self.send_action_goal(
            self.execute_client, goal, duration.sec + duration.nanosec * 1e-9 + 30.0)
        time.sleep(0.2)  # let the simulated joints settle before planning the next move
        return ok

    def move_joints(self, joints):
        state = self.moveit2.joint_state
        if state is not None:
            current = dict(zip(state.name, state.position))
            if all(abs(current.get(n, 1e9) - j) < 0.01 for n, j in zip(panda.joint_names(), joints)):
                return True  # already there
        for _ in range(3):
            if self.execute(self.plan(joint_positions=joints)):
                return True
        return False

    def move_pose(self, position, quat, cartesian=False):
        self.hand_target = (position, quat)
        for _ in range(1 if cartesian else 3):
            if self.execute(self.plan(position=position, quat_xyzw=quat, cartesian=cartesian)):
                return True
        return False

    def back_off_upwards(self, height=0.12):
        """Move the hand straight up without collision checking. Used when the
        arm touches an obstacle in the planning scene: MoveIt refuses to plan
        from a state in collision, and up is the way out of the clutter."""
        if self.hand_target is None:
            return False
        (x, y, z), quat = self.hand_target
        self.get_logger().warn("The arm touches an obstacle, backing off upwards")
        self.moveit2.cartesian_avoid_collisions = False
        try:
            return self.move_pose((x, y, z + height), quat, cartesian=True)
        finally:
            self.moveit2.cartesian_avoid_collisions = True

    def move_gripper(self, finger_position, duration=1.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = panda.gripper_joint_names()
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[finger_position, finger_position],
            time_from_start=DurationMsg(sec=int(duration), nanosec=int((duration % 1) * 1e9)))]
        ok = self.send_action_goal(self.gripper_client, goal, duration + 5.0)
        time.sleep(0.3)  # let the fingers settle on the object
        return ok

    def finger_positions(self):
        state = self.moveit2.joint_state
        if state is None:
            return None
        names = panda.gripper_joint_names()
        return [state.position[state.name.index(n)] for n in names if n in state.name]

    def holding_box(self, stage):
        """True if the fingers are held apart by the box (not closed on nothing)."""
        time.sleep(0.2)  # fresh joint state
        fingers = self.finger_positions()
        if not fingers:
            return True  # no gripper state to judge by
        self.get_logger().info(f"Finger opening {stage}: {sum(fingers) * 1000:.1f} mm")
        return sum(fingers) >= self.holding_min_finger_sum

    # -------------------------------------------------------------- pick task

    def run_pick(self, color_id):
        name = COLOR_NAMES[color_id]
        ok, message = False, ""
        try:
            for attempt in range(1, self.max_attempts + 1):
                if attempt > 1:
                    self.publish_status(
                        "busy", color_id, f"{message}. Retrying (attempt {attempt} of {self.max_attempts})")
                try:
                    ok, message = self.pick_and_place(color_id, first_attempt=attempt == 1)
                except Exception as e:  # keep the node alive for the next request
                    ok, message = False, f"Unexpected error: {e}"
                if ok:
                    break
                self.get_logger().warn(f"Attempt {attempt} to pick the {name} box failed: {message}")
                self.recover(color_id)
        except Exception as e:
            ok, message = False, f"Unexpected error: {e}"
        finally:
            # Always accept new requests afterwards, whatever happened
            with self.lock:
                self.busy = False
        if ok:
            self.picked.append(color_id)
            self.publish_status("done", color_id, f"The {name} box was dropped into the crate")
            self.publish_status("idle", message="Ready. Select a colour to pick.")
        else:
            self.publish_status("error", color_id, f"{message} (gave up after {self.max_attempts} attempts)")

    def recover(self, color_id):
        """Open the hand, forget the attached box and go back to the ready pose,
        where the camera can see the table again."""
        self.move_gripper(0.04)
        self.moveit2.detach_all_collision_objects()
        time.sleep(0.2)
        self.moveit2.remove_collision_object(self.scene["boxes"][color_id]["name"])
        time.sleep(0.3)
        if self.move_joints(READY_JOINTS):
            return
        # The box may still be in the planning scene right where the hand is
        self.update_planning_scene(exclude=(color_id,))
        if self.move_joints(READY_JOINTS):
            return
        # The arm itself touches an obstacle: get out of contact first
        self.back_off_upwards()
        self.move_joints(READY_JOINTS)

    def pick_and_place(self, color_id, first_attempt=True):
        name = COLOR_NAMES[color_id]
        box = self.scene["boxes"][color_id]
        box_id = box["name"]
        box_height = box["size"][2]

        self.publish_status("busy", color_id, f"Locating the {name} box")
        # Retries need a real detection: the box may have been pushed away from its spawn pose
        pose = self.box_pose(color_id, fallback=first_attempt)
        if pose is None:
            return False, f"The camera cannot see the {name} box"
        x, y, z, box_yaw = pose
        top = z + box_height / 2
        self.get_logger().info(f"{name} box at ({x:.3f}, {y:.3f}, {z:.3f}), yaw {math.degrees(box_yaw):.1f} deg")

        self.update_planning_scene()

        grasp_z = top + self.tcp_offset - self.grasp_depth
        pregrasp_z = grasp_z + self.pregrasp_height

        # 1. Move above the box, planning around the obstacles, with a grasp
        #    rotation from which the straight descent is possible
        self.publish_status("busy", color_id, f"Moving above the {name} box (avoiding obstacles)")
        self.move_gripper(0.04)
        approach, grasp_quat = self.plan_grasp(color_id, (x, y, z), box_yaw)
        if approach is None:
            return False, f"Could not find a collision-free path above the {name} box"
        self.hand_target = ((x, y, pregrasp_z), grasp_quat)
        if not self.execute(approach) and not self.move_pose((x, y, pregrasp_z), grasp_quat):
            return False, f"Could not move above the {name} box"

        # 2. Straight down, close the fingers
        self.publish_status("busy", color_id, f"Grasping the {name} box")
        self.moveit2.remove_collision_object(box_id)
        time.sleep(0.3)
        if not self.move_pose((x, y, grasp_z), grasp_quat, cartesian=True):
            return False, "Could not descend to the grasp pose"
        self.move_gripper(self.grasp_finger_position)

        if not self.holding_box("after closing"):
            return False, f"The gripper missed the {name} box"

        # Tell MoveIt the box is now part of the hand
        self.moveit2.add_collision_box(
            id=box_id, size=box["size"], position=(x, y, z), quat_xyzw=yaw_quat(box_yaw))
        time.sleep(0.2)
        self.moveit2.attach_collision_object(id=box_id, link_name="panda_hand", touch_links=HAND_LINKS)
        time.sleep(0.3)

        # 3. Lift straight up
        if not self.move_pose((x, y, pregrasp_z), grasp_quat, cartesian=True):
            return False, "Could not lift the box"

        if not self.holding_box("after lifting"):
            return False, f"The {name} box slipped out of the gripper"

        # 4. Carry it over the crate (again planning around the obstacles)
        self.publish_status("busy", color_id, f"Carrying the {name} box to the crate")
        crate = self.scene["crate"]
        cx, cy, crate_z = crate["position"]
        # Every colour has its own slot in the crate so boxes never land on each other
        cx += CRATE_SLOT_X[color_id]
        # Box bottom 4 cm above the rim, then lowered to 2.5 cm above the crate floor
        hand_above_box_bottom = box_height - self.grasp_depth + self.tcp_offset
        above_z = crate_z + crate["height"] + 0.04 + hand_above_box_bottom
        release_z = crate_z + crate["floor_thickness"] + 0.025 + hand_above_box_bottom
        # Fingers close along world y for yaw 0 / pi, i.e. towards the long side of the slot
        place_quat = None
        for yaw in (0.0, math.pi):
            if self.move_pose((cx, cy, above_z), top_down_quat(yaw)):
                place_quat = top_down_quat(yaw)
                break
        if place_quat is None:
            return False, "Could not find a collision-free path to the crate"
        if not self.holding_box("above the crate"):
            return False, f"The {name} box was lost on the way to the crate"

        # 5. Lower the box into its slot and release it
        self.publish_status("busy", color_id, f"Placing the {name} box in the crate")
        lowered = self.move_pose((cx, cy, release_z), place_quat, cartesian=True)
        if not lowered:
            self.get_logger().warn("Could not lower the box into the crate, dropping it from above")
        self.move_gripper(0.04)
        self.moveit2.detach_collision_object(box_id)
        time.sleep(0.2)
        self.moveit2.remove_collision_object(box_id)
        time.sleep(0.5)
        if lowered:
            self.move_pose((cx, cy, above_z), place_quat, cartesian=True)

        # 6. Back to the ready pose (out of the camera view), then make sure the
        #    box is no longer on the table (the crate is outside the camera view)
        self.publish_status("busy", color_id, "Returning to the ready pose")
        self.move_joints(READY_JOINTS)
        self.publish_status("busy", color_id, f"Checking that the {name} box left the table")
        if self.detected_since(color_id, time.monotonic() + 0.5):
            return False, f"The {name} box is still on the table"
        return True, ""


def main():
    rclpy.init()
    node = PickAndPlace()
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
