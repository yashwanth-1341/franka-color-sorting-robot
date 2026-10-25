#!/usr/bin/env python3
"""
Forwards the joint_state_publisher_gui sliders (/joint_commands) to the arm and
gripper trajectory controllers, see launch/slider_controller.launch.py.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
GRIPPER_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]


class SliderControl(Node):
    def __init__(self):
        super().__init__("slider_control")
        self.arm_pub = self.create_publisher(JointTrajectory, "arm_controller/joint_trajectory", 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, "gripper_controller/joint_trajectory", 10)
        self.create_subscription(JointState, "joint_commands", self.slider_callback, 10)
        self.get_logger().info("Slider Control Node started")

    def publish(self, publisher, joint_names, msg):
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in joint_names):
            return
        point = JointTrajectoryPoint()
        point.positions = [positions[name] for name in joint_names]
        point.time_from_start.sec = 1
        trajectory = JointTrajectory(joint_names=joint_names, points=[point])
        publisher.publish(trajectory)

    def slider_callback(self, msg):
        self.publish(self.arm_pub, ARM_JOINTS, msg)
        self.publish(self.gripper_pub, GRIPPER_JOINTS, msg)


def main():
    rclpy.init()
    node = SliderControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
