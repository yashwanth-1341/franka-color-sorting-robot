#!/usr/bin/env python3
"""
Detects the red, green and blue boxes in the overhead camera image and
publishes their positions in the `world` frame.

Each detected box is published on /color_coordinates (std_msgs/String) as

    "<color_id>,<x>,<y>,<z>,<yaw>"

where color_id is R, G or B, (x, y, z) is the centre of the box top face and
yaw is the box orientation around the world z axis (radians, in [-pi/4, pi/4],
since the boxes have a square footprint).

An annotated image is published on /color_detector/debug_image.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

# HSV ranges (OpenCV hue is 0..180). Red wraps around 0.
COLOR_RANGES = {
    "R": [((0, 120, 60), (8, 255, 255)), ((172, 120, 60), (180, 255, 255))],
    "G": [((45, 120, 60), (80, 255, 255))],
    "B": [((105, 120, 60), (130, 255, 255))],
}
DRAW_COLORS = {"R": (0, 0, 255), "G": (0, 255, 0), "B": (255, 0, 0)}


def quaternion_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class ColorDetector(Node):
    def __init__(self):
        super().__init__("color_detector")

        self.declare_parameter("world_frame", "world")
        # Height of the box top faces: table top (0.3657) + box height (0.08)
        self.declare_parameter("target_plane_z", 0.4457)
        self.declare_parameter("box_height", 0.08)
        self.declare_parameter("min_area_px", 80)
        self.declare_parameter("publish_rate", 2.0)

        self.world_frame = self.get_parameter("world_frame").value
        self.plane_z = float(self.get_parameter("target_plane_z").value)
        self.box_height = float(self.get_parameter("box_height").value)
        self.min_area = float(self.get_parameter("min_area_px").value)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_matrix = None
        self.latest_image = None

        self.create_subscription(CameraInfo, "/camera/camera_info", self.info_callback, 10)
        self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.coords_pub = self.create_publisher(String, "/color_coordinates", 10)
        self.debug_pub = self.create_publisher(Image, "/color_detector/debug_image", 10)

        period = 1.0 / float(self.get_parameter("publish_rate").value)
        self.create_timer(period, self.process)
        self.get_logger().info("Color detector started, waiting for camera images...")

    def info_callback(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=float).reshape(3, 3)

    def image_callback(self, msg):
        self.latest_image = msg

    def pixel_to_world(self, u, v, rotation, origin):
        """Intersect the camera ray through pixel (u, v) with the plane z = plane_z."""
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        ray = rotation @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        if abs(ray[2]) < 1e-6:
            return None
        t = (self.plane_z - origin[2]) / ray[2]
        return origin + t * ray

    def top_face_pose(self, contour, rotation, origin):
        """Centre and yaw of the box top face in the world frame.

        The blob contains the top face plus the side faces turned towards the
        camera, so its centroid is biased towards the point below the camera
        (nadir). Projected onto the top-face plane, the bottom edges appear
        scaled towards the nadir by k = h_cam / (h_cam + box_height). Along the
        radial direction e, the far end of the blob is the top face and the near
        end is the scaled bottom face, which gives the centre:
            c = (far + near / k) / 2

        Returns (center_xy, yaw) or None. The yaw (modulo 90 deg) comes from the
        edge directions of the far half of the outline, which is top face only.
        """
        points = [self.pixel_to_world(u, v, rotation, origin) for u, v in contour.reshape(-1, 2)]
        points = np.array([p[:2] for p in points if p is not None])
        if len(points) < 3:
            return None
        nadir = origin[:2]
        radial = points.mean(axis=0) - nadir
        r = np.linalg.norm(radial)
        if r < 1e-3:
            return None
        e = radial / r
        h_cam = origin[2] - self.plane_z
        k = h_cam / (h_cam + self.box_height)
        along = (points - nadir) @ e
        center_along = (along.max() + along.min() / k) / 2
        # Perpendicular offset: the blob is symmetric across e up to noise
        n = np.array([-e[1], e[0]])
        center_perp = ((points - nadir) @ n).mean()
        center = nadir + center_along * e + center_perp * n

        # Length-weighted circular mean of 4 * edge angle (square symmetry), over
        # the straight edges of the simplified outline (raw pixel contours are
        # staircases of horizontal/vertical steps)
        polygon = cv2.approxPolyDP(contour, 2.0, True).reshape(-1, 2)
        corners = [self.pixel_to_world(u, v, rotation, origin) for u, v in polygon]
        corners = np.array([p[:2] for p in corners if p is not None])
        if len(corners) < 3:
            return None
        corner_along = (corners - nadir) @ e
        segments = np.roll(corners, -1, axis=0) - corners
        segments = segments[(corner_along + np.roll(corner_along, -1)) / 2 > center_along]
        if len(segments) == 0:
            return None
        angles = np.arctan2(segments[:, 1], segments[:, 0])
        weights = np.linalg.norm(segments, axis=1)
        yaw = math.atan2((weights * np.sin(4 * angles)).sum(), (weights * np.cos(4 * angles)).sum()) / 4
        return center, yaw

    def process(self):
        if self.latest_image is None or self.camera_matrix is None:
            return
        msg = self.latest_image
        try:
            # No timeout: with sim time, waiting here would block the executor that
            # receives /clock and deadlock the node.
            tf = self.tf_buffer.lookup_transform(self.world_frame, msg.header.frame_id, Time())
        except TransformException as e:
            self.get_logger().warn(f"Waiting for camera transform: {e}", throttle_duration_sec=5.0)
            return
        rotation = quaternion_to_matrix(tf.transform.rotation)
        t = tf.transform.translation
        origin = np.array([t.x, t.y, t.z])

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        debug = image.copy()

        for color_id, ranges in COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(contour) < self.min_area:
                continue

            moments = cv2.moments(contour)
            u = moments["m10"] / moments["m00"]
            v = moments["m01"] / moments["m00"]
            top_face = self.top_face_pose(contour, rotation, origin)
            if top_face is None:
                continue
            center_xy, yaw = top_face
            center = np.array([center_xy[0], center_xy[1], self.plane_z])
            corners = cv2.boxPoints(cv2.minAreaRect(contour))

            self.coords_pub.publish(String(
                data=f"{color_id},{center[0]:.4f},{center[1]:.4f},{center[2]:.4f},{yaw:.4f}"))

            cv2.drawContours(debug, [corners.astype(np.int32)], 0, DRAW_COLORS[color_id], 2)
            cv2.circle(debug, (int(u), int(v)), 3, (255, 255, 255), -1)
            cv2.putText(debug, f"{color_id} ({center[0]:.2f}, {center[1]:.2f})",
                        (int(u) + 8, int(v) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255), 1, cv2.LINE_AA)

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main():
    rclpy.init()
    node = ColorDetector()
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
