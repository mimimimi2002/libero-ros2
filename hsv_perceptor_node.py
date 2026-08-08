#!/usr/bin/env python3
"""
HSV-based vision perceptor.

Parameters from config/perception.json and ball radius from config/env.json.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float64MultiArray, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

from config_loader import load_env_config, load_perception_config


class HsvPerceptorNode(Node):
    def __init__(self):
        super().__init__('hsv_perceptor_node')
        env_cfg = load_env_config()
        perc_cfg = load_perception_config()

        self.BALL_RADIUS = float(env_cfg["balls"]["radius"])
        self.BOX_CENTER_OFFSET_Z = float(perc_cfg.get("box_center_offset_z", 0.02))
        self.ball_min_area = float(perc_cfg["detection"]["ball_min_area"])
        self.box_min_area = float(perc_cfg["detection"]["box_min_area"])

        hsv = perc_cfg["hsv"]
        self.ball_hsv_low = np.array(hsv["ball_low"], dtype=np.int32)
        self.ball_hsv_high = np.array(hsv["ball_high"], dtype=np.int32)
        self.box_hsv_low = np.array(hsv["box_low"], dtype=np.int32)
        self.box_hsv_high = np.array(hsv["box_high"], dtype=np.int32)

        self.get_logger().info("HSV perceptor started (HSV + table-plane lift)...")

        self.bridge = CvBridge()
        self.K = None
        self.T_world_cam = None
        self.table_height = None

        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/camera/extrinsic', self.extrinsic_callback, 10
        )
        self.create_subscription(Float32, '/world/table_height', self.table_callback, 10)

        self.pub_balls = self.create_publisher(PoseArray, '/percept/ball_poses', 10)
        self.pub_box = self.create_publisher(Pose, '/percept/box_pose', 10)

    def info_callback(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def extrinsic_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 16:
            return
        self.T_world_cam = np.array(msg.data, dtype=np.float64).reshape(4, 4)

    def table_callback(self, msg: Float32):
        self.table_height = float(msg.data)

    def pixel_to_plane(self, u, v, plane_z):
        """Cast a ray through pixel (u, v) and intersect z = plane_z."""
        if self.K is None or self.T_world_cam is None:
            return None

        pix = np.array([u, v, 1.0], dtype=np.float64)
        d_cam = np.linalg.inv(self.K) @ pix
        d_cam = d_cam / np.linalg.norm(d_cam)

        R = self.T_world_cam[:3, :3]
        o = self.T_world_cam[:3, 3]
        d = R @ d_cam

        if abs(d[2]) < 1e-8:
            return None

        t = (plane_z - o[2]) / d[2]
        if t <= 0:
            return None

        return o + t * d

    def image_callback(self, msg):
        if self.K is None or self.T_world_cam is None or self.table_height is None:
            return

        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        ball_plane_z = self.table_height + self.BALL_RADIUS
        box_plane_z = self.table_height + self.BOX_CENTER_OFFSET_Z

        ball_mask = cv2.inRange(hsv_img, self.ball_hsv_low, self.ball_hsv_high)
        contours, _ = cv2.findContours(
            ball_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        ball_poses = PoseArray()
        ball_poses.header = msg.header
        detections = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area <= self.ball_min_area:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = float(M["m10"] / M["m00"])
            cy = float(M["m01"] / M["m00"])
            xyz = self.pixel_to_plane(cx, cy, ball_plane_z)
            if xyz is None:
                continue

            pose = Pose()
            pose.position.x = float(xyz[0])
            pose.position.y = float(xyz[1])
            pose.position.z = float(xyz[2])
            detections.append(pose)

            cv2.circle(cv_img, (int(cx), int(cy)), 5, (0, 255, 0), -1)
            label = f"({xyz[0]:.2f},{xyz[1]:.2f})"
            cv2.putText(
                cv_img, label, (int(cx) + 6, int(cy) - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1,
            )

        detections.sort(key=lambda p: p.position.x)
        ball_poses.poses = detections
        self.pub_balls.publish(ball_poses)

        box_mask = cv2.inRange(hsv_img, self.box_hsv_low, self.box_hsv_high)
        box_contours, _ = cv2.findContours(
            box_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if len(box_contours) > 0:
            largest_box = max(box_contours, key=cv2.contourArea)
            if cv2.contourArea(largest_box) > self.box_min_area:
                M = cv2.moments(largest_box)
                if M["m00"] != 0:
                    cx = float(M["m10"] / M["m00"])
                    cy = float(M["m01"] / M["m00"])
                    xyz = self.pixel_to_plane(cx, cy, box_plane_z)
                    if xyz is not None:
                        box_pose = Pose()
                        box_pose.position.x = float(xyz[0])
                        box_pose.position.y = float(xyz[1])
                        box_pose.position.z = float(xyz[2])
                        self.pub_box.publish(box_pose)

                        x, y, w, h = cv2.boundingRect(largest_box)
                        cv2.rectangle(cv_img, (x, y), (x + w, y + h), (255, 0, 0), 2)

        cv2.imshow("Perception Debug Window", cv_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = HsvPerceptorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
