#!/usr/bin/env python3
"""
Minimal Lift-env bridge (legacy / smoke-test).

Publishes /camera/image_raw at 30 Hz and steps the sim on /robot/cmd_action.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

import cv2
from cv_bridge import CvBridge
import numpy as np
import robosuite as suite


class RobosuiteBridgeLiftNode(Node):
    def __init__(self):
        super().__init__('robosuite_bridge_lift_node')
        self.get_logger().info("Initializing Lift simulation scene...")

        self.env = suite.make(
            env_name="Lift",
            robots="Panda",
            has_renderer=True,
            has_offscreen_renderer=True,
            render_camera="agentview",
            use_camera_obs=True,
            camera_names=["agentview"],
            camera_heights=480,
            camera_widths=640,
            control_freq=20,
        )
        self.obs = self.env.reset()
        self.bridge = CvBridge()

        self.publisher_image = self.create_publisher(Image, '/camera/image_raw', 10)
        self.subscription_action = self.create_subscription(
            Float32MultiArray, '/robot/cmd_action', self.action_callback, 10
        )

        self.create_timer(1.0 / 30.0, self.publish_frame_callback)
        self.get_logger().info("Lift bridge ready; publishing /camera/image_raw")

    def publish_frame_callback(self):
        raw_image = self.obs['agentview_image']
        bgr_image = cv2.cvtColor(raw_image, cv2.COLOR_RGB2BGR)
        flipped_image = cv2.flip(bgr_image, 0)

        img_msg = self.bridge.cv2_to_imgmsg(flipped_image, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "agentview_camera_link"
        self.publisher_image.publish(img_msg)
        self.env.render()

    def action_callback(self, msg):
        action = np.array(msg.data, dtype=np.float32)
        if len(action) == self.env.action_dim:
            self.obs, reward, done, info = self.env.step(action)
        else:
            self.get_logger().warn(
                f"Action dim mismatch: expected {self.env.action_dim}, got {len(action)}"
            )

    def destroy_node(self):
        self.env.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobosuiteBridgeLiftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Lift bridge...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
