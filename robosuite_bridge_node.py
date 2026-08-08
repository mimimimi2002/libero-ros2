#!/usr/bin/env python3
"""
Main robosuite bridge node for the tidy-up task.

Scene parameters are loaded from config/env.json.
Publishes camera image, camera geometry, EEF pose, and GT object poses (/gt/*).
Subscribes to /robot/cmd_action and /robot/target_pose.
"""
import os

# Load GL backend from config before importing MuJoCo / robosuite
from config_loader import load_env_config

_ENV_CFG = load_env_config()
os.environ["MUJOCO_GL"] = str(_ENV_CFG.get("sim", {}).get("mujoco_gl", "glfw"))
if "PYOPENGL_PLATFORM" in os.environ:
    del os.environ["PYOPENGL_PLATFORM"]

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Float32
from geometry_msgs.msg import Pose, PoseArray

import cv2
from cv_bridge import CvBridge
import numpy as np
from robosuite.environments.manipulation.lift import Lift
from robosuite.models.objects import BallObject, BoxObject
from robosuite.controllers import load_controller_config
from robosuite.utils.camera_utils import (
    get_camera_intrinsic_matrix,
    get_camera_extrinsic_matrix,
)


class CustomTidyUpEnv(Lift):
    """Tidy-up scene built from config/env.json."""

    def __init__(self, env_cfg, **kwargs):
        self.env_cfg = env_cfg
        super().__init__(**kwargs)

    def _load_model(self):
        super()._load_model()
        balls_cfg = self.env_cfg["balls"]
        box_cfg = self.env_cfg["box"]

        self.ball_radius = float(balls_cfg["radius"])
        self.balls = []
        for i in range(len(balls_cfg["positions_xy"])):
            ball = BallObject(
                name=f"pingpong_ball_{i}",
                size=[self.ball_radius],
                rgba=list(balls_cfg["rgba"]),
                density=float(balls_cfg["density"]),
            )
            self.balls.append(ball)

        self.box_parts = []
        self.box_bottom = None
        for part in box_cfg["parts"]:
            obj = BoxObject(
                name=part["name"],
                size=list(part["size"]),
                rgba=list(box_cfg["rgba"]),
                joints=[],
            )
            self.box_parts.append((obj, list(part["offset"])))
            if part["name"] == "box_bottom":
                self.box_bottom = obj
            self.model.worldbody.append(obj.get_obj())

        if self.box_bottom is None:
            raise ValueError("config/env.json box.parts must include 'box_bottom'")

        for ball in self.balls:
            self.model.worldbody.append(ball.get_obj())

    def _reset_internal(self):
        super()._reset_internal()

        # Hide the default Lift cube
        if hasattr(self, "cube"):
            cube_joint = self.cube.joints[0]
            if cube_joint in self.sim.model.joint_names:
                j_id = self.sim.model.joint_name2id(cube_joint)
                qpos_addr = self.sim.model.jnt_qposadr[j_id]
                self.sim.data.qpos[qpos_addr : qpos_addr + 3] = np.array([0, 0, -10.0])

        table_z = float(self.env_cfg["table"]["height"])
        z_ball = table_z + self.ball_radius
        positions_xy = self.env_cfg["balls"]["positions_xy"]
        self.ball_init_poses = [
            np.array([float(xy[0]), float(xy[1]), z_ball]) for xy in positions_xy
        ]
        for ball, pos in zip(self.balls, self.ball_init_poses):
            for joint in ball.joints:
                if joint in self.sim.model.joint_names:
                    j_id = self.sim.model.joint_name2id(joint)
                    qpos_addr = self.sim.model.jnt_qposadr[j_id]
                    self.sim.data.qpos[qpos_addr : qpos_addr + 3] = pos

        center = np.array(self.env_cfg["box"]["center"], dtype=float)
        for obj, offset in self.box_parts:
            body_id = self.sim.model.body_name2id(obj.root_body)
            self.sim.model.body_pos[body_id] = center + np.array(offset, dtype=float)

        self.sim.forward()


class RobosuiteBridgeNode(Node):
    """Simulation boundary: sensors out, actuator in. GT poses go to /gt/* only."""

    def __init__(self):
        super().__init__('robosuite_bridge_node')
        self.env_cfg = _ENV_CFG
        cam = self.env_cfg["camera"]
        sim = self.env_cfg["sim"]

        self.camera_name = str(cam["name"])
        self.camera_height = int(cam["height"])
        self.camera_width = int(cam["width"])
        self.table_height = float(self.env_cfg["table"]["height"])

        self.get_logger().info("Initializing tidy-up scene from config/env.json...")

        controller_config = load_controller_config(
            default_controller=str(sim.get("controller", "OSC_POSE"))
        )

        self.env = CustomTidyUpEnv(
            self.env_cfg,
            robots=str(sim.get("robot", "Panda")),
            controller_configs=controller_config,
            has_renderer=bool(sim.get("has_renderer", False)),
            has_offscreen_renderer=bool(sim.get("has_offscreen_renderer", True)),
            use_camera_obs=True,
            camera_names=[self.camera_name],
            camera_heights=self.camera_height,
            camera_widths=self.camera_width,
            control_freq=int(sim.get("control_freq", 20)),
            horizon=100000,
            ignore_done=True,
        )

        self.obs = self.env.reset()
        self.bridge = CvBridge()
        self.current_action = np.zeros(self.env.action_dim)

        self.publisher_image = self.create_publisher(Image, '/camera/image_raw', 10)
        self.publisher_camera_info = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.publisher_extrinsic = self.create_publisher(Float64MultiArray, '/camera/extrinsic', 10)
        self.publisher_table_height = self.create_publisher(Float32, '/world/table_height', 10)
        self.publisher_eef = self.create_publisher(Pose, '/robot/eef_pose', 10)
        self.publisher_gt_balls = self.create_publisher(PoseArray, '/gt/ball_poses', 10)
        self.publisher_gt_box = self.create_publisher(Pose, '/gt/box_pose', 10)

        self.create_subscription(
            Float32MultiArray, '/robot/cmd_action', self.action_callback, 10
        )
        self.create_subscription(
            Pose, '/robot/target_pose', self.target_callback, 10
        )

        self.latest_eef = None
        self.latest_target = None
        self._last_pose_log_t = 0.0

        cv2.namedWindow("MuJoCo Realtime Simulation", cv2.WINDOW_NORMAL)
        self.get_logger().info(
            "Bridge ready: image/camera/eef + /gt/* for debug "
            "(task_manager should use /percept/* only)"
        )

    def action_callback(self, msg):
        action = np.array(msg.data, dtype=np.float32)
        if len(action) == self.env.action_dim:
            self.current_action = action

    def target_callback(self, msg):
        self.latest_target = msg

    def _log_eef_and_target(self):
        now = time.time()
        if now - self._last_pose_log_t < 1.0:
            return
        self._last_pose_log_t = now

        if self.latest_eef is None:
            return

        e = self.latest_eef.position
        if self.latest_target is None:
            self.get_logger().info(
                f"gripper=({e.x:.4f}, {e.y:.4f}, {e.z:.4f}) | target=(waiting)"
            )
            return

        t = self.latest_target.position
        err = np.array([t.x - e.x, t.y - e.y, t.z - e.z])
        self.get_logger().info(
            f"gripper=({e.x:.4f}, {e.y:.4f}, {e.z:.4f}) | "
            f"target=({t.x:.4f}, {t.y:.4f}, {t.z:.4f}) | "
            f"err_xyz=({err[0]*1000:.1f}, {err[1]*1000:.1f}, {err[2]*1000:.1f}) mm "
            f"|err|={np.linalg.norm(err)*1000:.1f} mm"
        )

    def _publish_camera_geometry(self, stamp):
        K = get_camera_intrinsic_matrix(
            self.env.sim, self.camera_name, self.camera_height, self.camera_width,
        )
        T_world_cam = get_camera_extrinsic_matrix(self.env.sim, self.camera_name)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "agentview_camera_link"
        info.height = self.camera_height
        info.width = self.camera_width
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx, fy = float(K[0, 0]), float(K[1, 1])
        cx, cy = float(K[0, 2]), float(K[1, 2])
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.publisher_camera_info.publish(info)

        ext = Float64MultiArray()
        ext.data = T_world_cam.astype(np.float64).reshape(-1).tolist()
        self.publisher_extrinsic.publish(ext)

        table_msg = Float32()
        table_msg.data = float(self.table_height)
        self.publisher_table_height.publish(table_msg)

    def run_loop(self):
        cam_key = f"{self.camera_name}_image"
        while rclpy.ok():
            try:
                self.obs, reward, done, info = self.env.step(self.current_action)
            except ValueError:
                self.obs = self.env.reset()
                continue

            stamp = self.get_clock().now().to_msg()

            eef_pos = self.obs['robot0_eef_pos']
            eef_msg = Pose()
            eef_msg.position.x = float(eef_pos[0])
            eef_msg.position.y = float(eef_pos[1])
            eef_msg.position.z = float(eef_pos[2])
            self.latest_eef = eef_msg
            self.publisher_eef.publish(eef_msg)
            self._log_eef_and_target()

            self._publish_camera_geometry(stamp)

            ball_msg = PoseArray()
            ball_msg.header.stamp = stamp
            ball_msg.header.frame_id = "world"
            for ball in self.env.balls:
                body_id = self.env.sim.model.body_name2id(ball.root_body)
                pos = self.env.sim.data.body_xpos[body_id]
                p = Pose()
                p.position.x = float(pos[0])
                p.position.y = float(pos[1])
                p.position.z = float(pos[2])
                ball_msg.poses.append(p)
            self.publisher_gt_balls.publish(ball_msg)

            box_bottom_id = self.env.sim.model.body_name2id(self.env.box_bottom.root_body)
            box_pos = self.env.sim.data.body_xpos[box_bottom_id]
            box_msg = Pose()
            box_msg.position.x = float(box_pos[0])
            box_msg.position.y = float(box_pos[1])
            box_msg.position.z = float(box_pos[2])
            self.publisher_gt_box.publish(box_msg)

            raw_image = self.obs[cam_key]
            bgr_image = cv2.cvtColor(raw_image, cv2.COLOR_RGB2BGR)
            flipped_image = cv2.flip(bgr_image, 0)

            img_msg = self.bridge.cv2_to_imgmsg(flipped_image, encoding="bgr8")
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = "agentview_camera_link"
            self.publisher_image.publish(img_msg)

            cv2.imshow("MuJoCo Realtime Simulation", flipped_image)
            cv2.waitKey(1)

            rclpy.spin_once(self, timeout_sec=0.001)

    def destroy_node(self):
        cv2.destroyAllWindows()
        self.env.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobosuiteBridgeNode()
    try:
        node.run_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
