#!/usr/bin/env python3
"""
Main robosuite bridge node for the tidy-up task.

Scene / sim parameters are loaded from config/env.json.
Publishes camera image, camera geometry, EEF pose, and GT object poses (/gt/*).
Subscribes to /robot/cmd_action and /robot/target_pose.
"""
import os

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
        cfg = self.env_cfg
        balls_cfg = cfg["balls"]
        box_cfg = cfg["box"]

        self.ball_radius = float(balls_cfg["radius"])
        self.balls = []
        damping = str(balls_cfg.get("joint_damping", 2.0))
        for i in range(len(balls_cfg["positions_xy"])):
            ball = BallObject(
                name=f"pingpong_ball_{i}",
                size=[self.ball_radius],
                rgba=list(balls_cfg["rgba"]),
                density=float(balls_cfg.get("density", 400.0)),
                friction=list(balls_cfg.get("friction", [2.5, 0.2, 0.1])),
                joints=[{"type": "free", "damping": damping}],
            )
            self.balls.append(ball)

        self.box_parts = []
        rgba = list(box_cfg["rgba"])
        for part in box_cfg["parts"]:
            obj = BoxObject(
                name=part["name"],
                size=list(part["size"]),
                rgba=rgba,
                joints=[],
            )
            self.box_parts.append(obj)
            if part["name"] == "box_bottom":
                self.box_bottom = obj

        for ball in self.balls:
            self.model.worldbody.append(ball.get_obj())
        for part in self.box_parts:
            self.model.worldbody.append(part.get_obj())

    def _bury_lift_cube(self):
        """Hide Lift's default red cube (invisible, no contact, under floor)."""
        if not hasattr(self, "cube"):
            return

        for joint in self.cube.joints:
            if joint not in self.sim.model.joint_names:
                continue
            j_id = self.sim.model.joint_name2id(joint)
            qpos_addr = self.sim.model.jnt_qposadr[j_id]
            qvel_addr = self.sim.model.jnt_dofadr[j_id]
            self.sim.data.qpos[qpos_addr : qpos_addr + 3] = np.array([0.0, 0.0, -1.0])
            ndof = 6 if self.sim.model.jnt_type[j_id] == 0 else 1
            self.sim.data.qvel[qvel_addr : qvel_addr + ndof] = 0.0

        body_id = self.sim.model.body_name2id(self.cube.root_body)
        for i in range(self.sim.model.ngeom):
            if self.sim.model.geom_bodyid[i] == body_id:
                self.sim.model.geom_rgba[i, 3] = 0.0
                self.sim.model.geom_contype[i] = 0
                self.sim.model.geom_conaffinity[i] = 0

        self.sim.forward()

    def _reset_internal(self):
        super()._reset_internal()
        self._bury_lift_cube()

        table_z = float(self.env_cfg["table"]["height"])
        z_ball = table_z + self.ball_radius
        self.ball_init_poses = [
            np.array([xy[0], xy[1], z_ball], dtype=float)
            for xy in self.env_cfg["balls"]["positions_xy"]
        ]
        for ball, pos in zip(self.balls, self.ball_init_poses):
            for joint in ball.joints:
                if joint in self.sim.model.joint_names:
                    j_id = self.sim.model.joint_name2id(joint)
                    qpos_addr = self.sim.model.jnt_qposadr[j_id]
                    self.sim.data.qpos[qpos_addr : qpos_addr + 3] = pos

        bx, by, bz = self.env_cfg["box"]["center"]
        for part_cfg, part in zip(self.env_cfg["box"]["parts"], self.box_parts):
            ox, oy, oz = part_cfg["offset"]
            body_id = self.sim.model.body_name2id(part.root_body)
            self.sim.model.body_pos[body_id] = [bx + ox, by + oy, bz + oz]

        self.sim.forward()


class RobosuiteBridgeNode(Node):
    """Simulation boundary: sensors out, actuator in. GT poses go to /gt/* only."""

    def __init__(self):
        super().__init__('robosuite_bridge_node')
        self.cfg = load_env_config()
        sim = self.cfg["sim"]
        cam = self.cfg["camera"]

        self.CAMERA_NAME = cam["name"]
        self.CAMERA_HEIGHT = int(cam["height"])
        self.CAMERA_WIDTH = int(cam["width"])
        self.TABLE_HEIGHT = float(self.cfg["table"]["height"])

        self.get_logger().info("Initializing tidy-up simulation scene...")

        controller_config = load_controller_config(
            default_controller=str(sim.get("controller", "OSC_POSE"))
        )

        self.control_freq = float(sim.get("control_freq", 40))
        self.env = CustomTidyUpEnv(
            env_cfg=self.cfg,
            robots=str(sim.get("robot", "Panda")),
            controller_configs=controller_config,
            has_renderer=bool(sim.get("has_renderer", False)),
            has_offscreen_renderer=bool(sim.get("has_offscreen_renderer", True)),
            use_camera_obs=True,
            camera_names=[self.CAMERA_NAME],
            camera_heights=self.CAMERA_HEIGHT,
            camera_widths=self.CAMERA_WIDTH,
            control_freq=int(self.control_freq),
            horizon=int(sim.get("horizon", 100000)),
            ignore_done=True,
        )

        self.obs = self.env.reset()
        self.bridge = CvBridge()
        self.target_action = np.zeros(self.env.action_dim, dtype=np.float64)
        self.applied_action = np.zeros(self.env.action_dim, dtype=np.float64)
        self.action_alpha = float(sim.get("action_alpha", 0.35))
        self._dt = 1.0 / self.control_freq
        self._next_step_t = time.perf_counter()
        self._step_count = 0
        self._display_stride = int(sim.get("display_stride", 2))

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
            f"Bridge ready @ {self.control_freq:.0f} Hz with action ramp "
            f"(alpha={self.action_alpha})"
        )

    def action_callback(self, msg):
        action = np.array(msg.data, dtype=np.float64)
        if len(action) == self.env.action_dim:
            self.target_action = action

    def target_callback(self, msg):
        self.latest_target = msg

    def _ramped_action(self):
        """Low-pass filter position deltas; snap gripper immediately."""
        a = self.action_alpha
        blended = (1.0 - a) * self.applied_action + a * self.target_action
        blended[-1] = self.target_action[-1]
        self.applied_action = blended
        return self.applied_action.astype(np.float32)

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
            self.env.sim, self.CAMERA_NAME, self.CAMERA_HEIGHT, self.CAMERA_WIDTH,
        )
        T_world_cam = get_camera_extrinsic_matrix(self.env.sim, self.CAMERA_NAME)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "agentview_camera_link"
        info.height = self.CAMERA_HEIGHT
        info.width = self.CAMERA_WIDTH
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
        table_msg.data = float(self.TABLE_HEIGHT)
        self.publisher_table_height.publish(table_msg)

    def run_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            now = time.perf_counter()
            if now < self._next_step_t:
                remaining = self._next_step_t - now
                rclpy.spin_once(self, timeout_sec=min(remaining, 0.005))
                continue

            self._next_step_t += self._dt
            if now - self._next_step_t > self._dt:
                self._next_step_t = now + self._dt

            try:
                action = self._ramped_action()
                self.obs, reward, done, info = self.env.step(action)
            except ValueError:
                self.obs = self.env.reset()
                continue

            self._step_count += 1
            stamp = self.get_clock().now().to_msg()

            eef_pos = self.obs['robot0_eef_pos']
            eef_msg = Pose()
            eef_msg.position.x = float(eef_pos[0])
            eef_msg.position.y = float(eef_pos[1])
            eef_msg.position.z = float(eef_pos[2])
            self.latest_eef = eef_msg
            self.publisher_eef.publish(eef_msg)
            self._log_eef_and_target()

            if self._step_count % self._display_stride == 0:
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

                raw_image = self.obs['agentview_image']
                bgr_image = cv2.cvtColor(raw_image, cv2.COLOR_RGB2BGR)
                flipped_image = cv2.flip(bgr_image, 0)

                img_msg = self.bridge.cv2_to_imgmsg(flipped_image, encoding="bgr8")
                img_msg.header.stamp = stamp
                img_msg.header.frame_id = "agentview_camera_link"
                self.publisher_image.publish(img_msg)

                cv2.imshow("MuJoCo Realtime Simulation", flipped_image)
                cv2.waitKey(1)

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
