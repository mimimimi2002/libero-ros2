#!/usr/bin/env python3
"""
Task manager for the tidy-up pick-and-place sequence.

Parameters from config/control.json.
Motion is strictly axis-separated (XY-only and Z-only phases).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray
import numpy as np

from config_loader import load_control_config


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        cfg = load_control_config()

        tcp = cfg.get("tcp_offset_xy", [0.0, 0.0])
        self.TCP_OFFSET_X = float(tcp[0])
        self.TCP_OFFSET_Y = float(tcp[1])
        self.TRAVEL_Z = float(cfg.get("travel_z", 1.05))
        self.CONTROL_HZ = float(cfg.get("control_hz", 40.0))
        self.XY_TOL = float(cfg.get("xy_tol", 0.012))
        self.Z_TOL = float(cfg.get("z_tol", 0.012))
        self.PLACE_XY_TOL = float(cfg.get("place_xy_tol", 0.020))
        self.STABLE_NEED = int(cfg.get("stable_need", 8))
        self.Kp = float(cfg.get("kp", 12.0))
        self.CMD_MAX_FAR = float(cfg.get("cmd_max_far", 0.40))
        self.CMD_MAX_NEAR = float(cfg.get("cmd_max_near", 0.18))
        self.NEAR_DIST = float(cfg.get("near_dist", 0.06))
        self.GRASP_HOLD_STEPS = int(cfg.get("grasp_hold_steps", 60))
        self.RELEASE_HOLD_STEPS = int(cfg.get("release_hold_steps", 40))

        self.get_logger().info(
            f"Task manager @ {self.CONTROL_HZ:.0f} Hz (axis-separated XY / Z)"
        )

        self.create_subscription(PoseArray, '/percept/ball_poses', self.balls_callback, 10)
        self.create_subscription(Pose, '/percept/box_pose', self.box_callback, 10)
        self.create_subscription(Pose, '/robot/eef_pose', self.eef_callback, 10)
        self.pub_action = self.create_publisher(Float32MultiArray, '/robot/cmd_action', 10)
        self.pub_target = self.create_publisher(Pose, '/robot/target_pose', 10)

        self.balls = []
        self.box_pose = None
        self.eef_pose = None

        self.state = "INIT"
        self.target_ball_idx = 0
        # 0 Z-up / 1 XY-to-ball / 2 Z-down / 3 grasp
        # 4 Z-lift / 5 XY-to-box / 6 release
        self.sub_step = 0
        self.step_counter = 0
        self.stable_count = 0
        self.gripper = -1.0

        self.create_timer(1.0 / self.CONTROL_HZ, self.control_loop)

    def balls_callback(self, msg):
        if self.state == "INIT" and len(msg.poses) > 0:
            self.balls = sorted(list(msg.poses), key=lambda p: p.position.x)

    def box_callback(self, msg):
        if self.state == "INIT":
            self.box_pose = msg

    def eef_callback(self, msg):
        self.eef_pose = msg

    def send_action(self, dx, dy, dz):
        msg = Float32MultiArray()
        msg.data = [float(dx), float(dy), float(dz), 0.0, 0.0, 0.0, float(self.gripper)]
        self.pub_action.publish(msg)

    def publish_target(self, x, y, z):
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        self.pub_target.publish(msg)

    def _cmd_max(self, dist):
        if dist > self.NEAR_DIST:
            return self.CMD_MAX_FAR
        alpha = dist / self.NEAR_DIST
        return self.CMD_MAX_NEAR + alpha * (self.CMD_MAX_FAR - self.CMD_MAX_NEAR)

    def _axis_cmd(self, err_vec):
        """Scale a 3-vector error (m) to OSC cmd in [-1, 1]."""
        err = np.asarray(err_vec, dtype=np.float64)
        dist = float(np.linalg.norm(err))
        if dist < 1e-9:
            return np.zeros(3), 0.0
        raw = err * self.Kp
        lim = self._cmd_max(dist)
        n = float(np.linalg.norm(raw))
        if n > lim:
            raw = raw * (lim / n)
        return np.clip(raw, -1.0, 1.0), dist

    def _cmd_xy_only(self, target_xy, cur):
        err = np.array([target_xy[0] - cur[0], target_xy[1] - cur[1], 0.0])
        cmd, dist = self._axis_cmd(err)
        return float(cmd[0]), float(cmd[1]), 0.0, dist

    def _cmd_z_only(self, target_z, cur):
        err = np.array([0.0, 0.0, target_z - cur[2]])
        cmd, dist = self._axis_cmd(err)
        return 0.0, 0.0, float(cmd[2]), dist

    def _advance_if_stable(self, ok):
        if ok:
            self.stable_count += 1
        else:
            self.stable_count = 0
        if self.stable_count >= self.STABLE_NEED:
            self.stable_count = 0
            return True
        return False

    def control_loop(self):
        if self.eef_pose is None:
            return

        if self.state == "INIT":
            self.gripper = -1.0
            self.send_action(0.0, 0.0, 0.0)
            if len(self.balls) >= 3 and self.box_pose is not None:
                self.get_logger().info("Locked 3 balls + box; starting pick sequence")
                self.state = "EXECUTE_TASK"
                self.sub_step = 0
                self.stable_count = 0
            return

        if self.state == "FINISHED":
            self.gripper = -1.0
            self.send_action(0.0, 0.0, 0.0)
            return

        if self.target_ball_idx >= len(self.balls):
            self.get_logger().info("All balls placed; finished")
            self.state = "FINISHED"
            return

        ball = self.balls[self.target_ball_idx]
        self.step_counter += 1

        cur = np.array([
            self.eef_pose.position.x,
            self.eef_pose.position.y,
            self.eef_pose.position.z,
        ])
        ball_xy = np.array([
            ball.position.x + self.TCP_OFFSET_X,
            ball.position.y + self.TCP_OFFSET_Y,
        ])
        ball_z = float(ball.position.z)
        travel_z = self.TRAVEL_Z
        box_xy = np.array([self.box_pose.position.x, self.box_pose.position.y])

        if self.sub_step == 0:
            self.gripper = -1.0
            self.publish_target(cur[0], cur[1], travel_z)
            dx, dy, dz, _ = self._cmd_z_only(travel_z, cur)
            self.send_action(dx, dy, dz)
            if self._advance_if_stable(abs(travel_z - cur[2]) < self.Z_TOL):
                self.get_logger().info(
                    f"ball{self.target_ball_idx+1}: Z travel -> XY to ball"
                )
                self.sub_step = 1
                self.step_counter = 0

        elif self.sub_step == 1:
            self.gripper = -1.0
            self.publish_target(ball_xy[0], ball_xy[1], travel_z)
            dx, dy, dz, _ = self._cmd_xy_only(ball_xy, cur)
            self.send_action(dx, dy, dz)
            ok = (abs(ball_xy[0] - cur[0]) < self.XY_TOL
                  and abs(ball_xy[1] - cur[1]) < self.XY_TOL)
            if self._advance_if_stable(ok):
                self.get_logger().info(
                    f"ball{self.target_ball_idx+1}: XY above ball -> Z descend"
                )
                self.sub_step = 2
                self.step_counter = 0

        elif self.sub_step == 2:
            self.gripper = -1.0
            self.publish_target(ball_xy[0], ball_xy[1], ball_z)
            dx, dy, dz, _ = self._cmd_z_only(ball_z, cur)
            self.send_action(dx, dy, dz)
            if self._advance_if_stable(abs(ball_z - cur[2]) < self.Z_TOL):
                self.get_logger().info(
                    f"ball{self.target_ball_idx+1}: Z at ball -> grasp"
                )
                self.sub_step = 3
                self.step_counter = 0

        elif self.sub_step == 3:
            hold = np.array([ball_xy[0], ball_xy[1], ball_z])
            self.publish_target(*hold)
            self.gripper = 1.0
            self.send_action(0.0, 0.0, 0.0)
            if self.step_counter > self.GRASP_HOLD_STEPS:
                self.sub_step = 4
                self.step_counter = 0
                self.stable_count = 0

        elif self.sub_step == 4:
            self.gripper = 1.0
            self.publish_target(cur[0], cur[1], travel_z)
            dx, dy, dz, _ = self._cmd_z_only(travel_z, cur)
            self.send_action(dx, dy, dz)
            if self._advance_if_stable(abs(travel_z - cur[2]) < self.Z_TOL):
                self.get_logger().info(
                    f"ball{self.target_ball_idx+1}: Z travel -> XY to box"
                )
                self.sub_step = 5
                self.step_counter = 0

        elif self.sub_step == 5:
            self.gripper = 1.0
            self.publish_target(box_xy[0], box_xy[1], travel_z)
            dx, dy, dz, _ = self._cmd_xy_only(box_xy, cur)
            self.send_action(dx, dy, dz)
            ok = (abs(box_xy[0] - cur[0]) < self.PLACE_XY_TOL
                  and abs(box_xy[1] - cur[1]) < self.PLACE_XY_TOL)
            if self._advance_if_stable(ok):
                self.get_logger().info(
                    f"ball{self.target_ball_idx+1}: XY above box -> release"
                )
                self.sub_step = 6
                self.step_counter = 0

        elif self.sub_step == 6:
            hold = np.array([box_xy[0], box_xy[1], travel_z])
            self.publish_target(*hold)
            self.gripper = -1.0
            self.send_action(0.0, 0.0, 0.0)
            if self.step_counter > self.RELEASE_HOLD_STEPS:
                self.get_logger().info(f"Placed ball {self.target_ball_idx + 1}")
                self.target_ball_idx += 1
                self.sub_step = 0
                self.step_counter = 0
                self.stable_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
