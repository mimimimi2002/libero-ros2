#!/usr/bin/env python3
"""
Task manager for pick-and-place tidy-up.

Control gains and clearances are loaded from config/control.json.
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

        self.tcp_offset_x = float(cfg["tcp_offset_xy"][0])
        self.tcp_offset_y = float(cfg["tcp_offset_xy"][1])
        self.place_z = float(cfg["place_z"])
        self.hover_clearance = float(cfg["hover_clearance"])
        self.lift_clearance = float(cfg["lift_clearance"])
        self.xy_tol = float(cfg["xy_tol"])
        self.z_tol = float(cfg["z_tol"])
        self.place_xy_tol = float(cfg["place_xy_tol"])
        self.stable_need = int(cfg["stable_need"])
        self.kp = float(cfg["kp"])
        self.v_max_far = float(cfg["v_max_far"])
        self.v_max_near = float(cfg["v_max_near"])
        self.near_dist = float(cfg["near_dist"])
        self.grasp_hold_steps = int(cfg["grasp_hold_steps"])
        self.release_hold_steps = int(cfg["release_hold_steps"])
        control_period = float(cfg["control_period"])

        self.get_logger().info("Task manager ready (config/control.json)")

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
        # 0 hover / 1 descend / 2 grasp / 3 lift / 4 to box / 5 release
        self.sub_step = 0
        self.step_counter = 0
        self.stable_count = 0
        self.gripper = -1.0  # -1 open, +1 close

        self.create_timer(control_period, self.control_loop)

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

    def _vmax(self, dist):
        if dist > self.near_dist:
            return self.v_max_far
        alpha = dist / self.near_dist
        return self.v_max_near + alpha * (self.v_max_far - self.v_max_near)

    def _smooth_cmd(self, target, current):
        err = np.asarray(target, dtype=np.float64) - np.asarray(current, dtype=np.float64)
        dist = float(np.linalg.norm(err))
        if dist < 1e-6:
            return 0.0, 0.0, 0.0, dist
        raw = err * self.kp
        lim = self._vmax(dist)
        scale = min(1.0, lim / (np.linalg.norm(raw) + 1e-9))
        cmd = raw * scale
        return float(cmd[0]), float(cmd[1]), float(cmd[2]), dist

    def _advance_if_stable(self, ok):
        if ok:
            self.stable_count += 1
        else:
            self.stable_count = 0
        if self.stable_count >= self.stable_need:
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
            self.get_logger().info("All balls placed")
            self.state = "FINISHED"
            return

        ball = self.balls[self.target_ball_idx]
        self.step_counter += 1

        cur = np.array([
            self.eef_pose.position.x,
            self.eef_pose.position.y,
            self.eef_pose.position.z,
        ])
        ball_c = np.array([
            ball.position.x + self.tcp_offset_x,
            ball.position.y + self.tcp_offset_y,
            ball.position.z,
        ])

        if self.sub_step == 0:
            self.gripper = -1.0
            target = np.array([ball_c[0], ball_c[1], ball_c[2] + self.hover_clearance])
            self.publish_target(*target)
            dx, dy, dz, _ = self._smooth_cmd(target, cur)
            self.send_action(dx, dy, dz)

            ok = (
                abs(target[0] - cur[0]) < self.xy_tol
                and abs(target[1] - cur[1]) < self.xy_tol
                and abs(target[2] - cur[2]) < self.z_tol
            )
            if self._advance_if_stable(ok):
                self.get_logger().info(
                    f"Ball {self.target_ball_idx + 1}: hover reached -> descend"
                )
                self.sub_step = 1
                self.step_counter = 0

        elif self.sub_step == 1:
            self.gripper = -1.0
            target = ball_c.copy()
            self.publish_target(*target)
            _, _, dz, _ = self._smooth_cmd(
                np.array([cur[0], cur[1], target[2]]), cur
            )
            self.send_action(0.0, 0.0, dz)

            at_target_z = abs(target[2] - cur[2]) < self.z_tol
            if self._advance_if_stable(at_target_z):
                self.get_logger().info(
                    f"Ball {self.target_ball_idx + 1}: target z reached "
                    f"(z={cur[2]:.3f}, target_z={target[2]:.3f}) -> grasp"
                )
                self.sub_step = 2
                self.step_counter = 0

        elif self.sub_step == 2:
            self.publish_target(*ball_c)
            self.gripper = 1.0
            self.send_action(0.0, 0.0, 0.0)
            if self.step_counter > self.grasp_hold_steps:
                self.sub_step = 3
                self.step_counter = 0
                self.stable_count = 0

        elif self.sub_step == 3:
            self.gripper = 1.0
            target = np.array([ball_c[0], ball_c[1], ball_c[2] + self.lift_clearance])
            self.publish_target(*target)
            _, _, dz, _ = self._smooth_cmd(
                np.array([cur[0], cur[1], target[2]]), cur
            )
            self.send_action(0.0, 0.0, dz)

            ok = abs(target[2] - cur[2]) < 0.025
            if self._advance_if_stable(ok):
                self.sub_step = 4
                self.step_counter = 0

        elif self.sub_step == 4:
            self.gripper = 1.0
            target = np.array([
                self.box_pose.position.x,
                self.box_pose.position.y,
                self.place_z,
            ])
            self.publish_target(*target)
            dx, dy, dz, _ = self._smooth_cmd(target, cur)
            self.send_action(dx, dy, dz)

            ok = (
                abs(target[0] - cur[0]) < self.place_xy_tol
                and abs(target[1] - cur[1]) < self.place_xy_tol
            )
            if self._advance_if_stable(ok):
                self.sub_step = 5
                self.step_counter = 0

        elif self.sub_step == 5:
            self.publish_target(
                self.box_pose.position.x, self.box_pose.position.y, self.place_z
            )
            self.gripper = -1.0
            self.send_action(0.0, 0.0, 0.0)
            if self.step_counter > self.release_hold_steps:
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
