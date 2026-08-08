#!/usr/bin/env python3
"""
Debug node: compare perception estimates (/percept/*) against sim GT (/gt/*).

Subscribes:
  /percept/ball_poses, /percept/box_pose
  /gt/ball_poses, /gt/box_pose

Publishes:
  /debug/ball_pose_errors   PoseArray  position=(ex,ey,ez)= est - gt [m]
  /debug/box_pose_error     Pose       position=(ex,ey,ez)
  /debug/ball_error_stats   Float32MultiArray
      [n_matched, mean_xy, mean_3d, max_xy, max_3d]  (meters)
  /debug/box_error_stats    Float32MultiArray
      [ex, ey, ez, err_xy, err_3d]  (meters)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray
import numpy as np

from config_loader import load_perception_config


class PerceptErrorDebugNode(Node):
    def __init__(self):
        super().__init__('percept_error_debug_node')
        perc_cfg = load_perception_config()
        self.MATCH_THRESH_XY = float(
            perc_cfg.get("debug", {}).get("match_thresh_xy", 0.12)
        )
        self.get_logger().info("Percept error debug started (/percept vs /gt)...")

        self.est_balls = []
        self.gt_balls = []
        self.est_box = None
        self.gt_box = None

        self.create_subscription(PoseArray, '/percept/ball_poses', self.est_balls_cb, 10)
        self.create_subscription(PoseArray, '/gt/ball_poses', self.gt_balls_cb, 10)
        self.create_subscription(Pose, '/percept/box_pose', self.est_box_cb, 10)
        self.create_subscription(Pose, '/gt/box_pose', self.gt_box_cb, 10)

        self.pub_ball_errors = self.create_publisher(PoseArray, '/debug/ball_pose_errors', 10)
        self.pub_box_error = self.create_publisher(Pose, '/debug/box_pose_error', 10)
        self.pub_ball_stats = self.create_publisher(Float32MultiArray, '/debug/ball_error_stats', 10)
        self.pub_box_stats = self.create_publisher(Float32MultiArray, '/debug/box_error_stats', 10)

        self.create_timer(0.05, self.compare_loop)  # 20 Hz compare
        self.create_timer(1.0, self.log_summary)    # 1 Hz log

        self._last_ball_stats = None
        self._last_box_stats = None

    def est_balls_cb(self, msg):
        self.est_balls = list(msg.poses)

    def gt_balls_cb(self, msg):
        self.gt_balls = list(msg.poses)

    def est_box_cb(self, msg):
        self.est_box = msg

    def gt_box_cb(self, msg):
        self.gt_box = msg

    @staticmethod
    def _xyz(pose):
        return np.array(
            [pose.position.x, pose.position.y, pose.position.z], dtype=np.float64
        )

    def _match_pairs(self, gt_poses, est_poses):
        """Nearest-neighbor match in XY. Returns list of (gt_xyz, est_xyz)."""
        pairs = []
        used = set()
        for gt in gt_poses:
            g = self._xyz(gt)
            best_j, best_d = None, 1e9
            for j, est in enumerate(est_poses):
                if j in used:
                    continue
                e = self._xyz(est)
                d = np.linalg.norm(e[:2] - g[:2])
                if d < best_d:
                    best_d, best_j = d, j
            if best_j is not None and best_d <= self.MATCH_THRESH_XY:
                pairs.append((g, self._xyz(est_poses[best_j])))
                used.add(best_j)
        return pairs

    def compare_loop(self):
        # ---- balls ----
        if self.gt_balls and self.est_balls:
            pairs = self._match_pairs(self.gt_balls, self.est_balls)
            err_msg = PoseArray()
            err_msg.header.stamp = self.get_clock().now().to_msg()
            err_msg.header.frame_id = "world"

            xy_list, d3_list = [], []
            for g, e in pairs:
                err = e - g
                p = Pose()
                p.position.x = float(err[0])
                p.position.y = float(err[1])
                p.position.z = float(err[2])
                err_msg.poses.append(p)
                xy_list.append(float(np.linalg.norm(err[:2])))
                d3_list.append(float(np.linalg.norm(err)))

            self.pub_ball_errors.publish(err_msg)

            n = len(pairs)
            if n > 0:
                stats = Float32MultiArray()
                stats.data = [
                    float(n),
                    float(np.mean(xy_list)),
                    float(np.mean(d3_list)),
                    float(np.max(xy_list)),
                    float(np.max(d3_list)),
                ]
                self.pub_ball_stats.publish(stats)
                self._last_ball_stats = stats.data
            else:
                self._last_ball_stats = None

        # ---- box ----
        if self.gt_box is not None and self.est_box is not None:
            g = self._xyz(self.gt_box)
            e = self._xyz(self.est_box)
            err = e - g

            box_err = Pose()
            box_err.position.x = float(err[0])
            box_err.position.y = float(err[1])
            box_err.position.z = float(err[2])
            self.pub_box_error.publish(box_err)

            err_xy = float(np.linalg.norm(err[:2]))
            err_3d = float(np.linalg.norm(err))
            stats = Float32MultiArray()
            stats.data = [
                float(err[0]), float(err[1]), float(err[2]), err_xy, err_3d
            ]
            self.pub_box_stats.publish(stats)
            self._last_box_stats = stats.data

    def log_summary(self):
        parts = []
        if self._last_ball_stats is not None:
            n, mean_xy, mean_3d, max_xy, max_3d = self._last_ball_stats
            parts.append(
                f"balls n={int(n)} mean_xy={mean_xy*1000:.1f}mm "
                f"mean_3d={mean_3d*1000:.1f}mm max_xy={max_xy*1000:.1f}mm"
            )
        else:
            parts.append("balls: no match")

        if self._last_box_stats is not None:
            ex, ey, ez, err_xy, err_3d = self._last_box_stats
            parts.append(
                f"box dx={ex*1000:.1f} dy={ey*1000:.1f} dz={ez*1000:.1f} mm "
                f"|xy|={err_xy*1000:.1f}mm"
            )
        else:
            parts.append("box: waiting")

        self.get_logger().info(" | ".join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = PerceptErrorDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
