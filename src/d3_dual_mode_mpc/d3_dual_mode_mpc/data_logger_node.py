"""
Data Logger Node — Logs simulation data to CSV for post-processing.

Records: timestamp, leader pos, follower pos, tracking error,
signal status, signal loss duration, Q multiplier, controller mode.
"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, Float64, String
import os
import csv
from datetime import datetime


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger')

        self.declare_parameter('output_dir', '/tmp/d3_logs')
        self.declare_parameter('log_interval_ms', 50)

        output_dir = self.get_parameter('output_dir').value
        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(output_dir, f'run_{timestamp}.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'time_s',
            'leader_x', 'leader_y', 'leader_theta',
            'follower_x', 'follower_y', 'follower_theta',
            'tracking_error',
            'signal_available',
            'signal_loss_duration',
            'q_multiplier',
            'controller_mode',
            'leader_vx', 'leader_vy',
        ])

        # State
        self.leader_pose: Pose | None = None
        self.follower_pose: Pose | None = None
        self.tracking_error = 0.0
        self.signal_available = True
        self.signal_loss_duration = 0.0
        self.q_multiplier = 1.0
        self.controller_mode = 'unknown'
        self.leader_vel = (0.0, 0.0)
        self.start_time = None

        # Subscribers
        self.create_subscription(Pose, '/turtle1/pose', self._leader_cb, 10)
        self.create_subscription(Pose, '/turtle2/pose', self._follower_cb, 10)
        self.create_subscription(Float64, '/follower/tracking_error', self._error_cb, 10)
        self.create_subscription(Bool, '/follower/signal_status', self._signal_cb, 10)
        self.create_subscription(Float64, '/follower/signal_loss_duration', self._duration_cb, 10)
        self.create_subscription(Float64, '/follower/q_multiplier', self._q_cb, 10)
        self.create_subscription(String, '/follower/current_mode', self._mode_cb, 10)
        self.create_subscription(Vector3Stamped, '/leader/velocity', self._vel_cb, 10)

        interval = self.get_parameter('log_interval_ms').value / 1000.0
        self.create_timer(interval, self._log)

        self.get_logger().info(f'[DataLogger] Logging to {self.csv_path}')

    def _leader_cb(self, msg: Pose):
        self.leader_pose = msg

    def _follower_cb(self, msg: Pose):
        self.follower_pose = msg

    def _error_cb(self, msg: Float64):
        self.tracking_error = msg.data

    def _signal_cb(self, msg: Bool):
        self.signal_available = msg.data

    def _duration_cb(self, msg: Float64):
        self.signal_loss_duration = msg.data

    def _q_cb(self, msg: Float64):
        self.q_multiplier = msg.data

    def _mode_cb(self, msg: String):
        self.controller_mode = msg.data

    def _vel_cb(self, msg: Vector3Stamped):
        self.leader_vel = (msg.vector.x, msg.vector.y)

    def _log(self):
        if self.leader_pose is None or self.follower_pose is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now
        t = now - self.start_time

        self.writer.writerow([
            f'{t:.3f}',
            f'{self.leader_pose.x:.4f}', f'{self.leader_pose.y:.4f}',
            f'{self.leader_pose.theta:.4f}',
            f'{self.follower_pose.x:.4f}', f'{self.follower_pose.y:.4f}',
            f'{self.follower_pose.theta:.4f}',
            f'{self.tracking_error:.4f}',
            int(self.signal_available),
            f'{self.signal_loss_duration:.3f}',
            f'{self.q_multiplier:.4f}',
            self.controller_mode,
            f'{self.leader_vel[0]:.4f}', f'{self.leader_vel[1]:.4f}',
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(f'[DataLogger] Saved {self.csv_path}')
        super().destroy_node()


def main():
    rclpy.init()
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
