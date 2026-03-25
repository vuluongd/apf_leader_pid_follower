"""
Signal Simulator Node — Emulates RhOct visual pose estimation in 2D.

Reads ground-truth positions of leader (turtle1) and follower (turtle2),
determines if visual signal is available based on:
  1. Range: ||p_leader - p_follower|| <= d_max
  2. FoV: angle between relative vector and follower heading <= fov_half_angle
  3. (Optional) Occlusion: no obstacle between leader and follower

If signal available: publishes noisy relative pose with configurable delay.
If signal lost: publishes signal_lost status.
"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64
import math
import numpy as np
from collections import deque


class SignalSimulatorNode(Node):
    def __init__(self):
        super().__init__('signal_simulator')

        # Parameters
        self.declare_parameter('d_max', 4.0)
        self.declare_parameter('fov_half_angle', 45.0)
        self.declare_parameter('delay_h', 0.25)
        self.declare_parameter('noise_sigma_x', 0.01)
        self.declare_parameter('noise_sigma_y', 0.01)
        self.declare_parameter('force_signal_loss', False)

        # State
        self.leader_pose: Pose | None = None
        self.follower_pose: Pose | None = None
        self.signal_lost_since: float | None = None  # timestamp when signal was lost
        self.delay_buffer: deque = deque()  # buffer for simulating delay

        # Subscribers
        self.create_subscription(Pose, '/turtle1/pose', self._leader_cb, 10)
        self.create_subscription(Pose, '/turtle2/pose', self._follower_cb, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/follower/relative_pose', 10)
        self.signal_pub = self.create_publisher(Bool, '/follower/signal_status', 10)
        self.duration_pub = self.create_publisher(Float64, '/follower/signal_loss_duration', 10)

        # Timer at 20 Hz
        self.create_timer(0.05, self._process)
        self.get_logger().info('[SignalSimulator] Started')

    def _leader_cb(self, msg: Pose):
        self.leader_pose = msg

    def _follower_cb(self, msg: Pose):
        self.follower_pose = msg

    def _check_signal(self) -> bool:
        """Check if visual signal is available based on range and FoV."""
        if self.get_parameter('force_signal_loss').value:
            return False

        if self.leader_pose is None or self.follower_pose is None:
            return False

        d_max = self.get_parameter('d_max').value
        fov_half = math.radians(self.get_parameter('fov_half_angle').value)

        # Relative position
        dx = self.leader_pose.x - self.follower_pose.x
        dy = self.leader_pose.y - self.follower_pose.y
        dist = math.hypot(dx, dy)

        # Range check
        if dist > d_max:
            return False

        # FoV check: angle between (dx, dy) and follower heading direction
        rel_angle = math.atan2(dy, dx)
        angle_diff = abs(math.atan2(
            math.sin(rel_angle - self.follower_pose.theta),
            math.cos(rel_angle - self.follower_pose.theta)
        ))

        if angle_diff > fov_half:
            return False

        return True

    def _process(self):
        if self.leader_pose is None or self.follower_pose is None:
            return

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        delay_h = self.get_parameter('delay_h').value

        sigma = self._check_signal()

        # Publish signal status
        self.signal_pub.publish(Bool(data=sigma))

        # Track signal loss duration
        if not sigma:
            if self.signal_lost_since is None:
                self.signal_lost_since = now_sec
            tau = now_sec - self.signal_lost_since
        else:
            if self.signal_lost_since is not None:
                tau_final = now_sec - self.signal_lost_since
                self.get_logger().info(
                    f'[SignalSim] Signal recovered after {tau_final:.2f}s'
                )
            self.signal_lost_since = None
            tau = 0.0

        self.duration_pub.publish(Float64(data=tau))

        # If signal available, add to delay buffer
        if sigma:
            noise_x = np.random.normal(0, self.get_parameter('noise_sigma_x').value)
            noise_y = np.random.normal(0, self.get_parameter('noise_sigma_y').value)

            rel_pose = PoseStamped()
            rel_pose.header.stamp = now.to_msg()
            rel_pose.pose.position.x = (
                self.leader_pose.x - self.follower_pose.x + noise_x
            )
            rel_pose.pose.position.y = (
                self.leader_pose.y - self.follower_pose.y + noise_y
            )
            rel_pose.pose.position.z = 0.0

            # Store leader heading for reference
            rel_pose.pose.orientation.z = math.sin(self.leader_pose.theta / 2.0)
            rel_pose.pose.orientation.w = math.cos(self.leader_pose.theta / 2.0)

            self.delay_buffer.append((now_sec, rel_pose))

        # Output delayed measurements
        while self.delay_buffer:
            t_stored, pose_stored = self.delay_buffer[0]
            if now_sec - t_stored >= delay_h:
                self.delay_buffer.popleft()
                if sigma:  # only publish if still in signal
                    self.pose_pub.publish(pose_stored)
            else:
                break


def main():
    rclpy.init()
    node = SignalSimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
