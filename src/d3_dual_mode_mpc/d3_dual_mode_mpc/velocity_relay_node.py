"""
Velocity Relay Node — Simulates RF velocity broadcast from leader to follower.

Relays /leader/velocity -> /leader/velocity_rf with configurable delay
to simulate RF communication latency (Case B only).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from collections import deque


class VelocityRelayNode(Node):
    def __init__(self):
        super().__init__('velocity_relay')

        self.declare_parameter('rf_delay_ms', 50)

        self.buffer: deque = deque()

        self.create_subscription(
            Vector3Stamped, '/leader/velocity', self._vel_cb, 10
        )
        self.pub = self.create_publisher(
            Vector3Stamped, '/leader/velocity_rf', 10
        )

        self.create_timer(0.01, self._relay)  # 100 Hz check
        self.get_logger().info('[VelocityRelay] Started')

    def _vel_cb(self, msg: Vector3Stamped):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.buffer.append((now, msg))

    def _relay(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        delay = self.get_parameter('rf_delay_ms').value / 1000.0

        while self.buffer:
            t_stored, msg = self.buffer[0]
            if now - t_stored >= delay:
                self.buffer.popleft()
                self.pub.publish(msg)
            else:
                break


def main():
    rclpy.init()
    node = VelocityRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
