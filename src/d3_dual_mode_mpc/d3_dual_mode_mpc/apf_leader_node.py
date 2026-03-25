"""
APF Leader Node — Artificial Potential Field path planner for turtlesim.

Drives turtle1 using APF: attractive force to goal + repulsive force from obstacles.
Publishes velocity for follower estimation (Case B).
"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3Stamped
import math
import random


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class APFLeaderNode(Node):
    def __init__(self):
        super().__init__('apf_leader')

        # Declare parameters
        self.declare_parameter('k_att', 1.0)
        self.declare_parameter('k_rep', 2.0)
        self.declare_parameter('rho_0', 2.0)
        self.declare_parameter('v_max', 2.0)
        self.declare_parameter('max_angular', 3.0)
        self.declare_parameter('kp_angular', 4.0)
        self.declare_parameter('goal_radius', 0.4)
        self.declare_parameter('epsilon_stuck', 0.05)
        self.declare_parameter('T_stuck', 3.0)
        self.declare_parameter('delta_rand', 0.3)
        self.declare_parameter('obstacle_names', ['obstacle1', 'obstacle2'])

        self.pose: Pose | None = None
        self.obstacle_poses: dict[str, Pose] = {}
        self.goal_x, self.goal_y = self._random_goal()

        # Stuck detection
        self.stuck_timer = 0.0
        self.last_force_mag = 0.0

        self.get_logger().info(
            f'[APFLeader] Goal -> ({self.goal_x:.2f}, {self.goal_y:.2f})'
        )

        # Subscribers
        self.create_subscription(Pose, '/turtle1/pose', self._leader_cb, 10)
        obstacle_names = self.get_parameter('obstacle_names').value
        for name in obstacle_names:
            self.create_subscription(
                Pose, f'/{name}/pose',
                lambda msg, n=name: self.obstacle_poses.__setitem__(n, msg),
                10,
            )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.vel_pub = self.create_publisher(Vector3Stamped, '/leader/velocity', 10)

        # Timer at 20 Hz (T_s = 0.05s)
        self.create_timer(0.05, self._control_loop)

    @staticmethod
    def _random_goal(margin: float = 1.5) -> tuple[float, float]:
        return (
            random.uniform(margin, 11.0 - margin),
            random.uniform(margin, 11.0 - margin),
        )

    def _leader_cb(self, msg: Pose):
        self.pose = msg

    def _attractive_force(self) -> tuple[float, float]:
        ka = self.get_parameter('k_att').value
        return (
            ka * (self.goal_x - self.pose.x),
            ka * (self.goal_y - self.pose.y),
        )

    def _repulsive_force(self) -> tuple[float, float]:
        kr = self.get_parameter('k_rep').value
        rho_0 = self.get_parameter('rho_0').value
        fx, fy = 0.0, 0.0

        for obs in self.obstacle_poses.values():
            dx = self.pose.x - obs.x
            dy = self.pose.y - obs.y
            rho = math.hypot(dx, dy)
            if 0.0 < rho < rho_0:
                coef = kr * (1.0 / rho - 1.0 / rho_0) / (rho ** 2)
                fx += coef * dx / rho
                fy += coef * dy / rho

        return fx, fy

    def _control_loop(self):
        if self.pose is None:
            return

        v_max = self.get_parameter('v_max').value
        max_angular = self.get_parameter('max_angular').value
        kp_angular = self.get_parameter('kp_angular').value
        goal_radius = self.get_parameter('goal_radius').value
        eps_stuck = self.get_parameter('epsilon_stuck').value
        T_stuck = self.get_parameter('T_stuck').value
        delta_rand = self.get_parameter('delta_rand').value

        # Check if reached goal
        dist_to_goal = math.hypot(
            self.goal_x - self.pose.x, self.goal_y - self.pose.y
        )
        if dist_to_goal < goal_radius:
            self.goal_x, self.goal_y = self._random_goal()
            self.get_logger().info(
                f'[APFLeader] New goal -> ({self.goal_x:.2f}, {self.goal_y:.2f})'
            )

        # Compute APF forces
        fax, fay = self._attractive_force()
        frx, fry = self._repulsive_force()
        fx, fy = fax + frx, fay + fry
        force_mag = math.hypot(fx, fy)

        # Local minima detection
        if force_mag < eps_stuck:
            self.stuck_timer += 0.05
            if self.stuck_timer > T_stuck:
                # Random perturbation
                angle = random.uniform(0, 2 * math.pi)
                fx += delta_rand * math.cos(angle)
                fy += delta_rand * math.sin(angle)
                force_mag = math.hypot(fx, fy)
                self.stuck_timer = 0.0
                self.get_logger().warn('[APFLeader] Local minima — random perturbation!')
        else:
            self.stuck_timer = 0.0

        # Convert force to velocity command
        desired_angle = math.atan2(fy, fx)
        angle_error = normalize_angle(desired_angle - self.pose.theta)

        cmd = Twist()
        cmd.angular.z = max(-max_angular, min(max_angular, kp_angular * angle_error))

        if abs(angle_error) < 0.5:
            cmd.linear.x = max(0.0, min(v_max, force_mag))
        else:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

        # Publish velocity in world frame for follower estimation
        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.vector.x = cmd.linear.x * math.cos(self.pose.theta)
        vel_msg.vector.y = cmd.linear.x * math.sin(self.pose.theta)
        vel_msg.vector.z = 0.0
        self.vel_pub.publish(vel_msg)


def main():
    rclpy.init()
    node = APFLeaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
