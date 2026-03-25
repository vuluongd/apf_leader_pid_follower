"""
MPC Follower Node — Dual-Mode MPC for turtlesim.

Modes:
  - 'hover': Baseline hover-and-wait (from Evangeliou et al.)
  - 'mpc_a': MPC with RF-free dead reckoning (Case A)
  - 'mpc_b': MPC with RF-assisted velocity broadcast (Case B)
  - 'mpc_visual': MPC with perfect signal (upper bound baseline)

In visual mode (σ=1): tracks leader using RhOct relative pose measurement.
In prediction mode (σ=0): uses dead reckoning with Q(τ) decay.
"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, PoseStamped, Vector3Stamped
from std_msgs.msg import Bool, Float64, String
import math
import numpy as np

from d3_dual_mode_mpc.mpc.quadrotor_model import get_discrete_model, get_terminal_cost
from d3_dual_mode_mpc.mpc.mpc_solver import MPCSolver
from d3_dual_mode_mpc.mpc.leader_estimator import LeaderEstimator


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class MPCFollowerNode(Node):
    def __init__(self):
        super().__init__('mpc_follower')

        # Parameters
        self.declare_parameter('controller_mode', 'mpc_b')
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', -2.0)
        self.declare_parameter('N_horizon', 10)
        self.declare_parameter('T_s', 0.05)
        self.declare_parameter('Q_pos', 10.0)
        self.declare_parameter('Q_vel', 1.0)
        self.declare_parameter('R_input', 0.1)
        self.declare_parameter('lambda_decay', 0.5)
        self.declare_parameter('Q_floor', 0.1)
        self.declare_parameter('delta_reacq', 0.8)
        self.declare_parameter('alpha_reset', 0.3)
        self.declare_parameter('T_ramp_steps', 20)
        self.declare_parameter('v_max', 2.0)
        self.declare_parameter('max_angular', 3.0)
        self.declare_parameter('kp_angular', 4.0)
        self.declare_parameter('d_safe', 0.5)

        # Get parameters
        mode = self.get_parameter('controller_mode').value
        T_s = self.get_parameter('T_s').value
        N = self.get_parameter('N_horizon').value
        Q_pos = self.get_parameter('Q_pos').value
        Q_vel = self.get_parameter('Q_vel').value
        R_inp = self.get_parameter('R_input').value
        offset = [
            self.get_parameter('offset_x').value,
            self.get_parameter('offset_y').value,
        ]

        self.mode = mode
        self.T_s = T_s
        self.N = N

        # Build dynamics model
        A_d, B_d = get_discrete_model(T_s)
        Q_diag = np.array([Q_pos, Q_pos, Q_vel, Q_vel])
        R_diag = np.array([R_inp, R_inp])
        P = get_terminal_cost(A_d, B_d, np.diag(Q_diag), np.diag(R_diag))

        # MPC solver
        self.mpc = MPCSolver(A_d, B_d, Q_diag, R_diag, P, N, u_max=5.0)

        # Leader estimator
        self.estimator = LeaderEstimator(
            offset=offset,
            lambda_decay=self.get_parameter('lambda_decay').value,
            Q_floor=self.get_parameter('Q_floor').value,
            delta_reacq=self.get_parameter('delta_reacq').value,
            alpha_reset=self.get_parameter('alpha_reset').value,
            T_ramp_steps=self.get_parameter('T_ramp_steps').value,
        )

        # State
        self.follower_pose: Pose | None = None
        self.follower_vel = np.zeros(2)     # estimated from pose differences
        self.last_follower_pos = None
        self.signal_available = False
        self.leader_rel_pose: PoseStamped | None = None
        self.leader_vel_broadcast = np.zeros(2)  # from RF (Case B)
        self.hover_pos: np.ndarray | None = None  # for hover strategy

        # Subscribers
        self.create_subscription(Pose, '/turtle2/pose', self._follower_cb, 10)
        self.create_subscription(PoseStamped, '/follower/relative_pose', self._rel_pose_cb, 10)
        self.create_subscription(Bool, '/follower/signal_status', self._signal_cb, 10)
        self.create_subscription(Vector3Stamped, '/leader/velocity_rf', self._vel_rf_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/follower/current_mode', 10)
        self.error_pub = self.create_publisher(Float64, '/follower/tracking_error', 10)
        self.q_pub = self.create_publisher(Float64, '/follower/q_multiplier', 10)

        # Timer at 20 Hz
        self.create_timer(T_s, self._control_loop)
        self.get_logger().info(f'[MPCFollower] Started in mode: {mode}')

    def _follower_cb(self, msg: Pose):
        pos = np.array([msg.x, msg.y])
        if self.last_follower_pos is not None:
            self.follower_vel = (pos - self.last_follower_pos) / self.T_s
        self.last_follower_pos = pos.copy()
        self.follower_pose = msg

    def _rel_pose_cb(self, msg: PoseStamped):
        self.leader_rel_pose = msg

    def _signal_cb(self, msg: Bool):
        self.signal_available = msg.data

    def _vel_rf_cb(self, msg: Vector3Stamped):
        self.leader_vel_broadcast = np.array([msg.vector.x, msg.vector.y])

    def _control_loop(self):
        if self.follower_pose is None:
            return

        follower_pos = np.array([self.follower_pose.x, self.follower_pose.y])

        # --- Estimate leader position ---
        if self.signal_available and self.leader_rel_pose is not None:
            # Visual mode
            leader_pos = follower_pos + np.array([
                self.leader_rel_pose.pose.position.x,
                self.leader_rel_pose.pose.position.y,
            ])
            # Estimate leader velocity from position change
            leader_vel = self.leader_vel_broadcast  # use broadcast if available
            self.estimator.attempt_reacquisition(leader_pos, leader_vel)
            current_mode = 'visual'
        else:
            # Signal lost
            if self.mode == 'mpc_a' or self.mode == 'mpc_visual':
                self.estimator.update_prediction_case_a(self.T_s)
                current_mode = 'prediction_a'
            elif self.mode == 'mpc_b':
                self.estimator.update_prediction_case_b(
                    self.leader_vel_broadcast, self.T_s
                )
                current_mode = 'prediction_b'
            elif self.mode == 'hover':
                current_mode = 'hover'
            else:
                current_mode = 'unknown'

        # --- Compute desired position ---
        if self.mode == 'hover':
            # Hover strategy: freeze position when signal lost
            if self.signal_available and self.leader_rel_pose is not None:
                leader_pos = follower_pos + np.array([
                    self.leader_rel_pose.pose.position.x,
                    self.leader_rel_pose.pose.position.y,
                ])
                offset = np.array([
                    self.get_parameter('offset_x').value,
                    self.get_parameter('offset_y').value,
                ])
                self.hover_pos = leader_pos + offset
                target_pos = self.hover_pos
            elif self.hover_pos is not None:
                target_pos = self.hover_pos  # hold last known position
            else:
                return  # no info yet

            # Simple P-controller toward target (replicates Evangeliou baseline)
            self._publish_p_control(follower_pos, target_pos)

        elif self.mode == 'mpc_visual':
            # MPC with forced visual (ignore signal loss for upper bound)
            if self.leader_rel_pose is not None:
                leader_pos = follower_pos + np.array([
                    self.leader_rel_pose.pose.position.x,
                    self.leader_rel_pose.pose.position.y,
                ])
                self.estimator.update_visual(leader_pos, self.leader_vel_broadcast)
            ref_pos = self.estimator.get_reference_position()
            self._publish_mpc_control(follower_pos, ref_pos, 1.0)

        else:
            # MPC-A or MPC-B: use estimator
            ref_pos = self.estimator.get_reference_position()
            q_mult = self.estimator.get_q_multiplier()
            self._publish_mpc_control(follower_pos, ref_pos, q_mult)

        # Publish diagnostics
        self.mode_pub.publish(String(data=current_mode))
        self.q_pub.publish(Float64(data=self.estimator.get_q_multiplier()))

        # Tracking error (from current follower pos to desired)
        ref_pos = self.estimator.get_reference_position()
        error = np.linalg.norm(follower_pos - ref_pos)
        self.error_pub.publish(Float64(data=error))

    def _publish_mpc_control(self, follower_pos: np.ndarray,
                             ref_pos: np.ndarray, q_mult: float):
        """Run MPC and publish velocity command."""
        # Current state: [px, py, vx, vy]
        x0 = np.array([
            follower_pos[0], follower_pos[1],
            self.follower_vel[0], self.follower_vel[1],
        ])

        # Build reference trajectory (constant target for simplicity)
        ref_vel = np.zeros(2)
        x_ref = self.mpc.build_reference_trajectory(ref_pos, ref_vel, self.N)

        # Solve MPC
        u_opt = self.mpc.solve(x0, x_ref, q_mult)

        # Convert acceleration command to turtlesim Twist
        self._send_acceleration_as_twist(u_opt, follower_pos, ref_pos)

    def _publish_p_control(self, follower_pos: np.ndarray,
                           target_pos: np.ndarray):
        """Simple P-controller (Evangeliou baseline hover strategy)."""
        v_max = self.get_parameter('v_max').value
        max_angular = self.get_parameter('max_angular').value
        kp_angular = self.get_parameter('kp_angular').value

        dx = target_pos[0] - follower_pos[0]
        dy = target_pos[1] - follower_pos[1]
        dist = math.hypot(dx, dy)

        desired_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(desired_angle - self.follower_pose.theta)

        cmd = Twist()
        cmd.angular.z = max(-max_angular, min(max_angular, kp_angular * angle_error))

        if abs(angle_error) < 0.5:
            cmd.linear.x = max(0.0, min(v_max, 1.5 * dist))
        else:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

    def _send_acceleration_as_twist(self, u_opt: np.ndarray,
                                    follower_pos: np.ndarray,
                                    ref_pos: np.ndarray):
        """
        Convert MPC acceleration output to turtlesim Twist.
        Turtlesim uses (linear.x, angular.z) — unicycle model.
        We map the 2D acceleration to desired direction + speed.
        """
        v_max = self.get_parameter('v_max').value
        max_angular = self.get_parameter('max_angular').value
        kp_angular = self.get_parameter('kp_angular').value

        # Desired velocity = current velocity + acceleration * T_s
        desired_vx = self.follower_vel[0] + u_opt[0] * self.T_s
        desired_vy = self.follower_vel[1] + u_opt[1] * self.T_s

        speed = math.hypot(desired_vx, desired_vy)
        desired_angle = math.atan2(desired_vy, desired_vx)
        angle_error = normalize_angle(desired_angle - self.follower_pose.theta)

        cmd = Twist()
        cmd.angular.z = max(-max_angular, min(max_angular, kp_angular * angle_error))

        if abs(angle_error) < 0.5:
            cmd.linear.x = max(0.0, min(v_max, speed))
        else:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = MPCFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
