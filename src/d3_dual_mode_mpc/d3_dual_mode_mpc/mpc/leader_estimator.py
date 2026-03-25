"""
Leader state estimator with dead reckoning for signal-loss prediction.

Handles:
- Visual mode (σ=1): direct measurement
- Prediction mode Case A (RF-free): constant-velocity extrapolation
- Prediction mode Case B (RF-assisted): velocity-updated extrapolation
- Q(τ) exponential decay computation
"""

import numpy as np


class LeaderEstimator:
    """Estimates leader position under intermittent visual signal."""

    def __init__(self, offset: np.ndarray, lambda_decay: float,
                 Q_floor: float, delta_reacq: float,
                 alpha_reset: float, T_ramp_steps: int):
        """
        Args:
            offset: Formation offset [cx, cy] (follower desired pos relative to leader)
            lambda_decay: Exponential decay rate for Q in prediction mode
            Q_floor: Minimum Q multiplier (prevents Q from going to 0)
            delta_reacq: Consistency threshold for re-acquisition
            alpha_reset: Q reset factor on inconsistent re-acquisition
            T_ramp_steps: Steps to ramp Q back to 1.0 after inconsistent re-acquisition
        """
        self.offset = np.array(offset, dtype=float)
        self.lambda_decay = lambda_decay
        self.Q_floor = Q_floor
        self.delta_reacq = delta_reacq
        self.alpha_reset = alpha_reset
        self.T_ramp_steps = T_ramp_steps

        # Internal state
        self.signal_available = False
        self.leader_pos_hat = np.zeros(2)       # estimated leader position
        self.leader_vel_hat = np.zeros(2)       # estimated/broadcast leader velocity
        self.last_known_pos = np.zeros(2)
        self.last_known_vel = np.zeros(2)
        self.k_lost = 0                         # step when signal was lost
        self.current_step = 0
        self.q_multiplier = 1.0                 # current Q(τ) multiplier
        self.ramping = False
        self.ramp_step = 0

    def update_visual(self, leader_pos: np.ndarray,
                      leader_vel: np.ndarray) -> None:
        """Update with direct visual measurement (σ=1)."""
        self.leader_pos_hat = leader_pos.copy()
        self.leader_vel_hat = leader_vel.copy()
        self.last_known_pos = leader_pos.copy()
        self.last_known_vel = leader_vel.copy()
        self.signal_available = True
        self.current_step += 1

        # Q recovery
        if self.ramping:
            self.ramp_step += 1
            progress = min(1.0, self.ramp_step / self.T_ramp_steps)
            self.q_multiplier = self.alpha_reset + (1.0 - self.alpha_reset) * progress
            if progress >= 1.0:
                self.ramping = False
                self.q_multiplier = 1.0
        else:
            self.q_multiplier = 1.0

    def update_prediction_case_a(self, T_s: float) -> None:
        """
        Update leader estimate using constant-velocity dead reckoning (RF-free).
        Called when σ=0 and no RF broadcast available.
        """
        if self.signal_available:
            # Just lost signal
            self.k_lost = self.current_step
            self.signal_available = False

        tau = self.current_step - self.k_lost
        self.leader_pos_hat = (self.last_known_pos +
                               self.last_known_vel * tau * T_s)
        self._update_q_decay(tau)
        self.current_step += 1

    def update_prediction_case_b(self, leader_vel_broadcast: np.ndarray,
                                 T_s: float) -> None:
        """
        Update leader estimate using broadcast velocity (RF-assisted).
        Called when σ=0 but RF velocity is available.
        """
        if self.signal_available:
            self.k_lost = self.current_step
            self.signal_available = False

        self.leader_vel_hat = leader_vel_broadcast.copy()
        self.leader_pos_hat = self.leader_pos_hat + leader_vel_broadcast * T_s

        tau = self.current_step - self.k_lost
        self._update_q_decay(tau)
        self.current_step += 1

    def attempt_reacquisition(self, measured_pos: np.ndarray,
                              leader_vel: np.ndarray) -> bool:
        """
        Attempt to re-acquire visual tracking.
        Returns True if re-acquisition is consistent, False if inconsistent.
        """
        error = np.linalg.norm(measured_pos - self.leader_pos_hat)

        if error <= self.delta_reacq:
            # Consistent — smooth transition back
            self.update_visual(measured_pos, leader_vel)
            self.q_multiplier = 1.0
            self.ramping = False
            return True
        else:
            # Inconsistent — ramp Q gradually
            self.update_visual(measured_pos, leader_vel)
            self.q_multiplier = self.alpha_reset
            self.ramping = True
            self.ramp_step = 0
            return False

    def get_reference_position(self) -> np.ndarray:
        """Get the desired follower position = estimated leader pos + offset."""
        return self.leader_pos_hat + self.offset

    def get_q_multiplier(self) -> float:
        """Get current Q(τ) multiplier ∈ [Q_floor, 1.0]."""
        return self.q_multiplier

    def get_tau(self) -> float:
        """Get elapsed signal-loss steps (0 if visual mode)."""
        if self.signal_available:
            return 0.0
        return float(self.current_step - self.k_lost)

    def _update_q_decay(self, tau: int) -> None:
        """Compute Q(τ) = max(Q_floor, exp(-λ·τ))."""
        q = np.exp(-self.lambda_decay * tau)
        self.q_multiplier = max(self.Q_floor, q)
