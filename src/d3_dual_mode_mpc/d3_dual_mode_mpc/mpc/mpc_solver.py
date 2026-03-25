"""
MPC solver for 2D follower tracking using scipy quadratic programming.

Solves the finite-horizon optimal control problem:
    min  Σ_{j=0}^{N-1} [||e(k+j)||²_Q + ||u(k+j)||²_R] + ||e(k+N)||²_P

    subject to:
        x(k+j+1) = A_d x(k+j) + B_d u(k+j)
        u_min ≤ u(k+j) ≤ u_max

State: [px, py, vx, vy]  (4)
Input: [ax, ay]           (2)
"""

import numpy as np
from scipy.optimize import minimize


class MPCSolver:
    """Receding-horizon MPC for 2D double-integrator tracking."""

    def __init__(self, A_d: np.ndarray, B_d: np.ndarray,
                 Q_diag: np.ndarray, R_diag: np.ndarray,
                 P: np.ndarray, N: int,
                 u_max: float = 5.0):
        """
        Args:
            A_d: Discrete state matrix (4x4)
            B_d: Discrete input matrix (4x2)
            Q_diag: Diagonal of Q matrix (4,)
            R_diag: Diagonal of R matrix (2,)
            P: Terminal cost matrix (4x4)
            N: Prediction horizon
            u_max: Max acceleration magnitude per axis
        """
        self.A = A_d
        self.B = B_d
        self.Q_base = np.diag(Q_diag)
        self.R = np.diag(R_diag)
        self.P = P
        self.N = N
        self.nx = A_d.shape[0]  # 4
        self.nu = B_d.shape[1]  # 2
        self.u_max = u_max

        # Warm start
        self._last_solution = None

    def solve(self, x0: np.ndarray, x_ref_trajectory: np.ndarray,
              q_multiplier: float = 1.0) -> np.ndarray:
        """
        Solve MPC for one step.

        Args:
            x0: Current state [px, py, vx, vy] (4,)
            x_ref_trajectory: Reference trajectory (N+1, 4) — one ref per step
            q_multiplier: Q(τ) multiplier ∈ [Q_floor, 1.0]

        Returns:
            u_opt: Optimal first input [ax, ay] (2,)
        """
        N = self.N
        nu = self.nu

        Q = self.Q_base * q_multiplier
        P_term = self.P * q_multiplier

        # Decision variable: U = [u(0), u(1), ..., u(N-1)] flattened (N*nu,)
        n_vars = N * nu

        # Bounds
        bounds = [(-self.u_max, self.u_max)] * n_vars

        # Initial guess (warm start)
        if self._last_solution is not None and len(self._last_solution) == n_vars:
            u0 = self._last_solution.copy()
            # Shift: drop first input, append zero
            u0[:-nu] = u0[nu:]
            u0[-nu:] = 0.0
        else:
            u0 = np.zeros(n_vars)

        def cost_and_grad(U_flat):
            """Compute cost and gradient of the QP."""
            U = U_flat.reshape(N, nu)
            cost = 0.0
            grad = np.zeros_like(U)

            # Forward simulate
            x = x0.copy()
            xs = [x.copy()]
            for j in range(N):
                x = self.A @ x + self.B @ U[j]
                xs.append(x.copy())

            # Stage costs + gradients (backward pass for efficiency)
            # Terminal cost
            e_N = xs[N] - x_ref_trajectory[min(N, len(x_ref_trajectory) - 1)]
            cost += e_N @ P_term @ e_N

            # Backward accumulate dL/dx
            dlam = 2.0 * P_term @ e_N  # dL/dx_N

            for j in range(N - 1, -1, -1):
                # Stage cost
                e_j = xs[j] - x_ref_trajectory[min(j, len(x_ref_trajectory) - 1)]
                cost += e_j @ Q @ e_j + U[j] @ self.R @ U[j]

                # Gradient w.r.t. u(j)
                grad[j] = 2.0 * self.R @ U[j] + self.B.T @ dlam

                # Propagate adjoint backward
                dlam = 2.0 * Q @ e_j + self.A.T @ dlam

            return cost, grad.flatten()

        result = minimize(
            cost_and_grad, u0, method='L-BFGS-B',
            jac=True, bounds=bounds,
            options={'maxiter': 50, 'ftol': 1e-6}
        )

        self._last_solution = result.x.copy()

        # Return first input
        u_opt = result.x[:nu]
        return u_opt

    def build_reference_trajectory(self, ref_pos: np.ndarray,
                                   ref_vel: np.ndarray,
                                   N: int) -> np.ndarray:
        """
        Build a constant reference trajectory for the prediction horizon.

        Args:
            ref_pos: Reference position [px, py] (2,)
            ref_vel: Reference velocity [vx, vy] (2,) — typically zeros for formation

        Returns:
            x_ref: Reference trajectory (N+1, 4)
        """
        x_ref = np.zeros((N + 1, 4))
        x_ref[:, 0] = ref_pos[0]
        x_ref[:, 1] = ref_pos[1]
        x_ref[:, 2] = ref_vel[0]
        x_ref[:, 3] = ref_vel[1]
        return x_ref
