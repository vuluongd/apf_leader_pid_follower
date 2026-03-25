"""
Simplified 2D dynamics model for turtlesim-based prototype.

In turtlesim, each turtle has state [x, y, theta] and accepts
cmd_vel [linear.x, angular.z]. We model the follower as a
discrete-time double integrator in 2D for MPC:

State:  x = [px, py, vx, vy]^T   (4 states)
Input:  u = [ax, ay]^T            (2 inputs, mapped to cmd_vel later)

Continuous:
    dx/dt = [vx, vy, ax, ay]^T

Discrete (ZOH at T_s):
    x(k+1) = A_d x(k) + B_d u(k)
"""

import numpy as np


def get_discrete_model(T_s: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Returns (A_d, B_d) for the 2D double-integrator model.

    State: [px, py, vx, vy]
    Input: [ax, ay]
    """
    A_d = np.array([
        [1.0, 0.0, T_s, 0.0],
        [0.0, 1.0, 0.0, T_s],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])

    B_d = np.array([
        [0.5 * T_s**2, 0.0],
        [0.0, 0.5 * T_s**2],
        [T_s, 0.0],
        [0.0, T_s],
    ])

    return A_d, B_d


def get_terminal_cost(A_d: np.ndarray, B_d: np.ndarray,
                      Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """
    Solve the Discrete Algebraic Riccati Equation (DARE) for terminal cost P.
    P = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q
    """
    from scipy.linalg import solve_discrete_are
    P = solve_discrete_are(A_d, B_d, Q, R)
    return P
