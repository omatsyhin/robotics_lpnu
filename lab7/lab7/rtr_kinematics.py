from __future__ import annotations

import math
from typing import Tuple


def _matmul(a, b):
    """3x3 matrix multiplication."""
    return [
        [
            sum(a[i][k] * b[k][j] for k in range(3))
            for j in range(3)
        ]
        for i in range(3)
    ]


def _rotation_matrix_to_quaternion(R) -> Tuple[float, float, float, float]:
    """Convert 3x3 rotation matrix to quaternion (x, y, z, w)."""
    trace = R[0][0] + R[1][1] + R[2][2]

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2][1] - R[1][2]) / s
        qy = (R[0][2] - R[2][0]) / s
        qz = (R[1][0] - R[0][1]) / s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
        qw = (R[2][1] - R[1][2]) / s
        qx = 0.25 * s
        qy = (R[0][1] + R[1][0]) / s
        qz = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
        qw = (R[0][2] - R[2][0]) / s
        qx = (R[0][1] + R[1][0]) / s
        qy = 0.25 * s
        qz = (R[1][2] + R[2][1]) / s
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
        qw = (R[1][0] - R[0][1]) / s
        qx = (R[0][2] + R[2][0]) / s
        qy = (R[1][2] + R[2][1]) / s
        qz = 0.25 * s

    return (qx, qy, qz, qw)


def forward_position(
    theta_1: float,
    theta_2: float,
    theta_3: float,
    l2: float = 0.9,
    l3: float = 1.0,
) -> Tuple[float, float, float]:
    """
    End-effector position from URDF-consistent RTR kinematics.
    """
    reach = l2 + l3 * math.cos(theta_3)
    x = math.cos(theta_1) * reach
    y = math.sin(theta_1) * reach
    z = theta_2 + l3 * math.sin(theta_3)

    return (x, y, z)


def reference_ee_orientation(
    theta_1: float,
    theta_3: float,
) -> Tuple[float, float, float, float]:
    """
    End-effector orientation matching URDF chain:
        R = Rz(theta_1) * Ry(-theta_3)
    """

    cz = math.cos(theta_1)
    sz = math.sin(theta_1)

    cy = math.cos(-theta_3)
    sy = math.sin(-theta_3)

    Rz = [
        [cz, -sz, 0.0],
        [sz,  cz, 0.0],
        [0.0, 0.0, 1.0],
    ]

    Ry = [
        [cy,  0.0, sy],
        [0.0, 1.0, 0.0],
        [-sy, 0.0, cy],
    ]

    R = _matmul(Rz, Ry)

    return _rotation_matrix_to_quaternion(R)


def rtr_end_effector_transform(
    theta_1: float,
    theta_2: float,
    theta_3: float,
    l2: float = 0.9,
    l3: float = 1.0,
) -> Tuple[float, float, float, float, float, float, float]:
    """
    Full RTR end-effector pose.
    """
    tx, ty, tz = forward_position(theta_1, theta_2, theta_3, l2, l3)
    qx, qy, qz, qw = reference_ee_orientation(theta_1, theta_3)

    return (tx, ty, tz, qx, qy, qz, qw)


def tf_pose_matches_rtr_analytical(
    trans_x: float,
    trans_y: float,
    trans_z: float,
    rot_x: float,
    rot_y: float,
    rot_z: float,
    rot_w: float,
    theta_1: float,
    theta_2: float,
    theta_3: float,
    l2: float = 0.9,
    l3: float = 1.0,
    pos_tol: float = 1e-4,
    quat_dot_tol: float = 1e-4,
) -> bool:
    """
    Validate TF pose against analytical RTR FK.
    """
    ex, ey, ez = forward_position(theta_1, theta_2, theta_3, l2, l3)

    if (
        abs(trans_x - ex) > pos_tol
        or abs(trans_y - ey) > pos_tol
        or abs(trans_z - ez) > pos_tol
    ):
        return False

    eqx, eqy, eqz, eqw = reference_ee_orientation(theta_1, theta_3)

    dot = abs(
        rot_x * eqx +
        rot_y * eqy +
        rot_z * eqz +
        rot_w * eqw
    )

    return dot > (1.0 - quat_dot_tol)