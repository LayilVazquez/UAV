"""
functions.py
Olfati-Saber flocking functions, ported from MATLAB_code/scr/UAV_Flocking.
All of them accept NumPy arrays and work element by element.
"""
import numpy as np


def sigma_norm(dist, epsilon):
    """sigma-norm of a vector from its Euclidean length dist: (sqrt(1 + eps dist²) - 1) / eps (NSigma.m).
    Unlike the Euclidean norm, it is differentiable at 0."""
    return (np.sqrt(1.0 + epsilon * np.asarray(dist) ** 2) - 1.0) / epsilon


def sigma_1(z):
    """z / sqrt(1 + z²) (FSigma.m)."""
    return z / np.sqrt(1.0 + z ** 2)


def bump(z, h):
    """Bump function rho_h (Rho.m): 1 on [0, h), smooth cosine down to 0 on [h, 1], 0 elsewhere."""
    z = np.asarray(z, dtype=float)
    return np.select([(z >= 0) & (z < h), (z >= h) & (z <= 1)],
                     [1.0, 0.5 * (1.0 + np.cos(np.pi * (z - h) / (1.0 - h)))], 0.0)


def phi(z, a, b, c):
    """Uneven sigmoid phi (Psi.m), 0 < a <= b: tends to a for large z (attraction) and to -b for
    very negative z (repulsion); c = |a - b| / sqrt(4ab) makes phi(0) = 0."""
    return 0.5 * ((a + b) * sigma_1(z + c) + (a - b))


def phi_alpha(z, ra, da, h, a, b, c):
    """Action function phi_alpha (Psi_a.m): phi centred on the desired distance da,
    switched off smoothly beyond the range ra. z, ra and da are sigma-norms."""
    return bump(z / ra, h) * phi(z - da, a, b, c)


def pairwise(P, epsilon):
    """Pair geometry of N particles P (N x 3).
    Returns D[i, j] = p_j - p_i (N x N x 3), the sigma-norm distances s (N x N)
    and the vectors n_ij = (p_j - p_i) / sqrt(1 + eps ||p_j - p_i||²) (nij.m)."""
    D = P[None, :, :] - P[:, None, :]
    dist = np.linalg.norm(D, axis=2)
    s = sigma_norm(dist, epsilon)
    n = D / np.sqrt(1.0 + epsilon * dist ** 2)[:, :, None]
    return D, s, n


def adjacency(s, ra, h):
    """Communication graph (aij.m): a_ij = rho_h(s_ij / ra), with no self loops.
    a_ij > 0 means UAV i and UAV j are neighbours and exchange velocities."""
    A = bump(s / ra, h)
    np.fill_diagonal(A, 0.0)
    return A
