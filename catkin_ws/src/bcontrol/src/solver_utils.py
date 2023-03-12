import numpy as np
import cvxpy as cp

def get_evolution_matrices(As, Cs):
    """
    Inputs:
    As: Discrete-time A matrices
    Cs: Discrete-time C matrices

    Returns:
    state_evolution_matrix = [
        I,
        As[0],
        As[1]@As[0],
        ...
    ]
    output_evolution_matrix = [
        Cs[0],
        Cs[1]@As[0],
        Cs[2]@As[1]@As[0],
        ...
    ]
    In this way,
    state_evolution_matrix@x0 will return a vector that contains all states over time.
    output_evolution_matrix@x0 will return a vector that contains all measurements
    defined by the measurement matrices in Cs.

    Restrictions:
    1. len(As) == len(Cs) - 1
    2. elements of As and Cs are of the correct dimensions
       (e.g A has (n x n) elements, C[k] has (q_k x n) elements).
    """

    assert len(As) == len(Cs) - 1
    assert len(As) > 0

    n = As[0].shape[0]
    state_evolution_matrix_list = [np.eye(n)]
    for A_k in As:
        prev_matrix = state_evolution_matrix_list[-1]
        state_evolution_matrix_list.append(A_k@prev_matrix)

    output_evolution_matrix_list = [C_k @ m for m, C_k in zip(state_evolution_matrix_list, Cs)]

    return np.concatenate(state_evolution_matrix_list), np.concatenate(output_evolution_matrix_list)


def optimize_l1(n, q, N, Phi, Y):
    # solves the l1/l2 norm minimization problem
    # n: int - number of states
    # q: int - number of outputs
    # N: int - number of time steps
    # Phi: numpy.ndarray - matrix of size (q*N, n) - (C*A^0, C*A^1, ..., C*A^(N-1))'
    # Y: numpy.ndarray - measured outputs, with input effects subtracted, size (q*N)
    # returns: numpy.ndarray

    assert Phi.shape == (q*N, n)
    assert Y.shape == (q*N,)

    x0_hat = cp.Variable(n)
    # define the expression that we want to run l1/l2 optimization on
    optimizer = Y - np.matmul(Phi, x0_hat)
    # reshape to adapt to l1/l2 norm formulation
    # Note that cp uses fortran ordering (column-major), this is different from numpy,
    # which uses c ordering (row-major)
    optimizer_reshaped = cp.reshape(optimizer, (q, N))
    optimizer_final = cp.mixed_norm(optimizer_reshaped, p=2, q=1)
    # Equivalent to optimizer_final = cp.norm1(cp.norm(optimizer_reshaped, axis=1))

    obj = cp.Minimize(optimizer_final)

    # Form and solve problem.
    prob = cp.Problem(obj)
    prob.solve(verbose=False)  # Returns the optimal value.

    return (prob, x0_hat)
