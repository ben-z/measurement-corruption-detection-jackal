import numpy as np
import cvxpy as cp
from scipy.linalg import block_diag
from utils import powerset
import time
import multiprocessing as mp
from multiprocessing import cpu_count, Pool
from multiprocessing.pool import ThreadPool
import rospy
from typing import List

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


def optimize_l1(n, q, N, Phi, Y, eps: np.ndarray = 0.2, sensor_protection: np.ndarray = None, x0_regularization_lambda: float = 0.0):
    # solves the l1/l2 norm minimization problem
    # n: int - number of states
    # q: int - number of outputs
    # N: int - number of time steps
    # Phi: numpy.ndarray - matrix of size (q*N, n) - (C*A^0, C*A^1, ..., C*A^(N-1))'
    # Y: numpy.ndarray - measured outputs, with input effects subtracted, size (q*N)
    # returns: numpy.ndarray

    assert Phi.shape == (q*N, n)
    assert Y.shape == (q*N,)

    if sensor_protection is None:
        sensor_protection = np.zeros(q, dtype=np.bool)

    x0_hat = cp.Variable(n)
    # define the expression that we want to run l1/l2 optimization on
    optimizer = Y - np.matmul(Phi, x0_hat)
    # reshape to adapt to l1/l2 norm formulation
    # Note that cp uses fortran ordering (column-major), this is different from numpy,
    # which uses c ordering (row-major)
    optimizer_reshaped = cp.reshape(optimizer, (q, N))
    optimizer_final = cp.mixed_norm(optimizer_reshaped, p=2, q=1) + x0_regularization_lambda * cp.norm(x0_hat)
    # Equivalent to optimizer_final = cp.norm1(cp.norm(optimizer_reshaped, axis=1))

    # add sensor protection constraints
    constraints = []
    for i in range(q):
        if sensor_protection[i]:
            # optimizer_final += cp.norm(optimizer_reshaped[i, :]) / eps[i]
            # The extra 1e-6 is to avoid numerical issues
            constraints.append(optimizer_reshaped[i, :] <= eps[i] - 1e-6)
            constraints.append(optimizer_reshaped[i, :] >= -eps[i] + 1e-6)

    obj = cp.Minimize(optimizer_final)

    # Form and solve problem.
    prob = cp.Problem(obj, constraints)
    prob.solve(verbose=False)  # Returns the optimal value.

    if 'optimal' not in prob.status:
        raise NoSolutionError("No solution found")

    return (prob, x0_hat)

def get_input_effect_on_state(As, Bs, us):
    """
    Returns the effect of inputs on the state evolution.
    """
    N = len(As)
    assert len(Bs) == N, "len(Bs) != len(As)"
    assert len(us) == N, "len(us) != len(As)"

    n = As[0].shape[0]
    
    input_effect_matrix_As = block_diag(*[np.eye(n) for _ in range(N)])
    for k in range(1, N):
        for j in range(k):
            input_effect_matrix_As[k*n:(k+1)*n, j*n:(j+1)*n] = \
                As[k] @ input_effect_matrix_As[(k-1)*n:k*n, j*n:(j+1)*n]

    input_effect_matrix_Bs = block_diag(*Bs)

    input_effect = (input_effect_matrix_As @ input_effect_matrix_Bs @ np.vstack(us)).reshape((n, N), order='F')

    return input_effect


def get_l1_objective_fn(n, q, N, Phi, Y, x0_hat):
    """
    Returns the objective function optimizer of the l1/l2 norm minimization problem
    n: int - number of states
    q: int - number of outputs
    N: int - number of time steps
    Phi: numpy.ndarray - matrix of size (q*N, n) - (C*A^0, C*A^1, ..., C*A^(N-1))'
    Y: numpy.ndarray - measured outputs, with input effects subtracted, size (q*N)
    x0_hat: numpy.ndarray - estimated initial state, size (n)
    Returns: cp.Variable
    """
    
    assert Phi.shape == (q*N, n)
    assert Y.shape == (q*N,)
    assert x0_hat.shape == (n,)

    # define the expression that we want to run l1/l2 optimization on
    optimizer = Y - np.matmul(Phi, x0_hat)
    # reshape to adapt to l1/l2 norm formulation
    # Note that cp uses fortran ordering (column-major), this is different from numpy,
    # which uses c ordering (row-major)
    optimizer_reshaped = cp.reshape(optimizer, (q, N))
    optimizer_final = cp.mixed_norm(optimizer_reshaped, p=2, q=1)
    # Equivalent to optimizer_final = cp.norm1(cp.norm(optimizer_reshaped, axis=1))

    return optimizer_final

class NoSolutionError(Exception):
    pass

class AmbiguousSolutionError(Exception):
    def __init__(self, message, solutions):
        super().__init__(message)
        self.solutions = solutions
    
def to_mask(l: List[int], q: int) -> np.ndarray:
    # l: list of indices to be masked
    # q: size of the mask
    # returns: numpy.ndarray

    mask = np.zeros(q, dtype=np.bool)
    mask[l] = 1
    return mask

def optimize_l0(n: int, q: int, N: int, Phi: np.ndarray, Y: np.ndarray, eps: np.ndarray = 0.2,
    worker_pool: Pool = None,
    max_num_corruptions: int = -1,
    sensor_protection: np.ndarray = None,
    x0_regularization_lambda: float = 0.0,
):
    # solves the l0 minimization problem
    # n: int - number of states
    # q: int - number of outputs
    # N: int - number of time steps
    # Phi: numpy.ndarray - matrix of size (q*N, n) - (C*A^0, C*A^1, ..., C*A^(N-1))'
    # Y: numpy.ndarray - measured outputs, with input effects subtracted, size (q*N)
    # returns: numpy.ndarray

    assert Phi.shape == (q*N, n)
    assert Y.shape == (q*N,)

    if max_num_corruptions < 0:
        max_num_corruptions = q
    
    if sensor_protection is None:
        sensor_protection = np.zeros(q, dtype=np.bool)

    # Candidate sets of corrupted sensors, in increasing order of size of the set
    sensor_candidates = [list(s) for s in powerset(range(q)) if len(s) <= max_num_corruptions and not np.logical_and(to_mask(list(s), q), sensor_protection).any()]

    if worker_pool:
        # Parallel version with multiprocessing.Pool
        solns = worker_pool.starmap(
            optimize_l0_subproblem,
            # TODO: change the 1 to the maximum number of sensors that can be corrupted
            [(n, q, N, Phi, Y, corrupted_indices, eps, x0_regularization_lambda)
                for corrupted_indices in sensor_candidates]
        )
    else:
        # Sequential version
        solns = [optimize_l0_subproblem(n, q, N, Phi, Y, corrupted_indices, eps, x0_regularization_lambda)
                 for corrupted_indices in sensor_candidates]
    
    # Go through the solutions and return the first one that is feasible.
    # If there are two feasible solutions with the same size, raise AmbiguousSolutionError
    feasible_solns = [(soln, ss) for soln, ss in zip(solns, sensor_candidates) if soln[0].status in ["optimal", "optimal_inaccurate"]]
    if len(feasible_solns) == 0:
        raise NoSolutionError("No solution found")

    smallest_feasible_solns = [(soln, ss) for (soln, ss) in feasible_solns if len(ss) == len(feasible_solns[0][1])]

    if len(smallest_feasible_solns) > max_num_corruptions:
        raise AmbiguousSolutionError(f"Expected at most {max_num_corruptions} corruption(s), but got {len(smallest_feasible_solns)} corruption(s).", smallest_feasible_solns)

    # sensor validity
    sensor_validity = np.ones(q, dtype=bool)
    sensor_validity[list(smallest_feasible_solns[0][1])] = False

    return smallest_feasible_solns[0][0], sensor_validity


def optimize_l0_subproblem(n, q, N, Phi, Y, attacked_sensor_indices, eps, x0_regularization_lambda: float = 0.0):
    x0_hat = cp.Variable(n)
    num_valid_sensors = q - len(attacked_sensor_indices)

    optimizer_full = Y - np.matmul(Phi, x0_hat)
    valid_sensors_I = np.delete(np.eye(q), attacked_sensor_indices, axis=0)
    valid_sensors_matrix = block_diag(*[valid_sensors_I]*N)
    # make it so that we don't minimize the attacked sensors' magnitudes
    optimizer = valid_sensors_matrix @ optimizer_full
    optimizer_reshaped = cp.reshape(optimizer, (num_valid_sensors, N))
    optimizer_final = cp.mixed_norm(optimizer_reshaped, p=2, q=1) + x0_regularization_lambda * cp.norm(x0_hat)

    # Support both scalar eps and eps per sensor
    if np.isscalar(eps):
        eps = np.ones(q) * eps

    constraints = []
    for j in set(range(q)) - set(attacked_sensor_indices):
        for t in range(N):
            # constraints.append(cp.norm(optimizer[q*t+j]) <= eps)
            constraints.append(optimizer_full[q*t+j] <= eps[j] - 1e-6)
            constraints.append(optimizer_full[q*t+j] >= -eps[j] + 1e-6)

    prob = cp.Problem(cp.Minimize(cp.norm(optimizer_final)), constraints)
    start = time.time()
    prob.solve()
    end = time.time()

    print(
        f"Solved l0 subproblem in {end-start:.2f} seconds. Indices: {attacked_sensor_indices}, status: {prob.status}, value: {prob.value}")

    if 'optimal' in prob.status and attacked_sensor_indices == [2]:
        rospy.logwarn("Special output")
        expanded_optimizer = (Y - Phi @ x0_hat.value).reshape((q, N), order='F')
        rospy.logwarn(f"x0_hat={x0_hat.value}")
        rospy.logwarn(f"expanded_optimizer=\n{expanded_optimizer}")

    return (prob, x0_hat)