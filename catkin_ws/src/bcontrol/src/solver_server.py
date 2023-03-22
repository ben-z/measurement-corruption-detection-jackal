#!/usr/bin/env python3

import rospy
from bcontrol.srv import RunSolver, RunSolverRequest, RunSolverResponse
import numpy as np
from multiprocessing import cpu_count, Pool
from itertools import chain, combinations
from solver_utils import optimize_l0, AmbiguousSolutionError, NoSolutionError, optimize_l1
import time
from multiprocessing import cpu_count, Pool
from utils import make_srv_enum_lookup_dict

RES_ENUM_TO_STR = make_srv_enum_lookup_dict(RunSolverResponse)
USE_PARALLELISM = False

np.set_printoptions(precision=4, suppress=True)

def run_solver(req, worker_pool):
    perf_start = time.perf_counter()
    n = req.n
    q = req.q
    N = req.N
    Phi = np.array(req.Phi).reshape((q*N, n), order='F')
    Y = np.array(req.Y).reshape((q*N,), order='F')
    if len(req.eps) == 1:
        eps = np.array(req.eps) * np.ones((q, 1))
    elif len(req.eps) == q:
        eps = np.array(req.eps).reshape((q, 1), order='F')
    else:
        raise ValueError(f"Invalid eps {req.eps}")
    solver = req.solver
    max_num_corruptions = req.max_num_corruptions

    rospy.loginfo(f"Running solver {solver} with {n=} {q=} {N=} {Phi.shape=} {Y.shape=} {eps.shape=} {max_num_corruptions=}")

    if solver == RunSolverRequest.L0:
        x0_hat = []
        sensor_validity = []
        try:
            (prob, x0_hat_cp), sensor_validity_np = optimize_l0(n, q, N, Phi, Y, eps, worker_pool=worker_pool, max_num_corruptions=max_num_corruptions)
            status = RunSolverResponse.SUCCESS
            x0_hat = x0_hat_cp.value.tolist()
            sensor_validity = sensor_validity_np.tolist()
        except AmbiguousSolutionError as e:
            rospy.logwarn(f"Ambiguous solution: {e}")
            status = RunSolverResponse.AMBIGUOUS_SOLUTION
        except NoSolutionError as e:
            rospy.logwarn(f"No solution: {e}")
            status = RunSolverResponse.NO_SOLUTION
    elif solver == RunSolverRequest.L1:
        x0_hat = []
        sensor_validity = []
        prob, x0_hat_cp = optimize_l1(n, q, N, Phi, Y)
        if prob.status in ["optimal", "optimal_inaccurate"]:
            status = RunSolverResponse.SUCCESS
            x0_hat = x0_hat_cp.value.tolist()
        else:
            rospy.logwarn(f"Solve failed: {prob=}")
            status = RunSolverResponse.OTHER_FAILURE
    else:
        raise ValueError(f"Unknown solver {solver}")
    
    perf_secs = time.perf_counter() - perf_start
    rospy.loginfo(f"Solver {solver} took {perf_secs:.3f} seconds")
    rospy.loginfo(f"Solver {solver} returning {RES_ENUM_TO_STR[status]} with x0_hat={np.array(x0_hat)}, {sensor_validity=}")
    
    return RunSolverResponse(status=status, x0_hat=x0_hat, sensor_validity=sensor_validity)

def main():
    rospy.init_node('solver_server')

    # Use half the number of CPUs or 8, whichever is smaller
    # This is an arbitrary heuristic.
    numworkers = min(cpu_count() // 2, 8)
    rospy.loginfo(f"Starting the solver server with {numworkers=}...")
    with Pool(processes=numworkers) as pool:
        if not USE_PARALLELISM:
            pool = None
        service = rospy.Service('run_solver', RunSolver, lambda req: run_solver(req, pool))
        rospy.loginfo("Solver server ready")
        rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Got ROSInterruptException. Exiting...")
