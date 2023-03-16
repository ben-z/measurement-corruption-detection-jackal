#!/usr/bin/env python3

import rospy
from bcontrol.srv import RunSolver, RunSolverRequest, RunSolverResponse
import numpy as np
from multiprocessing import cpu_count, Pool
from itertools import chain, combinations
from solver_utils import optimize_l0, AmbiguousSolutionError, NoSolutionError
import time
from multiprocessing import cpu_count, Pool
from utils import make_srv_enum_lookup_dict

RES_ENUM_TO_STR = make_srv_enum_lookup_dict(RunSolverResponse)

def run_solver(req, worker_pool):
    perf_start = time.perf_counter()
    n = req.n
    q = req.q
    N = req.N
    Phi = np.array(req.Phi).reshape((q*N, n), order='F')
    Y = np.array(req.Y).reshape((q*N,), order='F')
    if len(req.eps) == 1:
        eps = np.array(req.eps) * np.ones((n, 1))
    elif len(req.eps) == n:
        eps = np.array(req.eps).reshape((n, 1), order='F')
    else:
        raise ValueError(f"Invalid eps {req.eps}")
    solver = req.solver
    max_num_corruptions = req.max_num_corruptions

    rospy.loginfo(f"Running solver {solver} with {n=} {q=} {N=} {Phi.shape=} {Y.shape=} {eps.shape=} {max_num_corruptions=}")

    if solver == RunSolverRequest.L0:
        x0_hat = []
        try:
            prob, x0_hat_cp = optimize_l0(n, q, N, Phi, Y, eps, worker_pool=worker_pool, max_num_corruptions=max_num_corruptions)
            status = RunSolverResponse.SUCCESS
            x0_hat = x0_hat_cp.value.tolist()
        except AmbiguousSolutionError as e:
            rospy.logwarn(f"Ambiguous solution: {e}")
            status = RunSolverResponse.AMBIGUOUS_SOLUTION
        except NoSolutionError as e:
            rospy.logwarn(f"No solution: {e}")
            status = RunSolverResponse.NO_SOLUTION
    elif solver == RunSolverRequest.L1:
        raise NotImplementedError
    else:
        raise ValueError(f"Unknown solver {solver}")
    
    perf_secs = time.perf_counter() - perf_start
    rospy.loginfo(f"Solver {solver} took {perf_secs:.3f} seconds")
    
    return RunSolverResponse(status=status, x0_hat=x0_hat)

def main():
    rospy.init_node('solver_server')

    # Use half the number of CPUs or 8, whichever is smaller
    # This is an arbitrary heuristic.
    numworkers = min(cpu_count() // 2, 8)
    rospy.loginfo(f"Starting the solver server with {numworkers=}...")
    with Pool(processes=numworkers) as pool:
        service = rospy.Service('run_solver', RunSolver, lambda req: run_solver(req, pool))
        rospy.loginfo("Solver server ready")
        rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Got ROSInterruptException. Exiting...")
