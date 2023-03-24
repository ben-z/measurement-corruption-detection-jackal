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
USE_PARALLELISM = True

np.set_printoptions(precision=4, suppress=True, linewidth=400)

def run_solver(req, worker_pool):
    perf_start = time.perf_counter()
    n = req.n
    q = req.q
    N = req.N
    Phi = np.array(req.Phi).reshape((q*N, n), order='F')
    Y = np.array(req.Y).reshape((q*N,), order='F')
    if len(req.eps) == 1:
        eps = np.array(req.eps) * np.ones((q,))
    elif len(req.eps) == q:
        eps = np.array(req.eps).reshape((q,), order='F')
    else:
        raise ValueError(f"Invalid eps {req.eps}")
    solver = req.solver
    max_num_corruptions = req.max_num_corruptions
    sensor_protection = np.array(req.sensor_protection, dtype=np.bool)

    rospy.loginfo(f"Running solver {solver} with {n=} {q=} {N=} {Phi.shape=} {Y.shape=} {eps.shape=} {max_num_corruptions=}")

    x0_hat = []
    sensor_validity = []
    expanded_optimizer = []
    sensor_malfunction_max_magnitude = []
    if solver == RunSolverRequest.L0:
        try:
            (prob, x0_hat_cp), sensor_validity_np = optimize_l0(n, q, N, Phi, Y, eps,
                worker_pool=worker_pool, max_num_corruptions=max_num_corruptions, sensor_protection=sensor_protection)
            status = RunSolverResponse.SUCCESS
            x0_hat = x0_hat_cp.value.tolist()
            sensor_validity = sensor_validity_np.tolist()
            expanded_optimizer = (Y - Phi @ x0_hat_cp.value).reshape((q, N), order='F')
            sensor_malfunction_max_magnitude = np.abs(expanded_optimizer).max(axis=1).tolist()
        except AmbiguousSolutionError as e:
            rospy.logerr(f"Ambiguous solution: {e}")
            status = RunSolverResponse.AMBIGUOUS_SOLUTION
        except NoSolutionError as e:
            rospy.logerr(f"No solution: {e}")
            status = RunSolverResponse.NO_SOLUTION
    elif solver == RunSolverRequest.L1:
        try:
            prob, x0_hat_cp = optimize_l1(n, q, N, Phi, Y, eps, sensor_protection)
            if prob.status in ["optimal", "optimal_inaccurate"]:
                status = RunSolverResponse.SUCCESS
                x0_hat = x0_hat_cp.value.tolist()
                expanded_optimizer = (Y - Phi @ x0_hat_cp.value).reshape((q, N), order='F')
                sensor_malfunction_max_magnitude_np = np.abs(expanded_optimizer).max(axis=1)
                sensor_malfunction_max_magnitude = sensor_malfunction_max_magnitude_np.tolist()

                sensor_validity_np = (sensor_malfunction_max_magnitude_np <= eps).astype(np.bool)
                sensor_validity = sensor_validity_np.tolist()

                corrupted_indices = np.where(sensor_validity_np == False)
                if len(corrupted_indices[0]) > max_num_corruptions:
                    raise AmbiguousSolutionError(f"Expected at most {max_num_corruptions} corruptions, but got {len(corrupted_indices)} corruptions.", corrupted_indices)
            else:
                rospy.logwarn(f"Solve failed: {prob=}")
                status = RunSolverResponse.OTHER_FAILURE
        except AmbiguousSolutionError as e:
            rospy.logerr(f"Ambiguous solution: {e}")
            status = RunSolverResponse.AMBIGUOUS_SOLUTION
        except NoSolutionError as e:
            rospy.logerr(f"No solution: {e}")
            status = RunSolverResponse.NO_SOLUTION
    else:
        raise ValueError(f"Unknown solver {solver}")
    
    perf_secs = time.perf_counter() - perf_start
    rospy.loginfo(f"Solver {solver} took {perf_secs:.3f} seconds")
    rospy.loginfo(f"Solver {solver} returning {RES_ENUM_TO_STR[status]}")
    rospy.loginfo(f"x0_hat={np.array(x0_hat)}, {sensor_validity=}")
    rospy.loginfo(f"expanded_optimizer=\n{expanded_optimizer}")
    rospy.loginfo(f"sensor_validity={sensor_validity}")
    rospy.loginfo(f"sensor_malfunction_max_magnitude={np.array(sensor_malfunction_max_magnitude)}")
    
    rospy.logwarn("==============================================================================================================")
    
    return RunSolverResponse(status=status, x0_hat=x0_hat, sensor_validity=sensor_validity, malfunction_max_magnitude=sensor_malfunction_max_magnitude)

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
