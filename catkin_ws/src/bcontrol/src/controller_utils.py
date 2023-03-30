import math


def drive_to_target_point(robot_position, robot_linear_velocity, robot_angular_velocity, target_point, Kp_linear, max_lin_acc, Kp_angular, max_ang_acc):
    """
    Control policy to drive the robot towards the target point on the path
    using a straight line motion, while aligning the robot's heading with
    the direction of motion.

    Parameters:
    - robot_position: tuple (x, y, theta) representing the current position and orientation of the robot
    - robot_linear_velocity: float representing the current linear velocity of the robot
    - robot_angular_velocity: float representing the current angular velocity of the robot
    - target_point: tuple (x, y, theta) representing the target point on the path
    - Kp_linear: Proportional gain for the linear velocity control
    - max_lin_acc: Maximum allowable linear acceleration
    - Kp_angular: Proportional gain for the angular velocity control
    - max_ang_acc: Maximum allowable angular acceleration

    Returns:
    - linear_acceleration: The computed linear acceleration command
    - angular_acceleration: The computed angular acceleration command
    """
    # Extract the current position and orientation of the robot
    x_robot, y_robot, theta_robot = robot_position

    # Extract the coordinates of the target point
    x_target, y_target, theta_target = target_point

    # Compute the vector pointing from the robot to the target point
    vector_to_target = (x_target - x_robot, y_target - y_robot)

    # Calculate the distance to the target point
    distance_to_target = math.sqrt(vector_to_target[0]**2 + vector_to_target[1]**2)

    # Normalize the vector to get the desired direction of motion
    direction_to_target = (vector_to_target[0] / distance_to_target, vector_to_target[1] / distance_to_target)

    # Compute the desired heading based on the direction to the target point
    # As the robot approaches the target point, the desired heading should
    # approach the target point's heading
    desired_heading = min(1, distance_to_target) * math.atan2(direction_to_target[1], direction_to_target[0]) \
        + (1 - min(1, distance_to_target)) * theta_target

    # Compute the heading error
    heading_error = math.atan2(math.sin(desired_heading - theta_robot), math.cos(desired_heading - theta_robot))

    # Compute the desired linear velocity using proportional control
    desired_linear_velocity = Kp_linear * distance_to_target

    # Compute the forward acceleration based on the difference between the desired and current linear velocities
    linear_acceleration = desired_linear_velocity - robot_linear_velocity

    # Limit the linear acceleration to the maximum allowable value
    linear_acceleration = max(
        min(linear_acceleration, max_lin_acc), -max_lin_acc)

    # Compute the desired angular velocity using proportional control based on heading error
    desired_angular_velocity = Kp_angular * heading_error

    # Compute the angular acceleration based on the difference between the desired and current angular velocities
    angular_acceleration = desired_angular_velocity - robot_angular_velocity

    # Limit the angular acceleration to the maximum allowable value
    angular_acceleration = max(min(angular_acceleration, max_ang_acc), -max_ang_acc)

    return linear_acceleration, angular_acceleration


def stanley_controller(cross_track_error, heading_error, linear_velocity, angular_velocity, desired_speed, curvature, d_curvature_ds, Kp_e, Kp_heading, Kp_lin, Kp_ang, max_lin_acc, max_ang_acc):
    """
    Modified Stanley controller for path following that outputs linear and angular accelerations.

    Parameters:
    - cross_track_error: float representing the lateral deviation from the path
    - heading_error: float representing the difference between the robot's heading and the path's tangent angle (in radians)
    - linear_velocity: float representing the current linear velocity of the robot
    - angular_velocity: float representing the current angular velocity of the robot
    - desired_speed: float representing the desired linear speed of the robot
    - curvature: float representing the curvature of the path at the nearest point
    - d_curvature_ds: float representing the derivative of curvature with respect to path length
    - Kp_e: float representing the gain for cross-track error correction
    - Kp_heading: float representing the gain for heading error correction
    - Kp_lin: Proportional gain for linear acceleration control
    - Kp_ang: Proportional gain for angular acceleration control
    - max_lin_acc: Maximum allowable linear acceleration
    - max_ang_acc: Maximum allowable angular acceleration

    Returns:
    - linear_acceleration: The computed linear acceleration command
    - angular_acceleration: The computed angular acceleration command
    """
    # Compute the desired linear velocity based on the desired speed
    desired_linear_velocity = desired_speed

    # Compute the desired angular velocity based on cross-track error, heading error, and curvature
    desired_angular_velocity = Kp_e * cross_track_error + \
        Kp_heading * heading_error + linear_velocity * curvature

    # Compute the linear acceleration based on the difference between desired and current linear velocities
    linear_acceleration = Kp_lin * (desired_linear_velocity - linear_velocity)
    # Limit linear acceleration
    linear_acceleration = max(
        min(linear_acceleration, max_lin_acc), -max_lin_acc)

    # Compute the angular acceleration based on the difference between desired and current angular velocities
    angular_acceleration = Kp_ang * \
        (desired_angular_velocity - angular_velocity) + \
        linear_velocity * d_curvature_ds
    # Limit angular acceleration
    angular_acceleration = max(
        min(angular_acceleration, max_ang_acc), -max_ang_acc)

    return linear_acceleration, angular_acceleration
