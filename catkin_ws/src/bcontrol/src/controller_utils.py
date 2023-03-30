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
    - target_point: tuple (x, y) representing the target point on the path
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
    x_target, y_target = target_point

    # Compute the vector pointing from the robot to the target point
    vector_to_target = (x_target - x_robot, y_target - y_robot)

    # Calculate the distance to the target point
    distance_to_target = math.sqrt(
        vector_to_target[0]**2 + vector_to_target[1]**2)

    # Normalize the vector to get the desired direction of motion
    direction_to_target = (
        vector_to_target[0] / distance_to_target, vector_to_target[1] / distance_to_target)

    # Compute the desired heading based on the direction to the target point
    desired_heading = math.atan2(
        direction_to_target[1], direction_to_target[0])

    # Compute the heading error
    heading_error = math.atan2(math.sin(
        desired_heading - theta_robot), math.cos(desired_heading - theta_robot))

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
