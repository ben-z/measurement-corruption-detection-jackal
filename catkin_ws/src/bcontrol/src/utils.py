import functools
import math
import numpy as np
from numpy.testing import assert_allclose
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from dataclasses import dataclass
from geometry_msgs.msg import Twist
from scipy.interpolate import interp1d
from typing import List, Tuple, Union, Any, Optional
import typeguard as _typeguard
from typeguard import checker_lookup_functions, TypeCheckerCallable, TypeCheckMemo, TypeCheckError
from enum import Enum
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import rospy
from itertools import chain, combinations
from numpy import sin, cos
from math import atan2, sqrt
from bcontrol.msg import Path as PathMsg

@dataclass
class PathPoint:
    """
    Object that represents a point on a path.
    """
    segment_progress_m: "float | None" = None
    segment_idx: "int | None" = None
    point: "list[float] | None" = None
    

class Path:
    """
    The Path object represents a path that can be followed by a robot.
    """
    
    def __init__(self,
        points: List[Tuple[float,float]],
        headings: Optional[List[float]] = None,
        curvatures: Optional[List[float]] = None,
        dK_ds_list: Optional[List[float]] = None,
        velocities: Optional[List[float]] = None,
        closed=False
    ):
        """
        Arguments:
            path: A list of points, each point is a tuple (x, y). The path is linearly interpolated.
            closed: Whether the path is closed. If the path is closed, the robot will return to the start point when it reaches the end point.
        """
        self.points = points
        self.length = 0
        self.lengths = []
        self.headings = headings or []
        self.curvatures = curvatures or ([0] * len(points))
        self.dK_ds_list = dK_ds_list or ([0] * len(points))
        self.velocities = velocities or []
        self.closed = closed
        for i in range(len(self.points) - 1):
            # Calculate the length of each path segment
            self.lengths.append(np.linalg.norm(np.array(self.points[i + 1]) - np.array(self.points[i])))
            # Calculate the total length of the path
            self.length += self.lengths[-1]

        if self.headings == []:
            # Heading is not provided. Calculate the heading of each path segment
            for i in range(len(self.points) - 1):
                self.headings.append(np.arctan2(self.points[i + 1][1] - self.points[i][1], self.points[i + 1][0] - self.points[i][0]))
            if closed:
                # Add the heading of the last point to the first point
                self.headings.append(np.arctan2(self.points[0][1] - self.points[-1][1], self.points[0][0] - self.points[-1][0]))
            else:
                # Repeat the last heading if the path is not closed
                self.headings.append(self.headings[-1])
        
        assert self.headings, f"Headings is {self.headings}"
        
        if closed:
            # Add the length of the last point to the first point
            self.lengths.append(np.linalg.norm(np.array(self.points[0]) - np.array(self.points[-1])))
            self.length += self.lengths[-1]
    
    @classmethod
    def from_pose_array(cls, pose_array, closed: Union[bool,None] = None):
        """
        Returns a new Path object created from the given PoseArray.
        """
        path = []
        for pose in pose_array.poses:
            path.append([pose.position.x, pose.position.y])
        if closed is None:
            return cls(path)
        return cls(path, closed=closed)
    
    @classmethod
    def from_path_msg(cls, path_msg: PathMsg, closed: Optional[bool] = None):
        """
        Returns a new Path object created from the given PathMsg.
        """
        points = [(pose.position.x, pose.position.y) for pose in path_msg.poses]
        headings = [euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2] for pose in path_msg.poses]
        velocities = [tw.linear.x for tw in path_msg.twists]
        args = [points, headings, path_msg.curvatures, path_msg.dK_ds_list, velocities]

        if closed is None:
            return cls(*args)
        return cls(*args, closed=closed)
        
    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Path):
            return NotImplemented
        
        return self.points == other.points and self.closed == other.closed
        
    def get_point(self, progress_m=None, progress_pct=None):
        """
        Returns the point on the path at the given progress. The runtime of this function is O(n), where n is the number of path segments.
        
        Arguments:
            progress_m: The progress along the path in meters. If the path is closed, this value can be greater than the path length.
            progress_pct: The progress along the path as a percentage. If the path is closed, this value can be greater than 1.
        Returns:
            The point on the path at the given progress, as a PathPoint object.
        """
        if progress_m is not None and progress_pct is not None:
            raise ValueError('Only one of progress_m or progress_pct can be specified.')
        if progress_m is None:
            if progress_pct is None:
                raise ValueError('Either progress_m or progress_pct must be specified.')
            else:
                progress_m = progress_pct * self.length

        if self.closed:
            progress_m = progress_m % self.length
        if progress_m < 0:
            raise ValueError(f'{progress_m=} must be greater than or equal to 0.')
        if progress_m > self.length:
            raise ValueError(f'{progress_m=} must be less than or equal to the path length.')
        for i, segment_length in enumerate(self.lengths):
            if progress_m <= segment_length:
                return PathPoint(
                    segment_progress_m=progress_m,
                    segment_idx=i,
                    point=get_point_on_segment(progress_m, self.points[i], self.points[(i + 1) % len(self.points)], segment_length)
                )
            progress_m -= segment_length
        
        raise ValueError(f"Couldn't fine the point in path! This is impossible. {progress_m=}")
    
    def get_lateral_position(self, point: np.ndarray, closest_point: Optional[PathPoint] = None):
        """
        Returns the lateral error of the given point from the path at the given path point.
        When the point is to the left of the path, the lateral position is positive.
        When the point is to the right of the path, the lateral position is negative.
        """
        if closest_point is None:
            closest_point = self.get_closest_point(point)
        
        assert closest_point.segment_idx is not None, f"{closest_point=} must have a valid segment_idx attribute."
        
        if not self.closed:
            assert closest_point.segment_idx < len(self.points) - 1, f"segment_idx must be less than the number of path points - 1 for open paths. {closest_point.segment_idx=} {len(self.points)=}"
        
        x1, y1 = self.points[closest_point.segment_idx]
        x2, y2 = self.points[(closest_point.segment_idx + 1) % len(self.points)]
        x0, y0 = point

        return calculate_signed_distance(x1, y1, x2, y2, x0, y0)

    def get_heading_at_point(self, path_point):
        """
        Returns the heading of the path at the given point.
        """
        if path_point.segment_idx is None:
            raise ValueError('PathPoint object must have a segment_idx attribute.')

        return self.headings[path_point.segment_idx]

    def get_velocity_at_point(self, path_point):
        """
        Returns the velocity of the path at the given point.
        """
        if path_point.segment_idx is None:
            raise ValueError('PathPoint object must have a segment_idx attribute.')

        if not self.velocities:
            raise ValueError(f'Path does not have velocities. {self.velocities=}')

        return self.velocities[path_point.segment_idx]
    
    def get_curvature_at_point(self, path_point):
        """
        Returns the curvature of the path at the given point.
        """
        if path_point.segment_idx is None:
            raise ValueError('PathPoint object must have a segment_idx attribute.')

        return self.curvatures[path_point.segment_idx]
    
    def get_dK_ds_at_point(self, path_point):
        """
        Returns the dK_ds of the path at the given point.
        """
        if path_point.segment_idx is None:
            raise ValueError('PathPoint object must have a segment_idx attribute.')

        return self.dK_ds_list[path_point.segment_idx]

    def get_closest_point(self, point):
        """
        Returns the closest point on the path to the given point.
        
        Arguments:
            point: The point to which the closest point on the path should be found.

        Returns a PathPoint object.

        Returns a dictionary with the following keys:
            idx: The index of the closest path segment on the path
            point: The closest point on the path
            dist: The distance between the closest point on the path and the given point
            progress_m: The distance along the path segment to the closest point on the path
        """
        closest = None
        closest_dist = None

        if self.closed:
            segments = zip(self.points, self.points[1:] + [self.points[0]])
        else:
            segments = zip(self.points[:-1], self.points[1:])

        for i, (p0, p1) in enumerate(segments):
            p = get_closest_point_on_segment(point, p0, p1)
            dist = np.linalg.norm(np.array(p.point) - np.array(point))
            if closest is None or dist < closest_dist:
                closest = p
                closest.segment_idx = i
                closest_dist = dist

        assert closest is not None, 'No closest point found.'

        return closest
    
    def get_local_closest_point(self, point: np.ndarray, path_point: PathPoint):
        """
        Returns the closest point on the path to the given point. This function is the same as get_closest_point, but it does a local search around the given path_point to improve performance.
        """

        if path_point.segment_idx is None:
            raise ValueError('PathPoint object must have a segment_idx attribute.')

        if self.closed:
            segments = list(zip(self.points, self.points[1:] + [self.points[0]]))
        else:
            segments = list(zip(self.points[:-1], self.points[1:]))

        assert path_point.segment_idx < len(segments), f"PathPoint segment_idx {path_point.segment_idx} is out of bounds for path with {len(segments)} segments."

        current_segment = segments[path_point.segment_idx]
        closest_point_current_segment = get_closest_point_on_segment(point, *current_segment)
        closest_point_current_segment.segment_idx = path_point.segment_idx
        dist_to_current_segment = np.linalg.norm(np.array(closest_point_current_segment.point) - np.array(point))
        
        if self.closed or path_point.segment_idx < len(segments) - 1:
            next_segment_idx = (path_point.segment_idx + 1) % len(segments)
            closest_point_next_segment = get_closest_point_on_segment(point, *segments[next_segment_idx])
            closest_point_next_segment.segment_idx = next_segment_idx
            dist_to_next_segment = np.linalg.norm(np.array(closest_point_next_segment.point) - np.array(point))
            if dist_to_next_segment < dist_to_current_segment:
                return self.get_local_closest_point(point, closest_point_next_segment)
        if self.closed or path_point.segment_idx > 0:
            prev_segment_idx = (path_point.segment_idx - 1) % len(segments)
            closest_point_previous_segment = get_closest_point_on_segment(point, *segments[prev_segment_idx])
            closest_point_previous_segment.segment_idx = prev_segment_idx
            dist_to_previous_segment = np.linalg.norm(np.array(closest_point_previous_segment.point) - np.array(point))
            if dist_to_previous_segment < dist_to_current_segment:
                return self.get_local_closest_point(point, closest_point_previous_segment)

        return closest_point_current_segment
    
    def to_pose_array(self, query_slice=None):
        """
        Returns a PoseArray representation of the path.
        """
        if query_slice is None:
            query_slice = slice(0, len(self.points))

        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        for i, p in enumerate(self.points[query_slice]):
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            heading = self.headings[i]
            pose.orientation = Quaternion(*quaternion_from_euler(0, 0, heading))
            pose_array.poses.append(pose)
        return pose_array
    
    def to_path_msg(self, query_slice=None):
        """
        Returns a Path message representation of the path.
        """
        if query_slice is None:
            query_slice = slice(0, len(self.points))
        
        path_msg = PathMsg()
        path_msg.header.frame_id = 'map'
        for i, p in enumerate(self.points[query_slice]):
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            heading = self.headings[i]
            pose.orientation = Quaternion(*quaternion_from_euler(0, 0, heading))
            path_msg.poses.append(pose)

            if self.velocities:
                twist = Twist()
                twist.linear.x = self.velocities[i]
                path_msg.twists.append(twist)
            
            path_msg.curvatures.append(self.curvatures[i])
            path_msg.dK_ds_list.append(self.dK_ds_list[i])

        return path_msg
    
    def walk(self, starting_point: PathPoint, length_m: float) -> PathPoint:
        """
        Walks along the path starting at the given point.
        Arguments:
            starting_point (PathPoint): The point at which to start walking along the path.
            length_m (float): The distance to walk along the path.
        """
        assert starting_point.segment_idx is not None, "The starting point must have a segment_idx attribute."
        assert starting_point.segment_progress_m is not None, "The starting point must have a segment_progress_m attribute."

        current_point = starting_point

        # Walk along the path until we have walked the given distance
        if self.closed:
            segments = list(zip(self.points, self.points[1:] + [self.points[0]]))
        else:
            segments = list(zip(self.points[:-1], self.points[1:]))

        # The maximum number of iterations to prevent infinite loops
        max_iterations = len(segments) * math.ceil(abs(length_m) / self.length)
        completed_iterations = 0

        while True:
            current_segment_idx = current_point.segment_idx
            assert current_segment_idx is not None, "The current point must have a segment_idx attribute."
            assert current_point.segment_progress_m is not None, "The current point must have a segment_progress_m attribute."
            if current_point.segment_progress_m + length_m > self.lengths[current_segment_idx]:
                # Walk forward to the next segment
                if self.closed:
                    next_segment_idx = (current_segment_idx + 1) % len(segments)
                else:
                    next_segment_idx = current_segment_idx + 1

                length_m -= self.lengths[current_segment_idx] - current_point.segment_progress_m
                current_point = PathPoint(segment_idx=next_segment_idx, segment_progress_m=0)
            elif current_point.segment_progress_m + length_m < 0:
                # Walk backward to the previous segment
                if self.closed:
                    next_segment_idx = (current_segment_idx - 1) % len(segments)
                else:
                    next_segment_idx = current_segment_idx - 1

                length_m += current_point.segment_progress_m
                current_point = PathPoint(segment_idx=next_segment_idx, segment_progress_m=self.lengths[next_segment_idx])
            elif current_point.segment_progress_m + length_m <= self.lengths[current_segment_idx]:
                # Walk within the current segment
                current_point = PathPoint(
                    segment_idx=current_segment_idx,
                    segment_progress_m=current_point.segment_progress_m + length_m,
                    point=get_point_on_segment(current_point.segment_progress_m + length_m, *segments[current_segment_idx], self.lengths[current_segment_idx])
                )
                return current_point

            completed_iterations += 1
            if completed_iterations > max_iterations:
                raise Exception(f"Walked {completed_iterations} iterations without completion. This is likely an infinite loop.")

def generate_circle_approximation(center, radius, num_points):
    """
    Generates a list of points on a circle.
    Arguments:
        center: The center of the circle.
        radius: The radius of the circle.
        num_points: The number of points to generate. The more points, the more accurate the approximation.
    Returns:
        points: A list of points on the figure eight shape.
        headings: A list of headings (θ) at each point.
        curvatures: A list of curvatures (κ) at each point.
        dK_ds_list: A list of dκ_ds at each point. Where κ is the curvature and s is the arc length.
    
    Derivation: https://github.com/ben-z/research-sensor-attack/blob/e20c7b02cf6aca6c18c37976550c03606919192a/curves.py#L173-L191
    """
    a = radius

    points = []
    headings = []
    curvatures = []
    dK_ds_list = []
    for i in range(num_points):
        t = 2 * math.pi * i / num_points
        points.append([
            center[0] + a * math.cos(t),
            center[1] + a * math.sin(t)
        ])
        headings.append(atan2(a*cos(t), -a*sin(t)))
        # This could be simplified to 1/a, but we leave it as is for consistency with the derivation.
        curvatures.append(1/sqrt(a**2*sin(t)**2 + a**2*cos(t)**2))
        dK_ds_list.append(0) # circles have constant curvature
    return points, headings, curvatures, dK_ds_list

def generate_figure_eight_approximation(center, length, width, num_points):
    """
    Generates a list of points on a figure eight shape.
    Arguments:
        center: The center of the figure eight shape.
        length: The length of the figure eight shape.
        width: The width of the figure eight shape.
        num_points: The number of points to generate. The more points, the more accurate the approximation.
    Returns:
        points: A list of points on the figure eight shape.
        headings: A list of headings (θ) at each point.
        curvatures: A list of curvatures (κ) at each point.
        dK_ds_list: A list of dκ_ds at each point. Where κ is the curvature and s is the arc length.

    The formula used to generate the points is:
        x = a * sin(t)
        y = b * sin(2t)/2
    where a = length / 2 and b = width.

    Supplementary visualization:
    https://www.desmos.com/calculator/fciqxay3p2
    Derivation:
    https://github.com/ben-z/research-sensor-attack/blob/e20c7b02cf6aca6c18c37976550c03606919192a/curves.py#L153-L171
    """
    a = length / 2
    b = width

    points = []
    headings = []
    curvatures = []
    dK_ds_list = []
    for i in range(num_points):
        # t is an arbitrary parameter that is used to generate the points
        # The result is known as an arbitrary-speed curve
        t = 2 * math.pi * i / num_points
        x = center[0] + a * math.sin(t)
        y = center[1] + b * (math.sin(t * 2) / 2)
        points.append([x, y])
        headings.append(atan2(b * cos(t * 2), a * cos(t)))
        curvatures.append((a*b*sin(t)*cos(2*t) - 2*a*b*sin(2*t)*cos(t))/(a**2*cos(t)**2 + b**2*cos(2*t)**2)**(3/2))
        dK_ds_list.append((-3*a*b*cos(t)*cos(2*t)/(a**2*cos(t)**2 + b**2*cos(2*t)**2)**(3/2) + (3*a**2*sin(t)*cos(t) + 6*b**2*sin(2*t)*cos(2*t))*(a*b*sin(t)*cos(2*t) - 2*a*b*sin(2*t)*cos(t))/(a**2*cos(t)**2 + b**2*cos(2*t)**2)**(5/2))/sqrt(a**2*cos(t)**2 + b**2*cos(2*t)**2))
    return points, headings, curvatures, dK_ds_list

# Generated by ChatGPT
def generate_ellipse_approximation(center, a, b, num_points, theta=0):
    """
    Generates a list of points on an ellipse.
    Arguments:
        center: The center of the ellipse.
        a: The length of the major axis of the ellipse.
        b: The length of the minor axis of the ellipse.
        num_points: The number of points to generate. The more points, the more accurate the approximation.
        theta: The orientation of the ellipse in radians.
    Returns a list of points on the ellipse.
    """
    points = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = center[0] + a * math.cos(angle) * math.cos(theta) - b * math.sin(angle) * math.sin(theta)
        y = center[1] + a * math.cos(angle) * math.sin(theta) + b * math.sin(angle) * math.cos(theta)
        points.append([x, y])
    return points

# Generated by ChatGPT
def rotate_points(points, angle, center_of_rotation=[0, 0]):
    """
    Rotates a list of points by a specified angle around a specified center of rotation.
    Arguments:
        points: A list of points to rotate.
        angle: The angle to rotate the points by, in radians.
        center_of_rotation: The center of rotation. Default is the origin [0, 0].
    Returns a new list of points that have been rotated by the specified angle around the specified center of rotation.
    """
    rotated_points = []
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)
    for point in points:
        x = center_of_rotation[0] + (point[0] - center_of_rotation[0]) * cos_angle - (point[1] - center_of_rotation[1]) * sin_angle
        y = center_of_rotation[1] + (point[0] - center_of_rotation[0]) * sin_angle + (point[1] - center_of_rotation[1]) * cos_angle
        rotated_points.append([x, y])
    return rotated_points

def get_point_on_segment(progress_m:float, p1, p2, v_length_m=None):
    """
    Returns the point on the line segment at the given progress.
    
    Arguments:
        progress_m: The progress along the segment.
        p1: The first point of the segment.
        p2: The second point of the segment.
        v_length_m: The length of the segment. If not specified, it will be calculated.
    
    Returns a PathPoint object.
    """
    p1 = np.array(p1)
    p2 = np.array(p2)
    v = p2 - p1
    if v_length_m is None:
        v_length_m = np.linalg.norm(v)
    v_normalized = v / v_length_m
    return p1 + v_normalized * progress_m

    
def get_closest_point_on_segment(point, p1, p2):
    """
    Returns the closest point on the line segment to the given point.

    Arguments:
        point: The point to which the closest point on the segment should be found.
        p1: The first point of the segment.
        p2: The second point of the segment.
    
    Returns a PathPoint object.
    """
    p = np.array(point)
    p1 = np.array(p1)
    p2 = np.array(p2)
    v = p2 - p1
    v_length_m = np.linalg.norm(v)
    v_normalized = v / v_length_m
    p1_to_p = p - p1
    progress_m = np.dot(p1_to_p, v_normalized)
    if progress_m < 0:
        progress_m = 0
    elif progress_m > v_length_m:
        progress_m = v_length_m
    closest = p1 + v_normalized * progress_m
    
    return PathPoint(point=closest, segment_progress_m=progress_m)

def pathpoints_to_pose_array(pathpoints, path, frame_id="map"):
    """
    Converts a list of PathPoint objects to a PoseArray message.
    """
    pose_array = PoseArray()
    pose_array.header.frame_id = frame_id
    for pathpoint in pathpoints:
        pose = Pose()
        pose.position.x = pathpoint.point[0]
        pose.position.y = pathpoint.point[1]
        pose.orientation = Quaternion(*quaternion_from_euler(0, 0, path.headings[pathpoint.segment_idx]))
        pose_array.poses.append(pose)
    return pose_array

def wrap_angle(angle):
    """
    Wraps an angle to the range [-pi, pi]
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi

def clamp(n, minn, maxn):
    """
    Clamps a number between a minimum and maximum value.
    """
    return max(min(maxn, n), minn)


# Generated by ChatGPT
def convert_steering_angle_to_twist(steering_angle, wheelbase, linear_velocity):
    """
    Convert a kinematic bicycle model's steering wheel angle command to a Twist message.

    :param steering_angle: the steering wheel angle in radians
    :param wheelbase: the distance between the front and rear axles of the vehicle in meters
    :param linear_velocity: the linear velocity of the vehicle in meters per second
    :return: a Twist message representing the vehicle's linear and angular velocities
    """
    # Calculate the vehicle's linear and angular velocities using the kinematic bicycle model
    angular_velocity = linear_velocity * np.tan(steering_angle) / wheelbase

    # Create and populate the Twist message
    twist = Twist()
    twist.linear.x = linear_velocity
    twist.angular.z = angular_velocity

    return twist

# Generated by ChatGPT
def convert_twist_to_steering_angle(twist, wheelbase):
    """
    Convert a Twist message to a kinematic bicycle model's steering wheel angle command.

    :param twist: a Twist message representing the vehicle's linear and angular velocities
    :param wheelbase: the distance between the front and rear axles of the vehicle in meters
    :return: the steering wheel angle in radians
    """
    # Extract the linear and angular velocities from the Twist message
    velocity = twist.linear.x
    angular_velocity = twist.angular.z

    # Calculate the steering wheel angle using the kinematic bicycle model
    # Here we assume that the vehicle is moving forward at a constant speed of 1 m/s
    steering_angle = np.arctan(angular_velocity * wheelbase / velocity)

    return steering_angle

def lookahead_resample(path, current_pos, lookahead_distance, num_subdivisions):
    """
    Resamples a path to have a constant number of subdivisions, and returns a slice of the path that
    looks ahead a constant distance from the current position.

    Args:
    - path: list of [x,y] coordinates defining the path
    - current_pos: [x,y] coordinates of the current position of the robot
    - lookahead_distance: distance to look ahead from the current position
    - num_subdivisions: number of subdivisions to resample the path to

    Returns:
    - A list of [x,y] coordinates representing the resampled path slice
    """
    # Find the index of the point on the path closest to the current position
    dists = np.linalg.norm(np.array(path) - np.array(current_pos), axis=1)
    closest_idx = np.argmin(dists)

    # roll the path so that the closest point is at the beginning
    path = np.roll(path, -closest_idx, axis=0)
    dists = np.roll(dists, -closest_idx)
    closest_idx = 0

    # Find the index of the point on the path that is 'lookahead_distance' away from the current position
    remaining_dists = np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))
    lookahead_idx = np.searchsorted(remaining_dists, remaining_dists[closest_idx] + lookahead_distance)

    path_slice = path[closest_idx:lookahead_idx]

    # resample the path slice to have 'num_subdivisions' subdivisions
    t = np.linspace(0, 1, len(path_slice))
    resampled_t = np.linspace(0, 1, num_subdivisions)
    interp = interp1d(t, path_slice, axis=0)
    resampled_path_slice = interp(resampled_t)

    return resampled_path_slice

    # Resample the path to have 'num_subdivisions' subdivisions
    t = np.linspace(0, 1, len(path))
    resampled_t = np.linspace(0, 1, num_subdivisions)
    interp = interp1d(t, path, axis=0)
    resampled_path = interp(resampled_t)

    # Return the slice of the resampled path from the closest point to the lookahead point
    return resampled_path[closest_idx:lookahead_idx]

def test_wrap_angle():
    """
    Test the implementation of the wrap_angle function
    """
    assert wrap_angle(0) == 0
    assert wrap_angle(0.5 * np.pi) == 0.5 * np.pi
    assert wrap_angle(np.pi) == -np.pi
    assert wrap_angle(1.5 * np.pi) == -0.5 * np.pi
    assert wrap_angle(2 * np.pi) == 0

    # vector inputs
    assert_allclose(wrap_angle(np.array([0, 0.5 * np.pi, np.pi, 1.5 * np.pi, 2 * np.pi])), [0, 0.5 * np.pi, -np.pi, -0.5 * np.pi, 0], rtol=1e-5)

def test_path():
    """
    Test the implementation of the Path class
    """

    path = Path([(0, 0), (1, 0), (1, 1)], closed=True)
    assert_allclose(path.length, 2 + np.sqrt(2), rtol=1e-5)
    assert_allclose(path.lengths, [1, 1, np.sqrt(2)], rtol=1e-5)
    assert_allclose(wrap_angle(np.array(path.headings)), [0, 0.5 * np.pi, -0.75 * np.pi], rtol=1e-5)
    assert path.closed

    # Test get_point
    point = path.get_point(progress_m=0.5)
    assert_allclose(point.point, [0.5, 0], rtol=1e-5)
    assert point.segment_idx == 0
    assert_allclose(point.segment_progress_m, 0.5, rtol=1e-5)

    point = path.get_point(progress_pct=0.75)
    assert_allclose(point.point, [0.60355339, 0.60355339], rtol=1e-5)
    assert point.segment_idx == 2
    assert_allclose(point.segment_progress_m, 0.75 * path.length - 2, rtol=1e-5)

    point = path.get_point(progress_m=path.length+0.1)
    assert_allclose(point.point, [0.1, 0], rtol=1e-5)
    assert point.segment_idx == 0
    assert_allclose(point.segment_progress_m, 0.1, rtol=1e-5)

    point = path.get_point(progress_m=-1)
    assert_allclose(point.point, [np.sqrt(1/2), np.sqrt(1/2)], rtol=1e-5)
    assert point.segment_idx == 2
    assert_allclose(point.segment_progress_m, path.lengths[2] - 1, rtol=1e-5)

    # Test get_heading_at_point
    heading = path.get_heading_at_point(point)
    assert_allclose(heading, -0.75 * np.pi, rtol=1e-5)

    # Test get_closest_point
    closest_point = path.get_closest_point([1, 0.5])
    assert_allclose(closest_point.point, [1, 0.5], rtol=1e-5)
    assert closest_point.segment_idx == 1
    assert_allclose(closest_point.segment_progress_m, 0.5, rtol=1e-5)

    closest_point = path.get_closest_point([-1, 0])
    assert_allclose(closest_point.point, [0, 0], rtol=1e-5)
    assert closest_point.segment_idx == 0
    assert_allclose(closest_point.segment_progress_m, 0, rtol=1e-5)

    # Test get_local_closest_point
    local_closest_point = path.get_local_closest_point([1, 0.5], closest_point)
    assert_allclose(local_closest_point.point, [1, 0.5], rtol=1e-5)
    assert local_closest_point.segment_idx == 1
    assert_allclose(local_closest_point.segment_progress_m, 0.5, rtol=1e-5)

    # Test to_pose_array
    pose_array = path.to_pose_array()
    assert isinstance(pose_array, PoseArray)
    assert len(pose_array.poses) == 3

    # Test walk
    start_point = path.get_point(progress_m=1.0)
    end_point = path.walk(start_point, length_m=0.4)
    assert_allclose(end_point.point, [1, 0.4])
    assert end_point.segment_idx == 1
    assert_allclose(end_point.segment_progress_m, 0.4)

    # Test __eq__
    path1 = Path([(0, 0), (1, 0), (1, 1)])
    path2 = Path([(0, 0), (1, 0), (1, 1)])
    path3 = Path([(0, 0), (1, 0), (1, 1)], closed=True)
    path4 = Path([(0, 0), (1, 0), (1, 1), (0, 1)], closed=True)

    assert path1 == path2
    assert path1 != path3
    assert path3 != path4

# Add a checker for enum types
def enum_checker(value: Any, origin_type: Any, args: Tuple[Any,...], memo: TypeCheckMemo) -> None:
    if value not in [e.value for e in origin_type]:
        raise TypeCheckError(f"Expected value of type {origin_type}, got {value}")

def checker_lookup(origin_type: Any, args: Tuple[Any, ...], extras: Tuple[Any, ...]) -> Union[TypeCheckerCallable, None]:
    if issubclass(origin_type, Enum):
        return enum_checker
    return None

# patch the checker lookup functions with custom checkers
checker_lookup_functions.append(checker_lookup)

# re-export typeguard so that users always import from this module.
typeguard = _typeguard


def deep_getattr(obj, attr):
    """
    Get a nested attribute from an object.
    Examples:
    getattr(x, 'y') <==> x.y
    getattr(x, 'y.z') <==> x.y.z
    """
    if not attr:
        raise ValueError("Attribute name must not be empty")

    attributes = attr.split(".")
    for i in attributes:
        obj = getattr(obj, i)
    return obj

def test_deep_getattr():
    # Define a sample object with nested attributes for testing
    class Sample:
        class Inner:
            class InnerInner:
                value = 42
            inner_inner = InnerInner()
        inner = Inner()

    # Test case: single attribute
    obj = Sample()
    result = deep_getattr(obj, 'inner')
    assert isinstance(result, Sample.Inner), "Failed: single attribute"

    # Test case: nested attribute
    result = deep_getattr(obj, 'inner.inner_inner')
    assert isinstance(result, Sample.Inner.InnerInner), "Failed: nested attribute"

    # Test case: deeply nested attribute
    result = deep_getattr(obj, 'inner.inner_inner.value')
    assert result == 42, "Failed: deeply nested attribute"

    # Test case: nonexistent attribute
    try:
        deep_getattr(obj, 'inner.nonexistent')
    except AttributeError:
        pass  # Expected exception
    else:
        assert False, "Failed: nonexistent attribute"

    # Test case: empty attribute
    try:
        deep_getattr(obj, '')
    except ValueError:
        pass  # Expected exception
    else:
        assert False, "Failed: empty attribute"

def add_timer_event_to_diag_status(diag_msg: DiagnosticStatus, event: rospy.timer.TimerEvent):
    """
    Add all event attributes to the diagnostic status message.
    """
    for k, v in [(a, getattr(event, a)) for a in dir(event) if not a.startswith('__')]:
        diag_msg.values.append(KeyValue(key=f"event.{k}", value=str(v)))

flatten = lambda l: [item for sublist in l for item in sublist]

def powerset(iterable):
    "powerset([1,2,3]) --> () (1,) (2,) (3,) (1,2) (1,3) (2,3) (1,2,3)"
    s = list(iterable)
    return chain.from_iterable(combinations(s, r) for r in range(len(s)+1))

def make_srv_enum_lookup_dict(srv):
    """
    Make a dictionary that maps the values of a service's enum attributes to the names of those attributes.
    Example:
    `RunSomething.srv`:
    ```
    ---
    int8 SUCCESS = 0
    int8 FAILURE = 1
    int8 UNKNOWN = 2
    int8 result
    ```
    ```python
    from run_something.srv import RunSomethingResponse
    lookup = make_srv_enum_lookup_dict(RunSomethingResponse)
    print(lookup[0])  # prints "SUCCESS"
    print(lookup[1])  # prints "FAILURE"
    print(lookup[2])  # prints "UNKNOWN"
    ```
    """
    return {getattr(srv,k): k for k in dir(srv) if getattr(srv,k).__class__ == int}

def assert_and_remove_suffix(s, suffix):
    """
    Assert that a string ends with a given suffix, and return the string without the suffix.
    """

    assert s.endswith(suffix), f"Expected {s} to end with {suffix}"
    return s[:-len(suffix)]

def calculate_signed_distance(x1, y1, x2, y2, x0, y0):
    """
    Calculates the signed distance (lateral error) from point P(x0, y0) to the line defined by points A(x1, y1) and B(x2, y2).
    When the point P lies above the line when looking left to right from A to B, the distance is positive.
    When the point P lies below the line when looking left to right from A to B, the distance is negative.
    
    Parameters:
        x1, y1: Coordinates of point A.
        x2, y2: Coordinates of point B.
        x0, y0: Coordinates of point P.
        
    Returns:
        The signed distance (float) from point P to the line AB.
    
    Written by ChatGPT
    """
    # Coefficients of the line equation: Ax + By + C = 0
    A = y2 - y1
    B = x1 - x2
    C = x2 * y1 - x1 * y2
    
    # Calculate the signed distance
    d = -(A * x0 + B * y0 + C) / sqrt(A**2 + B**2)
    return d


def test_calculate_signed_distance():
    # Test case 1: Point lies on the line
    assert abs(calculate_signed_distance(0, 0, 1, 1, 2, 2) - 0.0) < 1e-9

    # Test case 2: Point lies on the negative side of the line
    assert abs(calculate_signed_distance(0, 0, 1, 1, 1, 0) + 0.7071067811865475) < 1e-9

    # Test case 3: Point lies on the positive side of the line
    assert abs(calculate_signed_distance(0, 0, 1, 1, 0, 1) - 0.7071067811865475) < 1e-9

    # Test case 4: Vertical line
    assert abs(calculate_signed_distance(0, 0, 0, 1, 1, 0) + 1.0) < 1e-9

    # Test case 5: Horizontal line
    assert abs(calculate_signed_distance(0, 0, 1, 0, 0, 1) - 1.0) < 1e-9

    # Test case 6: Point is one of the defining points of the line
    assert abs(calculate_signed_distance(0, 0, 1, 1, 0, 0) + 0.0) < 1e-9


if __name__ == "__main__":
    test_wrap_angle()
    test_path()
    test_calculate_signed_distance()
    test_deep_getattr()

    print("All tests passed!")