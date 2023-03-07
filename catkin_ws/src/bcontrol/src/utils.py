import functools
import math
import numpy as np
from numpy.testing import assert_allclose
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from tf.transformations import quaternion_from_euler
from dataclasses import dataclass
from geometry_msgs.msg import Twist
from scipy.interpolate import interp1d
from typing import List, Tuple, Union, Any
import typeguard as _typeguard
from typeguard import checker_lookup_functions, TypeCheckerCallable, TypeCheckMemo, TypeCheckError
from enum import Enum

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
    
    def __init__(self, path: List[Tuple[float,float]], closed=False):
        """
        Arguments:
            path: A list of points, each point is a tuple (x, y). The path is linearly interpolated.
            closed: Whether the path is closed. If the path is closed, the robot will return to the start point when it reaches the end point.
        """
        self.path = path
        self.length = 0
        self.lengths = []
        self.headings = []
        self.closed = closed
        for i in range(len(self.path) - 1):
            # Calculate the length of each path segment
            self.lengths.append(np.linalg.norm(np.array(self.path[i + 1]) - np.array(self.path[i])))
            # Calculate the total length of the path
            self.length += self.lengths[-1]

        # Calculate the heading of each path segment
        for i in range(len(self.path) - 1):
            self.headings.append(np.arctan2(self.path[i + 1][1] - self.path[i][1], self.path[i + 1][0] - self.path[i][0]))
        
        if closed:
            # Add the length of the last point to the first point
            self.lengths.append(np.linalg.norm(np.array(self.path[0]) - np.array(self.path[-1])))
            self.length += self.lengths[-1]

            # Add the heading of the last point to the first point
            self.headings.append(np.arctan2(self.path[0][1] - self.path[-1][1], self.path[0][0] - self.path[-1][0]))
    
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
        return cls(path, closed)
        
    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Path):
            return NotImplemented
        
        return self.path == other.path and self.closed == other.closed
        
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
                    point=get_point_on_segment(progress_m, self.path[i], self.path[(i + 1) % len(self.path)], segment_length)
                )
            progress_m -= segment_length
        
        raise ValueError(f"Couldn't fine the point in path! This is impossible. {progress_m=}")

    def get_heading_at_point(self, path_point):
        """
        Returns the heading of the path at the given point.
        """
        if path_point.segment_idx is None:
            raise ValueError('PathPoint object must have a segment_idx attribute.')

        return self.headings[path_point.segment_idx]

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
            segments = zip(self.path, self.path[1:] + [self.path[0]])
        else:
            segments = zip(self.path[:-1], self.path[1:])

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
            segments = list(zip(self.path, self.path[1:] + [self.path[0]]))
        else:
            segments = list(zip(self.path[:-1], self.path[1:]))

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
            query_slice = slice(0, len(self.path))

        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        for i, p in enumerate(self.path[query_slice]):
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            heading = self.headings[i] if i < len(self.headings) else self.headings[-1]
            pose.orientation = Quaternion(*quaternion_from_euler(0, 0, heading))
            pose_array.poses.append(pose)
        return pose_array
    
    def walk(self, starting_point: PathPoint, length_m: float):
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
            segments = list(zip(self.path, self.path[1:] + [self.path[0]]))
        else:
            segments = list(zip(self.path[:-1], self.path[1:]))

        # The maximum number of iterations to prevent infinite loops
        max_iterations = len(segments) * math.ceil(length_m / self.length)
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
    Returns a list of points on the circle.
    """

    points = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        points.append([
            center[0] + radius * math.cos(angle),
            center[1] + radius * math.sin(angle)
        ])
    return points

# Generated by ChatGPT
def generate_figure_eight_approximation(center, radius, num_points):
    """
    Generates a list of points on a figure eight shape.
    Arguments:
        center: The center of the figure eight shape.
        radius: The distance from the center to the middle of the figure eight shape.
        num_points: The number of points to generate. The more points, the more accurate the approximation.
    Returns a list of points on the figure eight shape.
    """
    points = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = center[0] + radius * math.sin(angle)
        y = center[1] + radius * (math.sin(angle * 2) / 2)
        points.append([x, y])
    return points

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

def deep_getattr(obj, attr, *args):
    """
    Gets an attribute of an object recursively. For example, deep_getattr(obj, "a.b.c") is equivalent to obj.a.b.c.
    """
    return functools.reduce(getattr, [obj] + attr.split("."), *args)


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


if __name__ == "__main__":
    test_wrap_angle()
    test_path()

    print("All tests passed!")