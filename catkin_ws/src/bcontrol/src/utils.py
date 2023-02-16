import math
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from tf.transformations import quaternion_from_euler
from dataclasses import dataclass

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
    def __init__(self, path, closed=False):
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
        
    def get_point(self, progress_m=None, progress_pct=None):
        """
        Returns the point on the path at the given progress. The runtime of this function is O(n), where n is the number of path segments.
        
        Arguments:
            progress_m: The progress along the path in meters. If the path is closed, this value can be greater than the path length.
            progress_pct: The progress along the path as a percentage. If the path is closed, this value can be greater than 1.
        Returns:
            The point on the path at the given progress.
        """

        if progress_m is None and progress_pct is None:
            raise ValueError('Either progress_m or progress_pct must be specified.')
        if progress_m is not None and progress_pct is not None:
            raise ValueError('Only one of progress_m or progress_pct can be specified.')
        if progress_m is None:
            progress_m = progress_pct * self.length

        if self.closed:
            progress_m = progress_m % self.length
        if progress_m < 0:
            progress_m = 0
        if progress_m > self.length:
            progress_m = self.length
        for i in range(len(self.path) - 1):
            if progress_m <= self.lengths[i]:
                return self.path[i] + (self.path[i + 1] - self.path[i]) * progress_m / self.lengths[i]
            progress_m -= self.lengths[i]
        return self.path[-1]

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

        return closest
    
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
    
    def walk(self, starting_point, length_m):
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

def get_point_on_segment(progress_m, p1, p2, v_length_m=None):
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

def pathpoints_to_pose_array(pathpoints, path):
    """
    Converts a list of PathPoint objects to a PoseArray message.
    """
    pose_array = PoseArray()
    pose_array.header.frame_id = "map"
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


# class PathWalker:
#     """
#     Takes in a path and a starting position (segment, progress along segment) and implements a walk(distance) function that returns the new position (segment, progress along segment) after walking the given distance along the path. If the path is closed, the robot will return to the start point when it reaches the end point. If the path is not closed, the robot will stop at the end point when it reaches the end point. The return value will signal whether the robot has reached the end point.
#     """

# Remaining functions:

# PathWalker - takes in a path and a starting position (segment, progress along segment) and implements a walk(distance) function that returns the new position (segment, progress along segment) after walking the given distance along the path. If the path is closed, the robot will return to the start point when it reaches the end point. If the path is not closed, the robot will stop at the end point when it reaches the end point. The return value will signal whether the robot has reached the end point.
# What if the path updates?

# The control loop:
# - Get the current position and heading of the robot (from estimate)
# - Get the closest point on the path to the robot (initialize along a path, then do local search)
# - Look ahead X meters along the path, set this as the target point (or the end point if the path is not closed, this can be done using the PathWalker)
# - Based on the angle difference and the distance to the target point, calculate the desired angular velocity







# local_get_closest_point_on_path(path, point, idx, progress_m) - returns the closest point on the path to the given point. The return value is a dictionary with the following keys:
#     idx: The index of the closest path segment on the path
#     point: The closest point on the path
#     dist: The distance between the closest point on the path and the given point
#     progress_m: The distance along the path segment to the closest point on the path
# This function employs gradient descent where the gradient is whether a neighbouring segment is closer. This is a local search algorithm and is not guaranteed to find the global minimum. However, it is fast and works well in practice.
