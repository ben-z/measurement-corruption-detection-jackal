import numpy as np

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
        self.closed = closed
        for i in range(len(self.path) - 1):
            self.lengths.append(np.linalg.norm(np.array(self.path[i + 1]) - np.array(self.path[i])))
            self.length += self.lengths[-1]
        if closed:
            self.lengths.append(np.linalg.norm(np.array(self.path[0]) - np.array(self.path[-1])))
            self.length += self.lengths[-1]
        
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

    def get_closest_point(self, point):
        """
        Returns the closest point on the path to the given point.
        
        Arguments:
            point: The point to which the closest point on the path should be found.
        Returns a dictionary with the following keys:
            idx: The index of the closest path segment on the path
            point: The closest point on the path
            dist: The distance between the closest point on the path and the given point
            progress_m: The distance along the path segment to the closest point on the path
        """
        closest = None
        for i in range(len(self.path) - 1):
            p = self.get_closest_point_on_segment(point, self.path[i], self.path[i + 1])
            if closest is None or p['dist'] < closest['dist']:
                closest = p
                closest['idx'] = i
        if self.closed:
            p = self.get_closest_point_on_segment(point, self.path[-1], self.path[0])
            if closest is None or p['dist'] < closest['dist']:
                closest = p
                closest['idx'] = len(self.path) - 1
        return closest
    
    def get_closest_point_on_segment(self, point, p1, p2):
        """
        Returns the closest point on the segment to the given point.

        Arguments:
            point: The point to which the closest point on the segment should be found.
            p1: The first point of the segment.
            p2: The second point of the segment.
        Returns a dictionary with the following keys:
            point: The closest point on the segment
            dist: The distance between the closest point on the segment and the given point
            progress_m: The distance along the segment to the closest point on the segment
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
        return {
            'point': closest,
            'dist': np.linalg.norm(closest - p),
            'progress_m': progress_m
        }

# Remaining functions:

# PathWalker - takes in a path and a starting position (segment, progress along segment) and implements a walk(distance) function that returns the new position (segment, progress along segment) after walking the given distance along the path. If the path is closed, the robot will return to the start point when it reaches the end point. If the path is not closed, the robot will stop at the end point when it reaches the end point. The return value will signal whether the robot has reached the end point.

# local_get_closest_point_on_path(path, point, idx, progress_m) - returns the closest point on the path to the given point. The return value is a dictionary with the following keys:
#     idx: The index of the closest path segment on the path
#     point: The closest point on the path
#     dist: The distance between the closest point on the path and the given point
#     progress_m: The distance along the path segment to the closest point on the path
# This function employs gradient descent where the gradient is whether a neighbouring segment is closer. This is a local search algorithm and is not guaranteed to find the global minimum. However, it is fast and works well in practice.
