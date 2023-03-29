#!/usr/bin/env python
''' transform_frames.py

    Class to transform point and coordinates between robot frames

    Derived from https://gitlab.msu.edu/av/av_notes/-/blob/6d82e0d460af71526b83dabdba99d4f274231baf/ROS/python/transform_frames.py

    Daniel Morris, April 2020, Nov 2021    
    Copyright 2020, 2021
'''
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped, PointStamped, TransformStamped, PoseWithCovariance
from std_msgs.msg import Header
import tf2_ros, tf2_geometry_msgs
from nav_msgs.msg import Odometry
from bcontrol.msg import Path as PathMsg

class TransformFrames():
    def __init__(self):
        ''' Create a buffer of transforms and update it with TransformListener '''
        self.tfBuffer = tf2_ros.Buffer()           # Creates a frame buffer
        tf2_ros.TransformListener(self.tfBuffer)   # TransformListener fills the buffer as background task
    
    def get_transform(self, source_frame: str, target_frame: str, stamp=rospy.Time(0), duration=rospy.Duration(0.2)) -> TransformStamped:
        ''' Lookup transform between source_frame and target_frame from the buffer 
            stamp: time for which to look up transform
        '''
        try:
            trans = self.tfBuffer.lookup_transform(target_frame, source_frame, stamp, duration )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f'Cannot find transformation from {source_frame} to {target_frame}')
            raise Exception(f'Cannot find transformation from {source_frame} to {target_frame}') from e
        return trans     # Type: TransformStamped

    def pose_array_transform(self, pose_array: PoseArray, target_frame: str='odom') -> PoseArray:
        ''' pose_array: will be transformed to target_frame '''
        trans = self.get_transform( pose_array.header.frame_id, target_frame, pose_array.header.stamp )
        new_header = Header(frame_id=target_frame, stamp=pose_array.header.stamp) 
        pose_array_transformed = PoseArray(header=new_header)
        for pose in pose_array.poses:
            pose_s = PoseStamped(pose=pose, header=pose_array.header)
            pose_t = tf2_geometry_msgs.do_transform_pose(pose_s, trans)
            pose_array_transformed.poses.append( pose_t.pose )
        return pose_array_transformed

    def path_msg_transform(self, path_msg: PathMsg, target_frame: str='odom') -> PathMsg:
        trans = self.get_transform( path_msg.header.frame_id, target_frame, path_msg.header.stamp )
        new_header = Header(frame_id=target_frame, stamp=path_msg.header.stamp)
        path_msg_transformed = PathMsg(header=new_header, twists=path_msg.twists, curvatures=path_msg.curvatures, dK_ds_list=path_msg.dK_ds_list)
        for pose in path_msg.poses:
            pose_s = PoseStamped(pose=pose, header=path_msg.header)
            pose_t = tf2_geometry_msgs.do_transform_pose(pose_s, trans)
            path_msg_transformed.poses.append( pose_t.pose )
        
        return path_msg_transformed

    def point_transform(self, point: PointStamped, target_frame: str='odom') -> PointStamped:
        ''' Transform a PointStamped to a new frame '''
        trans = self.get_transform( point.header.frame_id, target_frame, point.header.stamp )
        return tf2_geometry_msgs.do_transform_point(point, trans )

    def get_frame_A_origin_frame_B(self, frame_A: str, frame_B: str, stamp=rospy.Time(0) ) -> PoseStamped:
        ''' Returns the pose of the origin of frame_A in frame_B as a PoseStamped '''
        header = Header(frame_id=frame_A, stamp=stamp)        
        origin_A = Pose(position=Point(0.,0.,0.), orientation=Quaternion(0.,0.,0.,1.))
        origin_A_stamped = PoseStamped( pose=origin_A, header=header )
        pose_frame_B = tf2_geometry_msgs.do_transform_pose(origin_A_stamped, self.get_transform(frame_A, frame_B, stamp))
        return pose_frame_B
    
    def odom_transform(self, odom: Odometry, target_frame: str='odom') -> Odometry:
        ''' Transform an Odometry message to a new frame '''
        trans = self.get_transform( odom.header.frame_id, target_frame, odom.header.stamp )
        new_header = Header(frame_id=target_frame, stamp=odom.header.stamp)
        pose_s = PoseStamped(pose=odom.pose.pose, header=odom.header)
        pose_t = tf2_geometry_msgs.do_transform_pose(pose_s, trans)
        odom_t = Odometry(header=new_header, child_frame_id=odom.child_frame_id, pose=PoseWithCovariance(pose=pose_t.pose))
        odom_t.twist = odom.twist
        return odom_t
        

