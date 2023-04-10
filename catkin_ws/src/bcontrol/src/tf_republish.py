#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
from importlib import import_module
from transform_frames import TransformFrames
from nav_msgs.msg import Odometry

class RepublishMessageInDifferentFrame:
    def __init__(self, topic, output_topic, message_type, target_frame):
        self.target_frame = target_frame
        self.message_type = message_type
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        if message_type == Odometry:
            rospy.Subscriber(topic, message_type, self.odom_callback)
        else:
            raise Exception(f"Unsupported message type: {message_type}")
        
        self.pub = rospy.Publisher(output_topic, message_type, queue_size=10)
        self.transform_frames = TransformFrames()

    def odom_callback(self, msg: Odometry):
        try:
            self.pub.publish(self.transform_frames.odom_transform(msg))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            rospy.logwarn("Could not transform message! " + str(e))

if __name__ == '__main__':
    rospy.init_node('republish_message_in_different_frame')
    topic = rospy.get_param('~topic')
    output_topic = rospy.get_param('~output_topic')
    message_type_str = rospy.get_param('~message_type')
    target_frame = rospy.get_param('~target_frame')

    message_type_module, message_type_class = message_type_str.rsplit('.', 1)
    module = import_module(message_type_module)
    message_type = getattr(module, message_type_class)

    RepublishMessageInDifferentFrame(topic, output_topic, message_type, target_frame)
    rospy.spin()
