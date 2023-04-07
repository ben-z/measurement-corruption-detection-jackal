from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import rospy

class OdometryConverter:
    FIELD_ORIENTATION = 'orientation'
    FIELD_LINEAR_VEL_X = 'linear_vel_x'
    FIELD_ANGULAR_VEL_Z = 'angular_vel_z'
    
    @staticmethod
    def to_numpy(odom_msg):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        quaternion = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        
        # Extract linear x velocity and angular velocity
        linear_vel_x = odom_msg.twist.twist.linear.x
        angular_vel_z = odom_msg.twist.twist.angular.z
        
        return np.array([yaw, linear_vel_x, angular_vel_z])
    
    @staticmethod
    def from_numpy(np_data, odom_msg_template=None):
        # If no template is provided, create a new Odometry message
        if odom_msg_template is None:
            odom_msg = Odometry()
        else:
            odom_msg = odom_msg_template
        
        # Extract data from numpy array
        yaw, linear_vel_x, angular_vel_z = np_data
        
        # Set linear x velocity and angular velocity
        odom_msg.twist.twist.linear.x = linear_vel_x
        odom_msg.twist.twist.angular.z = angular_vel_z
        
        # Set orientation
        q = quaternion_from_euler(0, 0, yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        return odom_msg
    
    @staticmethod
    def get_indices(field_selector):
        field_to_indices_map = {
            OdometryConverter.FIELD_ORIENTATION: [0],
            OdometryConverter.FIELD_LINEAR_VEL_X: [1],
            OdometryConverter.FIELD_ANGULAR_VEL_Z: [2]
        }
        
        if field_selector not in field_to_indices_map:
            raise ValueError(f"Invalid field selector '{field_selector}' for Odometry message.")
        
        return field_to_indices_map[field_selector]


class ImuConverter:
    FIELD_ANGULAR_VEL_Z = 'angular_vel_z'
    
    @staticmethod
    def to_numpy(imu_msg):
        # Extract the z-component of the angular velocity
        angular_vel_z = imu_msg.angular_velocity.z
        
        return np.array([angular_vel_z])
    
    @staticmethod
    def from_numpy(np_data, imu_msg_template=None):
        # If no template is provided, create a new Imu message
        if imu_msg_template is None:
            imu_msg = Imu()
        else:
            imu_msg = imu_msg_template
        
        # Extract data from numpy array
        angular_vel_z, = np_data
        
        # Set the z-component of the angular velocity
        imu_msg.angular_velocity.z = angular_vel_z
        
        return imu_msg
    
    @staticmethod
    def get_indices(field_selector):
        field_to_indices_map = {
            ImuConverter.FIELD_ANGULAR_VEL_Z: [0]
        }
        
        if field_selector not in field_to_indices_map:
            raise ValueError(f"Invalid field selector '{field_selector}' for Imu message.")
        
        return field_to_indices_map[field_selector]
