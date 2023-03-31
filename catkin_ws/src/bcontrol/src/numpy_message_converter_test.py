import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from numpy_message_converter import ImuConverter, OdometryConverter

def test_odometry_converter():
    # Test data for Odometry message
    odom_msg = Odometry()
    odom_msg.pose.pose.orientation.x = 0.0
    odom_msg.pose.pose.orientation.y = 0.0
    odom_msg.pose.pose.orientation.z = np.sin(np.pi/8)  # sin(pi/8)
    odom_msg.pose.pose.orientation.w = np.cos(np.pi/8)  # cos(pi/8)
    odom_msg.twist.twist.linear.x = 2.0
    odom_msg.twist.twist.angular.z = 1.0
    
    # Convert Odometry message to NumPy array
    np_data = OdometryConverter.to_numpy(odom_msg)
    
    # Expected NumPy array: [yaw, linear_x_vel, angular_vel]
    expected_np_data = np.array([np.pi / 4, 2.0, 1.0])
    
    # Test to_numpy method
    np.testing.assert_array_almost_equal(np_data, expected_np_data, decimal=8)
    
    # Convert NumPy array back to Odometry message
    new_odom_msg = OdometryConverter.from_numpy(np_data, odom_msg_template=odom_msg)
    
    # Test from_numpy method
    assert new_odom_msg.twist.twist.linear.x == 2.0
    assert new_odom_msg.twist.twist.angular.z == 1.0
    
    # Test get_indices method
    assert OdometryConverter.get_indices(OdometryConverter.FIELD_ORIENTATION) == [0]
    assert OdometryConverter.get_indices(OdometryConverter.FIELD_LINEAR_X_VEL) == [1]
    assert OdometryConverter.get_indices(OdometryConverter.FIELD_ANGULAR_VEL) == [2]

def test_imu_converter():
    # Test data for Imu message
    imu_msg = Imu()
    imu_msg.angular_velocity.x = 0.0
    imu_msg.angular_velocity.y = 0.0
    imu_msg.angular_velocity.z = 1.5
    
    # Convert Imu message to NumPy array
    np_data = ImuConverter.to_numpy(imu_msg)
    
    # Expected NumPy array: [angular_vel_z]
    expected_np_data = np.array([1.5])
    
    # Test to_numpy method
    np.testing.assert_array_almost_equal(np_data, expected_np_data, decimal=8)
    
    # Convert NumPy array back to Imu message
    new_imu_msg = ImuConverter.from_numpy(np_data, imu_msg_template=imu_msg)
    
    # Test from_numpy method
    assert new_imu_msg.angular_velocity.x == 0.0
    assert new_imu_msg.angular_velocity.y == 0.0
    assert new_imu_msg.angular_velocity.z == 1.5
    
    # Test get_indices method
    assert ImuConverter.get_indices(ImuConverter.FIELD_ANGULAR_VEL_Z) == [0]

# Run tests
test_odometry_converter()
test_imu_converter()
