# A message containing a path

std_msgs/Header header

geometry_msgs/Pose[] poses

# These twists are in the robot body frame (i.e. linear x velocity is forward velocity)
geometry_msgs/Twist[] twists

float64[] curvatures

# The derivative of the curvature with respect to the path length
float64[] dK_ds_list
