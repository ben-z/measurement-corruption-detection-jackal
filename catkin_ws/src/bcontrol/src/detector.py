# TODO: Implement the detector
# Inputs: odom from IMU, x, y, orientation from GPS, path from the planner
# Outputs:
# - Whether each sensor is corrupted
# - Passthrough of sensors that are not corrupted

datacollector_config = {
    'sensors': [
        {
            'topic': '/odom',
            'type': 'Odometry',
            'fields': [
            ],
        }
    ]
}