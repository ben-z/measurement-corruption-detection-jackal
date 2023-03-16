#!/usr/bin/env python3
import debugpy

# Debugger configuration reference:
# https://answers.ros.org/question/301015/ros-debugging-vscode-python/
# https://code.visualstudio.com/docs/containers/docker-compose
debugpy.listen(("0.0.0.0", 5678))
print("Debugpy listening on port 5678. Waiting for a connection...")
debugpy.wait_for_client()

import rospy
from solver_server import main

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Got ROSInterruptException. Exiting...")
