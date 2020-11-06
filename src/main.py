#!/usr/bin/env python
# Required for ROS to execute file as a Python script

import sys
sys.dont_write_bytecode = True

import rospy
import numpy as np

from scan_twist_center_control_ import ScanTwistCenterControlNode
from helper._pd_control import PDControlLoop


# Initialize ROS node
rospy.init_node('ros_maze_bot', disable_signals=True)

# Customize settings
kwargs = {
    'distance_to_wall_desired': 0.25,
    'max_speed': 0.25,
}

#endPoint = np.array([-1.5, 2.4])
endPoint = np.array([0, -2.5])

scan_monitor = ScanTwistCenterControlNode(
    endPoint=endPoint,
    scan_topic="/scan", 
    pub_topic="cmd_vel",
    policy='LHR', 
    helper_controller=PDControlLoop, **kwargs)



if __name__ == "__main__":
    scan_monitor.start()


