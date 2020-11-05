#!/usr/bin/env python
# Required for ROS to execute file as a Python script

import sys
sys.dont_write_bytecode = True

import rospy

from scan_twist_center_control_ import ScanTwistCenterControlNode
from helper._pd_control import PDControlLoop


# Initialize ROS node
rospy.init_node('ros_maze_bot', disable_signals=True)

# Customize settings
kwargs = {
    'distance_to_wall_desired': 0.3,
    'max_speed': 0.25,
}

scan_monitor = ScanTwistCenterControlNode(
    scan_topic="/scan", 
    pub_topic="cmd_vel",
    policy='LHR', 
    helper_controller=PDControlLoop, **kwargs)



if __name__ == "__main__":

    scan_monitor.start()


