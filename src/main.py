#!/usr/bin/env python
# Required for ROS to execute file as a Python script

import sys
sys.dont_write_bytecode = True

import rospy
import numpy as np

from scan_twist_center_control_ import ScanTwistCenterControlNode
from helper._pd_control import PDControlLoop


# Maze 1 Ground Truth Paths
gtPathWPs = np.array([
            [2.5, -2.5],
            [0, -2.5],
            [0, -1],
            [1, -1],
            [1, 1.3],
            [0, 1.3],
            [0, 2.4],
            [2.5, 2.4]])

'''
# Maze 2 Ground Truth Paths
gtPathWPs = np.array([
            [-2, -2.5],
            [0, -2.5],
            [0, 0.75],
            [-2, 0.75],
            [-2, 2.5]])

# Maze 3 Ground Truth Paths
gtPathWPs = np.array([
            [-1.5, -2.5],
            [-1.5, -0.5],
            [0.25, -0.5],
            [0.25, 0.5],
            [-0.5, 0.5],
            [-0.5, 2.4],
            [-1.5, 2.4]])
'''

# Initialize ROS node
rospy.init_node('ros_maze_bot', disable_signals=True)

# Customize settings
kwargs = {
    'distance_to_wall_desired': 0.25,
    'max_speed': 0.25,
}

# Endpoints
#endPoint = np.array([0, -2.5])
endPoint = np.array([2.5,2.4])

scan_monitor = ScanTwistCenterControlNode(
    endPoint=endPoint,
    gtPathWPs=gtPathWPs,
    scan_topic="/scan",
    pub_topic="cmd_vel",
    policy='LHR', 
    helper_controller=PDControlLoop, **kwargs)

print('BREAK1.5')

if __name__ == "__main__":
    print('BREAK2')
    print(len(sys.argv))
    if len(sys.argv) < 2:
        print("Not enough arguments")
    else:
        print(sys.argv[1])
    #test = rospy.get_param('test')
    #print(test)
    scan_monitor.start()



