#!/usr/bin/env python
# Required for ROS to execute file as a Python script

import rospy
import roslaunch
import numpy as np


# Deterministic mode start/end points
maze1StartPoints = [[-1.5, -2.5],
                    [0.5, -2.5],
                    [2.5, -2.5]]

maze2StartPoints = [[-0.5, -4.5],
                    [1, -4.5],
                    [3, -4.5]]

maze3StartPoints = [[-0.5, -3],
                    [1.25, -3],
                    [3.5, -3]]

maze1EndPoints = [[-1.5, 2.4],
                  [0.5, 2.4],
                  [2.5, 2.4]]

maze2EndPoints = [[-0.5, 0.5],
                  [1, 0.5],
                  [3, 0.5]]

maze3EndPoints = [[-0.5, 2],
                  [1.25, 2],
                  [3.5, 2]]

# Ground Truth Paths
maze1gtPathWPs = np.array([
            [1, -1],
            [1, 1]])

maze2gtPathWPs = np.array([
            [1, -4],
            [1, -1.5]])

maze3gtPathWPs = np.array([
            [3, -2],
            [3, -1],
            [1.5, -1],
            [1.5, 0.5],])


# Deterministic mode start/end point test case combinations
maze1StartEndCombos = np.array([[maze1StartPoints[2], maze1EndPoints[2]]])
maze2StartEndCombos = np.array([[maze2StartPoints[2], maze2EndPoints[2]]])
maze3StartEndCombos = np.array([[maze3StartPoints[2], maze3EndPoints[2]]])


# Prompt user for mode selection
mode = raw_input('Enter "d" for Deterministic mode or "s" for Stochastic mode: ')

while (mode != 'd' and mode != 's'):
    mode = raw_input('Invalid input. Enter "d" for Deterministic mode or "s" for Stochastic mode: ')


# Execute different simulation modes based on user selection
if (mode == 'd'):
    totalSims = 0
    successSims = 0

    for combo in maze1StartEndCombos:
        totalSims += 1
        startPoint = combo[0]
        endPoint = combo[1]

        #rospy.set_param('endPointX', float(endPoint[0]))
        #rospy.set_param('endPointY', float(endPoint[1]))
        rospy.set_param('endPoint', endPoint.tolist())

        # Set full ground truth path
        #gtPathWPs = np.concatenate((np.concatenate((startPoint, maze3gtPathWPs), axis=0), endPoint), axis=0)
        gtPathWPs = np.insert(maze1gtPathWPs, 0, startPoint, axis=0)
        gtPathWPs = np.insert(gtPathWPs, len(gtPathWPs), endPoint, axis=0)
        rospy.set_param('gtPathWPs', gtPathWPs.tolist())

        x_pos_arg = 'x_pos:=' + str(startPoint[0])
        y_pos_arg = 'y_pos:=' + str(startPoint[1])
        cli_args = ['/home/kevin/Workspaces/ENSE623/src/mazerunner/launch/maze1.launch', x_pos_arg, y_pos_arg]
        
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch_parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

        roslaunch_parent.start()

        # Do not terminate launch parent if simulation is not complete
        rospy.sleep(5)
        simComplete = rospy.get_param('simComplete')
        while (simComplete == False):
            simComplete = rospy.get_param('simComplete')
            continue
        
        runTime = rospy.get_param('runTime')
        if (runTime != 'FAILED'):
            successSims += 1

        roslaunch_parent.shutdown()

        rospy.sleep(10)
    
    for combo in maze2StartEndCombos:
        totalSims += 1
        startPoint = combo[0]
        endPoint = combo[1]

        #rospy.set_param('endPointX', float(endPoint[0]))
        #rospy.set_param('endPointY', float(endPoint[1]))
        rospy.set_param('endPoint', endPoint.tolist())

        # Set full ground truth path
        #gtPathWPs = np.concatenate((np.concatenate((startPoint, maze3gtPathWPs), axis=0), endPoint), axis=0)
        gtPathWPs = np.insert(maze2gtPathWPs, 0, startPoint, axis=0)
        gtPathWPs = np.insert(gtPathWPs, len(gtPathWPs), endPoint, axis=0)
        rospy.set_param('gtPathWPs', gtPathWPs.tolist())

        x_pos_arg = 'x_pos:=' + str(startPoint[0])
        y_pos_arg = 'y_pos:=' + str(startPoint[1])
        cli_args = ['/home/kevin/Workspaces/ENSE623/src/mazerunner/launch/maze2.launch', x_pos_arg, y_pos_arg]
        
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch_parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

        roslaunch_parent.start()

        # Do not terminate launch parent if simulation is not complete
        rospy.sleep(5)
        simComplete = rospy.get_param('simComplete')
        while (simComplete == False):
            simComplete = rospy.get_param('simComplete')
            continue
        
        runTime = rospy.get_param('runTime')
        if (runTime != 'FAILED'):
            successSims += 1
            

        roslaunch_parent.shutdown()

        rospy.sleep(10)
    
    for combo in maze3StartEndCombos:
        totalSims += 1
        startPoint = combo[0]
        endPoint = combo[1]

        #rospy.set_param('endPointX', float(endPoint[0]))
        #rospy.set_param('endPointY', float(endPoint[1]))
        rospy.set_param('endPoint', endPoint.tolist())

        # Set full ground truth path
        #gtPathWPs = np.concatenate((np.concatenate((startPoint, maze3gtPathWPs), axis=0), endPoint), axis=0)
        gtPathWPs = np.insert(maze3gtPathWPs, 0, startPoint, axis=0)
        gtPathWPs = np.insert(gtPathWPs, len(gtPathWPs), endPoint, axis=0)
        rospy.set_param('gtPathWPs', gtPathWPs.tolist())

        x_pos_arg = 'x_pos:=' + str(startPoint[0])
        y_pos_arg = 'y_pos:=' + str(startPoint[1])
        cli_args = ['/home/kevin/Workspaces/ENSE623/src/mazerunner/launch/maze3.launch', x_pos_arg, y_pos_arg]
        
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch_parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

        roslaunch_parent.start()

        # Do not terminate launch parent if simulation is not complete
        rospy.sleep(5)
        simComplete = rospy.get_param('simComplete')
        while (simComplete == False):
            simComplete = rospy.get_param('simComplete')
            continue
        
        runTime = rospy.get_param('runTime')
        if (runTime != 'FAILED'):
            successSims += 1
            
        roslaunch_parent.shutdown()

        rospy.sleep(10)

    successRate = float(successSims) / totalSims
    print('Success Rate: ', successRate)


elif (mode == 's'):
    runTime = rospy.get_param('runTime')
    navDistance = rospy.get_param('navDistance')
    meanLocomotionError = rospy.get_param('meanLocomotionError')





