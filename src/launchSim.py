#!/usr/bin/env python
# Required for ROS to execute file as a Python script

import rospy
import roslaunch
import numpy as np
import random
import math


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
#maze1StartEndCombos = np.array([[maze1StartPoints[2], maze1EndPoints[2]]])
maze2StartEndCombos = np.array([[maze2StartPoints[2], maze2EndPoints[2]]])
#maze3StartEndCombos = np.array([[maze3StartPoints[2], maze3EndPoints[2]]])
maze1StartEndCombos = np.array([])
maze3StartEndCombos = np.array([])

# Number of runs per maze in stochastic mode
runsPerMaze = 3

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

        rospy.sleep(5)
    
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

        rospy.sleep(5)
    
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

        rospy.sleep(5)

    print('Success Rate: ', float(successSims) / totalSims)


elif (mode == 's'):
    totalSims = 0
    successSims = 0

    runTimeList = []
    navDistanceList = []
    locomotionErrorList = []
    startXValues = []
    endXValues = []
    for i in range(runsPerMaze):
        totalSims += 1

        # Generate random start and end points along the x-axis
        startPointX = random.uniform(maze1StartPoints[0][0], maze1StartPoints[2][0])
        endPointX = random.uniform(maze1EndPoints[0][0], maze1EndPoints[2][0])

        startPoint = np.array([startPointX, maze1StartPoints[0][1]])
        endPoint = np.array([endPointX, maze1EndPoints[0][1]])

        startXValues.append(startPointX)
        endXValues.append(endPointX)

        # Generate random orientation
        yaw = random.uniform(0, 2*math.pi)

        rospy.set_param('endPoint', endPoint.tolist())

        # Set full ground truth path
        #gtPathWPs = np.concatenate((np.concatenate((startPoint, maze3gtPathWPs), axis=0), endPoint), axis=0)
        gtPathWPs = np.insert(maze1gtPathWPs, 0, startPoint, axis=0)
        gtPathWPs = np.insert(gtPathWPs, len(gtPathWPs), endPoint, axis=0)
        rospy.set_param('gtPathWPs', gtPathWPs.tolist())

        x_pos_arg = 'x_pos:=' + str(startPoint[0])
        y_pos_arg = 'y_pos:=' + str(startPoint[1])
        yaw_arg = 'yaw:=' + str(yaw)
        cli_args = ['/home/kevin/Workspaces/ENSE623/src/mazerunner/launch/maze1.launch', x_pos_arg, y_pos_arg, yaw_arg]
        
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
            runTimeList.append(rospy.get_param('runTime'))
            navDistanceList.append(rospy.get_param('navDistance'))
            locomotionErrorList.append(rospy.get_param('meanLocomotionError'))

        roslaunch_parent.shutdown()

        rospy.sleep(5)

    if (len(runTimeList) != 0):
        runTimeArray = np.array(runTimeList)
        navDistanceArray = np.array(navDistanceList)
        locomotionErrorArray = np.array(locomotionErrorList)
        print('Maze 1 Mean Navigation Time (s): ', np.average(runTimeArray))
        print('Maze 1 Navigation Time Standard Deviation: ', np.std(runTimeArray))
        print('Maze 1 Mean Navigation Distance (m): ', np.average(navDistanceArray))
        print('Maze 1 Navigation Distance Standard Deviation: ', np.std(navDistanceArray))
        print('Maze 1 Mean Locomotion Error (m): ', np.average(locomotionErrorArray))
        print('Maze 1 Locomotion Error Standard Deviation: ', np.std(locomotionErrorArray))
    else:
        print('All runs failed for this maze')

    runTimeList = []
    navDistanceList = []
    locomotionErrorList = []
    startXValues = []
    endXValues = []
    for i in range(runsPerMaze):
        totalSims += 1

        # Generate random start and end points along the x-axis
        startPointX = random.uniform(maze2StartPoints[0][0], maze2StartPoints[2][0])
        endPointX = random.uniform(maze2EndPoints[0][0], maze2EndPoints[2][0])

        startPoint = np.array([startPointX, maze2StartPoints[0][1]])
        endPoint = np.array([endPointX, maze2EndPoints[0][1]])

        startXValues.append(startPointX)
        endXValues.append(endPointX)

        # Generate random orientation
        yaw = random.uniform(0, 2*math.pi)

        rospy.set_param('endPoint', endPoint.tolist())

        # Set full ground truth path
        #gtPathWPs = np.concatenate((np.concatenate((startPoint, maze3gtPathWPs), axis=0), endPoint), axis=0)
        gtPathWPs = np.insert(maze1gtPathWPs, 0, startPoint, axis=0)
        gtPathWPs = np.insert(gtPathWPs, len(gtPathWPs), endPoint, axis=0)
        rospy.set_param('gtPathWPs', gtPathWPs.tolist())

        x_pos_arg = 'x_pos:=' + str(startPoint[0])
        y_pos_arg = 'y_pos:=' + str(startPoint[1])
        yaw_arg = 'yaw:=' + str(yaw)
        cli_args = ['/home/kevin/Workspaces/ENSE623/src/mazerunner/launch/maze2.launch', x_pos_arg, y_pos_arg, yaw_arg]
        
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
            runTimeList.append(rospy.get_param('runTime'))
            navDistanceList.append(rospy.get_param('navDistance'))
            locomotionErrorList.append(rospy.get_param('meanLocomotionError'))

        roslaunch_parent.shutdown()

        rospy.sleep(5)

    if (len(runTimeList) != 0):
        print('Maze 2 Mean Navigation Time (s): ', sum(runTimeList)/len(runTimeList))
        print('Maze 2 Mean Navigation Distance (m): ', sum(navDistanceList)/len(navDistanceList))
        print('Maze 2 Mean Locomotion Error (m): ', sum(locomotionErrorList)/len(locomotionErrorList))
    else:
        print('All runs failed for this maze')

    runTimeList = []
    navDistanceList = []
    locomotionErrorList = []
    startXValues = []
    endXValues = []
    for i in range(runsPerMaze):
        totalSims += 1

        # Generate random start and end points along the x-axis
        startPointX = random.uniform(maze3StartPoints[0][0], maze3StartPoints[2][0])
        endPointX = random.uniform(maze3EndPoints[0][0], maze3EndPoints[2][0])

        startPoint = np.array([startPointX, maze3StartPoints[0][1]])
        endPoint = np.array([endPointX, maze3EndPoints[0][1]])

        startXValues.append(startPointX)
        endXValues.append(endPointX)

        # Generate random orientation
        yaw = random.uniform(0, 2*math.pi)

        rospy.set_param('endPoint', endPoint.tolist())

        # Set full ground truth path
        #gtPathWPs = np.concatenate((np.concatenate((startPoint, maze3gtPathWPs), axis=0), endPoint), axis=0)
        gtPathWPs = np.insert(maze1gtPathWPs, 0, startPoint, axis=0)
        gtPathWPs = np.insert(gtPathWPs, len(gtPathWPs), endPoint, axis=0)
        rospy.set_param('gtPathWPs', gtPathWPs.tolist())

        x_pos_arg = 'x_pos:=' + str(startPoint[0])
        y_pos_arg = 'y_pos:=' + str(startPoint[1])
        yaw_arg = 'yaw:=' + str(yaw)
        cli_args = ['/home/kevin/Workspaces/ENSE623/src/mazerunner/launch/maze3.launch', x_pos_arg, y_pos_arg, yaw_arg]
        
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
            runTimeList.append(rospy.get_param('runTime'))
            navDistanceList.append(rospy.get_param('navDistance'))
            locomotionErrorList.append(rospy.get_param('meanLocomotionError'))

        roslaunch_parent.shutdown()

        rospy.sleep(5)

    if (len(runTimeList) != 0):
        print('Maze 3 Mean Navigation Time (s): ', sum(runTimeList)/len(runTimeList))
        print('Maze 3 Mean Navigation Distance (m): ', sum(navDistanceList)/len(navDistanceList))
        print('Maze 3 Mean Locomotion Error (m): ', sum(locomotionErrorList)/len(locomotionErrorList))
    else:
        print('All runs failed for this maze')

    print('Success Rate: ', float(successSims) / totalSims)


