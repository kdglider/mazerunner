#!/usr/bin/env python
# Required for ROS to execute file as a Python script

import rospy
import roslaunch


# Deterministic mode start/end points
maze1StartPoints = [[-1.5, -2.5],
                    [0.5, -2.5],
                    [2.5, -2.5]]

maze2StartPoints = [[-2, -2.5],
                    [0, -2.5],
                    [2, -2.5]]

maze3StartPoints = [[-1.5, -2.5],
                    [0.5, -2.5],
                    [2.5, -2.5]]

maze1EndPoints = [[-1.5, 2.4],
                  [0.5, 2.4],
                  [2.5, 2.4]]

maze2EndPoints = [[-2, 2.5],
                  [-0.5, 2.5],
                  [2, 2.5]]

maze3EndPoints = [[-1.5, -2.4],
                  [0.5, -2.4],
                  [2.5, -2.4]]



'''
#package = 'mazerunner'
#executable = 'main.py'
package = 'rqt_gui'
executable = 'rqt_gui'
node = roslaunch.core.Node(package, executable)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
print(process.is_alive())
process.stop()
'''


cli_args = ['/home/kevin/Workspaces/ENSE623/src/mazerunner/launch/maze1.launch','x_pos:=0.5', 'y_pos:=0']
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch_parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

roslaunch_parent.start()




#test = rospy.get_param('test')
#print(test)

package = 'mazerunner'
executable = 'main.py'
node = roslaunch.core.Node(package, executable, args='42')

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()
print('BREAK1')
process = launch.launch(node)
rospy.sleep(5)
print('BREAK3')
print(process.is_alive())
process.stop()


rospy.sleep(15)
roslaunch_parent.shutdown()

'''
rospy.sleep(5)

roslaunch_parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

roslaunch_parent.start()

rospy.sleep(10)
roslaunch_parent.shutdown()
'''