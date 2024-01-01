#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from math import pi 

class GripperControl(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_to_goal', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "gripper"
        group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        self.robot = robot
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher


    def go_to_gripper_joint_state(self, joint_goal):
        group = self.group
        group.go(joint_goal, wait=True)
        group.stop()
        current_joints = self.group.get_current_joint_values()
        print("go to joint states completed")



def main():
    try:
        gripperDriver = GripperControl()
        #open
        gripper_joint_goal =[0.0, 0.0]
        gripperDriver.go_to_gripper_joint_state(gripper_joint_goal)
        #close
        gripper_joint_goal =[0.011, -0.011]
        gripperDriver.go_to_gripper_joint_state(gripper_joint_goal)
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()

