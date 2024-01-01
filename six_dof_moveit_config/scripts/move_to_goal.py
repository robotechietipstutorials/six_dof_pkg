#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from math import pi 

class MoveToGoal(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_to_goal', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        self.robot = robot
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.eef_link = "link6"

    def go_to_joint_state(self, joint_goal):
        group = self.group
        group.go(joint_goal, wait=True)
        group.stop()
        current_joints = self.group.get_current_joint_values()
        print("go to joint states completed")


    def moveto_xyzrpy(self, xyzrpy):
        self.group.set_pose_target(xyzrpy, end_effector_link=self.eef_link)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        current_pose = self.group.get_current_pose().pose
        print("go to pose completed")

def main():
    try:
        armDriver = MoveToGoal()

        xyz = [0.593, 0.100, 0.189]
        rpy = [1.57, 1.57, 1.57]
        xyzrpy = xyz + rpy
        #move to random pose
        armDriver.moveto_xyzrpy(xyzrpy)
        joint_goal = [pi/4, 0, 0, 0, 0, 0]
        #move to home pose
        armDriver.go_to_joint_state(joint_goal)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()

