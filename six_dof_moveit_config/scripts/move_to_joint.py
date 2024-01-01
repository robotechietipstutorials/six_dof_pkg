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
        #move to before pick pose
        joint_goal =[-3.1415, -0.2862, -0.5, 3.14, 1.6613, -0.0142]
        armDriver.go_to_joint_state(joint_goal)
        joint_goal =[-3.1415, -0.9975, -0.497, 3.14, 1.6613, -0.0142]
        #move to pick pose
        armDriver.go_to_joint_state(joint_goal)
        #move to after pick pose
        joint_goal =[-3.1415, -0.2862, -0.5, 3.14, 1.6613, -0.0142]
        armDriver.go_to_joint_state(joint_goal)
        #move to before place pose
        joint_goal =[-0.9546, -0.2862, -0.5, 3.14, 1.6613, -0.0142]
        armDriver.go_to_joint_state(joint_goal)
        #move to place pose
        joint_goal =[-0.9546, -1.0149, -0.5, 3.131, 1.6673, -0.0142]
        armDriver.go_to_joint_state(joint_goal)
        #move to after place pose
        joint_goal =[-0.9546, -0.2862, -0.5, 3.14, 1.6613, -0.0142]
        armDriver.go_to_joint_state(joint_goal)
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()

