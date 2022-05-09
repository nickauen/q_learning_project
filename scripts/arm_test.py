#!/usr/bin/env python3

import math
import random
import time

import moveit_commander
import numpy as np
import rospy

class Robot(object):
    def __init__(self):
        rospy.init_node('turtlebot3_arm_test')

        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

    def do(self, state):
        arm_joint_goal = None
        gripper_joint_goal = None
        if state == 0:
            print('Travel mode')
            arm_joint_goal = [0.0, -1.0, 1.0, 0.0]
            gripper_joint_goal = [0.017, 0.017]
        elif state == 1:
            print('Prepare to grip')
            arm_joint_goal = [0.0, 0.3, -0.3, 0.0]
        elif state == 2:
            print('Grip object')
            gripper_joint_goal = [-0.01, -0.01]
        elif state == 3:
            print('Lift object')
            arm_joint_goal = [0.0, -1.0, -0.3, 0.0]
        elif state == 4:
            print('Set object')
            arm_joint_goal = [0.0, 0.3, -0.3, 0.0]
        elif state == 5:
            print('Release object')
            gripper_joint_goal = [0.017, 0.017]

        if arm_joint_goal is not None:
            print('  moving arm')
            self.move_group_arm.go(arm_joint_goal, wait=True)
            self.move_group_arm.stop()

        if gripper_joint_goal is not None:
            print('  moving gripper')
            self.move_group_gripper.go(gripper_joint_goal, wait=True)
            self.move_group_gripper.stop()

        print('  sleeping')
        time.sleep(1)

        print('  done')

    def run(self):
        time.sleep(1)
        self.move_group_arm.go([0, 0, 0, 0], wait=True)
        self.move_group_gripper.go([0, 0], wait=True)
        time.sleep(1)

        rate = rospy.Rate(0.15)
        state = 0
        while (not rospy.is_shutdown()):
            self.do(state)
            state = (state+1) % 6
            rate.sleep()

if __name__ == '__main__':
    robot = Robot()
    robot.run()
