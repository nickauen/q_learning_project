#!/usr/bin/env python3

# roscore
# roslaunch q_learning_project turtlebot3_intro_robo_manipulation.launch
# rosrun q_learning_project phantom_robot_movement.py
# rosrun q_learning_project q_learning.py

import time
import rospy
import numpy as np
import os
from random import choices

from q_learning_project.msg import QLearningReward, RobotMoveObjectToTag

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))

        #### Project modifications
        # Publishers and subscribers
        self.action_pub = rospy.Publisher('/q_learning/robot_action', RobotMoveObjectToTag, queue_size=10)
        self.matrix_fname = os.path.expanduser('~/catkin_ws/src/q_learning_project/q_matrix.csv')

        # Use:  q_value = self.q_matrix[state][action]
        self.q_matrix = self.load_q_matrix()

        # Use:  new_state = self.possible_actions[state][action]
        self.possible_actions = [{int(act): new_state for new_state, act in enumerate(state)} for state in self.action_matrix]

    def run(self):
        # Give publisher time to set up
        time.sleep(1)

        current_state = 0
        new_state = 0
        for _ in range(3):
            # Get all possible actions in this state
            actions = []
            q_values = []
            for act in self.possible_actions[current_state].keys():
                if act != -1:
                    actions.append(act)
                    q_value = self.q_matrix[current_state][act]
                    q_values.append(q_value)

            # Weighted random selection from the actions
            action = choices(actions, q_values)[0]

            # Use the action to get the next state
            new_state = self.possible_actions[current_state][action]

            # Publish that we took the acton
            msg = RobotMoveObjectToTag()
            msg.robot_object = self.actions[action]['object']
            msg.tag_id = self.actions[action]['tag']

            # Wait until a reward is given
            self.action_pub.publish(msg)
            rospy.Rate(1).sleep()

            current_state = new_state

        time.sleep(10)

    def load_q_matrix(self):
        return np.loadtxt(self.matrix_fname, delimiter=',')

if __name__ == "__main__":
    node = QLearning()
    node.run()
