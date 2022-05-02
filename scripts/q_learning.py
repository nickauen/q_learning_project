#!/usr/bin/env python3

# roscore
# roslaunch q_learning_project turtlebot3_intro_robo_manipulation.launch
# rosrun q_learning_project phantom_robot_movement.py
# rosrun q_learning_project q_learning.py

import time
import rospy
import numpy as np
import os

from q_learning_project.msg import QLearningReward, RobotMoveObjectToTag

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))

        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        # self.current_state = 0
        # self.q_matrix = np.zeros((64, 9))

        # self.possible_actions = []
        # for state in self.action_matrix:
        #     self.possible_actions.append({})
        #     for new_state, act in enumerate(state):
        #         if act != -1:
        #             self.possible_actions[-1][act] = new_state
        pass

    def run(self):
        # for _ in range(3):
        #     action = np.argmax(self.q_matrix[self.current_state])
        #     new_state = self.possible_actions[self.current_state][action]
        #     msg = self.actions[action]
        #     publish(msg)
        pass

    def reward(self, data):
        # update q matrix with reward and state
        # self.current_state = 0
        # reset environment ??
        pass

    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        return

if __name__ == "__main__":
    node = QLearning()
