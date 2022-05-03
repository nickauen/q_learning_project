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
        # e.g. [[0, 0, 0], [1, 0, 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        #### Project modifications
        # Publishers and subscribers
        self.action_pub = rospy.Publisher('/q_learning/robot_action', RobotMoveObjectToTag, queue_size=10)
        self.reward_sub = rospy.Subscriber('/q_learning/reward', QLearningReward, self.reward)
        # Reward stuff
        self.reward_value = 0
        self.got_reward = False
        # Q matrix stuff
        self.discount_factor = 0.8
        self.matrix_fname = os.path.expanduser('~/catkin_ws/src/q_learning_project/q_matrix.csv')

        # Use:  q_value = self.q_matrix[state][action]
        self.q_matrix = np.zeros((64, 9))

        # Use:  new_state = self.possible_actions[state][action]
        self.possible_actions = [{int(act): new_state for new_state, act in enumerate(state)} for state in self.action_matrix]

    def run(self):
        # Give publisher time to set up
        time.sleep(1)

        while not rospy.is_shutdown():
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
                        # This +10 allows the random selection just below to have
                        # a chance to 'explore', rather than always 'exploit'
                        q_values.append(q_value+10)

                # Weighted random selection from the actions
                action = choices(actions, q_values)[0]

                # Use the action to get the next state
                new_state = self.possible_actions[current_state][action]

                # Publish that we took the acton
                msg = RobotMoveObjectToTag()
                msg.robot_object = self.actions[action]['object']
                msg.tag_id = self.actions[action]['tag']

                # Wait until a reward is given
                self.got_reward = False
                self.action_pub.publish(msg)
                while not self.got_reward:
                    rospy.Rate(5).sleep()

                # If we got a positive reward, then the next state is a 'winning' state.
                # This needs to be remembered so that future actions that lead to this
                # state will have this equation calculated correctly...
                #         np.max(self.q_matrix[new_state])
                if self.reward_value == 100:
                    self.q_matrix[new_state] = 100

                # Update q_matrix with reward and state
                old_value = self.q_matrix[current_state][action]
                new_value = self.reward_value + self.discount_factor * np.max(self.q_matrix[new_state])
                self.q_matrix[current_state][action] = new_value

                # Save the new q matrix
                if old_value != new_value:
                    print('----------------')
                    print(np.round(self.q_matrix, 0))
                    self.save_q_matrix()

                current_state = new_state

    def reward(self, data):
        self.reward_value = data.reward
        self.got_reward = True

    def save_q_matrix(self):
        np.savetxt(self.matrix_fname, self.q_matrix, delimiter=',')

if __name__ == "__main__":
    node = QLearning()
    node.run()
