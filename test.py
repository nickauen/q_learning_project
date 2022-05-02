#!/usr/bin/env python3

# rosrun q_learning_project test.py blue 2

import sys
import rospy
import time
from geometry_msgs.msg import Twist, Point, PointStamped
from std_msgs.msg import Header
from q_learning_project.msg import RobotMoveObjectToTag
from q_learning_project.msg import QLearningReward

class SendVelocities(object):
    def __init__(self):
        rospy.init_node('send_velocities')
        self.action_pub = rospy.Publisher('/q_learning/robot_action', RobotMoveObjectToTag, queue_size=10)
        self.reward_sub = rospy.Subscriber('/q_learning/reward', QLearningReward, self.print_reward)

    def print_reward(self, data):
        print(data)

    def run(self):
        # Give publisher time to set up
        time.sleep(1)

        msg = RobotMoveObjectToTag()
        msg.robot_object = sys.argv[1]
        msg.tag_id = int(sys.argv[2])

        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.action_pub.publish(msg)
            r.sleep()

if __name__ == '__main__':
    node = SendVelocities()
    node.run()
