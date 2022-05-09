#!/usr/bin/env python3

import math
import random
import time
import cv2
import cv_bridge
import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from q_learning_project.msg import RobotMoveObjectToTag

class Robot(object):
    def __init__(self):
        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # subscribe to the robot's RGB camera data stream
        # TODO fix leading '/' here?
        self.distance = 100

        self.color_distance_threshold = 0.25
        self.color_angle_threshold = 15

        self.artag_distance_threshold = 0.4
        self.artag_angle_threshold = 40

        self.move_group_arm = moveit_commander.MoveGroupCommander('arm')
        self.move_group_gripper = moveit_commander.MoveGroupCommander('gripper')

        self.color_cx = None
        self.artag_cx = None

        self.img_cx = None

        self.queue = []
        self.current_task = None

        # 'new task' = has no object, safe for new task
        # 'at object' = at the object, safe to pick up the object
        # 'at object wait' = performing operation to pick up
        # 'has object' = has the object, safe to pick go to the tag
        # 'at tag' = at the tag, safe to set down the object
        # 'at tag wait' = performing operation to set down
        # 'done' = finished task
        self.state = 'new task'

        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        rospy.Subscriber('/q_learning/robot_action', RobotMoveObjectToTag, self.add_action_to_queue)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def process_scan(self, data):
        distances = [*data.ranges[355:], *data.ranges[:5]]
        distances = list(filter(lambda x:x!=0, distances))
        if len(distances):
            self.distance = sum(distances)/len(distances)

    def add_action_to_queue(self, action):
        print('GOT ACTION', (action.robot_object, action.tag_id))
        self.queue.append((action.robot_object, action.tag_id))

    def image_callback(self, msg):
        # converts the incoming ROS message to OpenCV format
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.img_cx = image.shape[1] // 2

        if self.current_task is not None:
            color, tag_id = self.current_task
            self.color_cx = self.color_recog(image, color)
            self.artag_cx = self.artag_recog(image, tag_id)
        else:
            self.color_cx = None
            self.artag_cx = None

        cv2.imshow('img', image)
        cv2.waitKey(1)

    def color_recog(self, image, color_desired):
        # Convert to HSV (hue, saturation, value)
        image = image[:image.shape[0]//2,:,:]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        if color_desired == 'blue':
            lower_blue = np.array([110, 80, 50])
            upper_blue = np.array([170, 130, 90])

            # this erases all pixels that aren't blue
            mask = cv2.inRange(image, lower_blue, upper_blue)
        elif color_desired == 'green':
            lower_green = np.array([30, 80, 70])
            upper_green = np.array([90, 140, 120])

            # this erases all pixels that aren't green
            mask = cv2.inRange(image, lower_green, upper_green)
        else:
            lower_pink = np.array([120, 60, 140])
            upper_pink = np.array([160, 100, 180])

            # this erases all pixels that aren't pink
            mask = cv2.inRange(image, lower_pink, upper_pink)

        mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 7)), iterations=1)

        cv2.imshow('mask', mask)
        cv2.waitKey(1)

        # using moments() function, the center of any colored pixels is determined
        M = cv2.moments(mask)

        # if there are any yellow pixels found
        if M['m00'] > 0:
            # center of the yellow pixels in the image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            # Debug
            # cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            return cx

    def artag_recog(self, image, tag_id_desired):
        # load DICT_4X4_50
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        # turn the image into a grayscale
        grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # search for tags from DICT_4X4_50 in a GRAYSCALE image
        corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, aruco_dict)

        # corners is a 4D array of shape (n, 1, 4, 2), where n is the number of tags detected
        # each entry is a set of four (x, y) pixel coordinates corresponding to the
        # location of a tag's corners in the image
        img_center = -1000

        if corners is None or ids is None:
            return

        for corner, tag_id in zip(corners, ids):
            # extract corners
            (tLeftCorner, tRightCorner, bRightCorner, bLeftCorner) = corner.reshape((4, 2))

            # convert to integers
            tRightCorner = (int(tRightCorner[0]), int(tRightCorner[1]))
            bRightCorner = (int(bRightCorner[0]), int(bRightCorner[1]))
            bLeftCorner = (int(bLeftCorner[0]), int(bLeftCorner[1]))
            tLeftCorner = (int(tLeftCorner[0]), int(tLeftCorner[1]))

            # Debug
            # cv2.rectangle(image, (tLeftCorner), (bRightCorner), (255,0,0), 2)

            if tag_id == tag_id_desired:
                cx = (tLeftCorner[0] + tRightCorner[0]) // 2
                return cx

    def go_to(self, cx, target_type):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = 0.0

        distance_thresh = self.color_distance_threshold if target_type == 'color' else self.artag_distance_threshold
        angle_thresh = self.color_angle_threshold if target_type == 'color' else self.artag_angle_threshold
        speed = 0.04 if target_type == 'color' else 0.1

        good = False

        if cx is None or self.img_cx is None:
            msg.angular.z = -0.25
            print('Turning')
            self.twist_pub.publish(msg)
            return good

        bias = cx - self.img_cx

        msg.angular.z = -bias*0.003
        print('  left/right offset', bias)
        print('  turning speed    ', -bias*0.003)
        print('  target distance  ', self.distance)

        if self.distance != 0 and abs(bias) < angle_thresh:
            if self.distance > distance_thresh:
                msg.linear.x = speed
            if self.distance > 4*distance_thresh:
                msg.linear.x = 2*speed

        if self.distance != 0 and self.distance <= distance_thresh:
            good = True

        self.twist_pub.publish(msg)

        return good

    def execute_grab(self):
        print('Prepare to grip')
        arm_joint_goal = [0.0, 0.3, -0.2, -0.3]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        time.sleep(4)
        print('Grip object')
        gripper_joint_goal = [-0.007, -0.007]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        time.sleep(2)
        print('Lift object')
        arm_joint_goal = [0.0, -1.0, -0.3, 0.0]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        time.sleep(4)

        self.move_group_arm.stop()
        self.move_group_gripper.stop()
        time.sleep(1)

        self.state = 'has object'

    def execute_drop(self):
        print('Set object')
        arm_joint_goal = [0.0, 0.3, -0.3, 0.0]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        time.sleep(4)
        print('Release object')
        gripper_joint_goal = [0.017, 0.017]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        time.sleep(2)
        print('Move away')
        arm_joint_goal = [0.0, -1.0, 1.0, 0.0]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        time.sleep(4)

        self.move_group_arm.stop()
        self.move_group_gripper.stop()
        time.sleep(1)

        self.state = 'done'

    def run(self):
        # Give ROS time to set up
        time.sleep(1)

        arm_joint_goal = [0.0, -1.0, 1.0, 0.0]
        gripper_joint_goal = [0.017, 0.017]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = 0.0
        self.twist_pub.publish(msg)

        rate = rospy.Rate(1)
        while (not rospy.is_shutdown()):
            if len(self.queue) and self.current_task is None:
                self.current_task = self.queue[0]
                self.queue = self.queue[1:]

            if self.current_task is not None:
                print('STATE ----- ', self.state)
                if self.state == 'new task':
                    good = self.go_to(self.color_cx, 'color')
                    if good:
                        self.state = 'at object'
                        msg = Twist()
                        msg.angular.z = 0.0
                        msg.linear.x = 0.0
                        self.twist_pub.publish(msg)
                elif self.state == 'at object':
                    self.state = 'at object wait'
                    self.execute_grab()
                elif self.state == 'at object wait':
                    pass
                elif self.state == 'has object':
                    good = self.go_to(self.artag_cx, 'tag')
                    if good:
                        self.state = 'at tag'
                        msg = Twist()
                        msg.angular.z = 0.0
                        msg.linear.x = 0.0
                        self.twist_pub.publish(msg)
                elif self.state == 'at tag':
                    self.state = 'at tag wait'
                    self.execute_drop()
                elif self.state == 'at tag wait':
                    pass
                elif self.state == 'done':
                    self.current_task = None
                    self.state = 'new task'
                else:
                    assert False

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pipeline')
    robot = Robot()
    robot.run()
