#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Follower:
    def __init__(self):
        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.distance = 100

    def process_scan(self, data):
        self.distance = data.ranges[0]
        print('dist', self.distance)

    def image_callback(self, msg):
        # converts the incoming ROS message to OpenCV format
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # cx = self.color_recog(image, 'blue')
        cx = self.artag_recog(image, 1)
        self.turn_to(image, cx)

        cv2.imshow('img', image)
        cv2.waitKey(1)

    def color_recog(self, image, color_desired):
        # Convert to HSV (hue, saturation, value)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        if color_desired == 'blue':
            # blue = np.uint8([[[72,160,191 ]]])
            # hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)

            lower_blue = np.array([40, 100, 100])
            upper_blue = np.array([100, 255, 255])

            # this erases all pixels that aren't blue
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
        elif color_desired == 'green':
            # green = np.uint8([[[167,198,61 ]]])
            # hsv_blue = cv2.cvtColor(blue,cv2.COLOR_BGR2HSV)

            lower_green = np.array([5, 100, 100])
            upper_green = np.array([40, 255, 255])

            # this erases all pixels that aren't green
            mask = cv2.inRange(hsv, lower_green, upper_green)
        else:
            # pink = np.uint8([[[203,43,129 ]]])
            # hsv_pink = cv2.cvtColor(pink,cv2.COLOR_BGR2HSV)

            lower_pink = np.array([100, 100, 100])
            upper_pink = np.array([180, 255, 255])

            # this erases all pixels that aren't pink
            mask = cv2.inRange(hsv, lower_pink, upper_pink)

        # using moments() function, the center of any colored pixels is determined
        M = cv2.moments(mask)

        # if there are any yellow pixels found
        if M['m00'] > 0:
            # center of the yellow pixels in the image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            # a red circle is visualized in the debugging window to indicate
            # the center point of the yellow pixels
            # hint: if you don't see a red circle, check your bounds for what is considered 'yellow'
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            return cx

    def artag_recog(self, image, ar_id_desired):
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

        for corner, ar_id in zip(corners, ids):
            # extract corners
            (tLeftCorner, tRightCorner, bRightCorner, bLeftCorner) = corner.reshape((4, 2))

            # convert to integers
            tRightCorner = (int(tRightCorner[0]), int(tRightCorner[1]))
            bRightCorner = (int(bRightCorner[0]), int(bRightCorner[1]))
            bLeftCorner = (int(bLeftCorner[0]), int(bLeftCorner[1]))
            tLeftCorner = (int(tLeftCorner[0]), int(tLeftCorner[1]))

            cv2.rectangle(image, (tLeftCorner), (bRightCorner), (255,0,0), 2)

            if ar_id == ar_id_desired:
                cx = (tLeftCorner[0] + tRightCorner[0]) // 2
                return cx

    def turn_to(self, image, cx):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = 0.0

        if cx is None:
            msg.angular.z = -0.15
            print('Turning')
            self.twist_pub.publish(msg)
            return

        img_cx = image.shape[1] // 2
        bias = cx - img_cx

        msg.angular.z = -bias*0.003
        print(bias, '   ', -bias*0.003)

        if abs(bias) < 15 and self.distance > 0.5:
            msg.linear.x = 0.02

        self.twist_pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('line_follower')
    follower = Follower()
    follower.run()
