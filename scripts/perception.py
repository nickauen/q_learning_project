#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # initalize the debugging window
        cv2.namedWindow("window", 1)

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
        Image, self.image_callback)

        # TODO: set up cmd_vel publisher
        #self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #self.twist=Twist()

    def image_callback(self, msg):
        # COLOR RECOGNITION

        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


        blue = numpy.uint8([[[72,160,191 ]]])
        #hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)

        lower_blue = numpy.array([40, 100, 100])
        upper_blue = numpy.array([100, 255, 255])


        green = numpy.uint8([[[167,198,61 ]]])
        #hsv_blue = cv2.cvtColor(blue,cv2.COLOR_BGR2HSV)

        lower_green = numpy.array([5, 100, 100])
        upper_green = numpy.array([40, 255, 255])


        pink = numpy.uint8([[[203,43,129 ]]])
        #hsv_pink = cv2.cvtColor(pink,cv2.COLOR_BGR2HSV)

        lower_pink = numpy.array([100, 100, 100])
        upper_pink = numpy.array([180, 255, 255])


        # this erases all pixels that aren't blue, green, or pink
        mask = (cv2.inRange(hsv, lower_blue, upper_blue) + cv2.inRange(hsv, lower_green, upper_green) + cv2.inRange(hsv, lower_pink, upper_pink))

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

        # shows the debugging window
        # hint: you might want to disable this once you're able to get a red circle in the debugging window

        #cv2.imshow("window", image)
        #cv2.imshow("mask", mask)
        #cv2.waitKey(3)

        # AR TAG DETECTION
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

        for i in corners:
            #extract corners
            corners = i.reshape((4, 2))
            (tLeftCorner, tRightCorner, bRightCorner, bLeftCorner) = corners

            #convert to integers
            tRightCorner = (int(tRightCorner[0]), int(tRightCorner[1]))
            bRightCorner = (int(bRightCorner[0]), int(bRightCorner[1]))
            bLeftCorner = (int(bLeftCorner[0]), int(bLeftCorner[1]))
            tLeftCorner = (int(tLeftCorner[0]), int(tLeftCorner[1]))

            cv2.rectangle(image,(tLeftCorner),(bRightCorner),(255,0,0),2)

        # ids is a 2D array array of shape (n, 1)
        # each entry is the id of a detected tag in the same order as in corners

        # rejected_points contains points from detected tags that don't have codes matching the dictionary

        cv2.imshow('img',image)
        cv2.waitKey(3)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('line_follower')
    follower = Follower()
    follower.run()
