#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosparam
from std_msgs import msg
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import math
import time
import message_filters
import tf


class fingertip_detector:
    def __init__(self):
        self.image_pub = rospy.Publisher("/frame", Image, queue_size=
        1)
        self.bridge = CvBridge()
        self.image_topic = rospy.get_param(
            "~image_topic", "/kinect2/qhd/image_color_rect")
        self.depth_image_topic = rospy.get_param(
            "~depth_image_topic", "/kinect2/qhd/image_depth_rect")
        print self.image_topic, self.depth_image_topic
        self.image_sub = message_filters.Subscriber(self.image_topic, Image)
        self.depth_sub = message_filters.Subscriber(
            self.depth_image_topic, Image)
        self.both = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub], 10, 0.5)
        self.both.registerCallback(self.callback)
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(30.0)

    def callback(self, rgb, depth):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth, "16UC1")
            cv_depth_image = np.array(depth_image, dtype=np.float)
        except CvBridgeError as e:
            print e
        mask, frame, ft_point = self.detectFingertip(cv_image)
        cv_depth_image = cv2.flip(cv_depth_image, -1)
        p = self.getPositon(ft_point, cv_depth_image)
        print ft_point, p
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        self.rate.sleep()

    def detectFingertip(self, frame):
        frame = cv2.flip(frame, -1)
        topLeft = [50, 300]
        btmRight = [450, 800]
        roi = frame[topLeft[0]:btmRight[0], topLeft[1]:btmRight[1]]
        cv2.rectangle(frame, (topLeft[1], topLeft[0]),
                      (btmRight[1], btmRight[0]), (0, 255, 0), 3)
        kernel = np.ones((3, 3), np.uint8)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # define range of skin color in HSV
        lower_skin = np.array([0, 20, 70], dtype=np.uint8)
        upper_skin = np.array([20, 255, 255], dtype=np.uint8)
        # extract skin color image
        mask = cv2.inRange(hsv, lower_skin, upper_skin)
        # find contours
        _, contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # find contour of max area(hand)
        try:
            cnt = max(contours, key=lambda x: cv2.contourArea(x))
        except ValueError as e:
            return(mask, frame, [])
        # approx the contour a little
        epsilon = 0.0005*cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        cv2.drawContours(roi, approx, -1, (0, 0, 255), 2)
        ft_point = min(approx, key=(lambda x: x[0][1]))[0]+[300, 50]
        cv2.circle(frame, tuple(ft_point), 5, [0, 255, 255], 5)
        return(mask, frame, ft_point)

    def getPositon(self, ft_point, depth_frame):
        if(len(ft_point) == 0):
            return []
        cx = 478.72932820862985
        cy = 279.96325294928386
        fx = 543.256550023704
        fy = 543.817235985575
        depthScale = 1000
        depth = depth_frame[ft_point[1], ft_point[0]]
        y_shift = -0.02 # mm
        if (depth != 0 and depth != None):
            z = depth/depthScale
            x = -(ft_point[0]-cx)*z/fx
            y = -(ft_point[1]-cy)*z/fy+y_shift
            self.br.sendTransform((x, y, z), (0, 0, 0, 1),
                                  rospy.Time.now(), "tool", "kinect2_link")
            return [x, y, z]


if __name__ == '__main__':
    try:
        rospy.init_node("simple_hgr_node", anonymous=True,
                        disable_signals=True)
        rospy.loginfo("Starting simple_hgr node")
        fingertip_detector()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down simple_hgr node."
        cv2.destroyAllWindows()
