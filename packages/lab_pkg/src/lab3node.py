#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class edge_finder:
    def __init__(self):
        rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.lanefilter_cb, queue_size=1, buff_size=2**24)
            
        self.pub_white = rospy.Publisher("/image_lines_white", Image, queue_size = 10)
        self.pub_yellow = rospy.Publisher("/image_lines_yellow", Image, queue_size = 10)
        self.bridge = CvBridge()

    def lanefilter_cb(self, msg):
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") 
        image_size = (160, 120)
        offset = 40
        new_image = cv2.resize(cv_image, image_size, interpolation=cv2.INTER_NEAREST)
        cv_cropped = new_image[offset:, :]
     
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        image_hsv = cv2.cvtColor(cv_cropped, cv2.COLOR_BGR2HSV)
        
        image_white = cv2.inRange(image_hsv,(30,0,155),(255,100,255))
        white_erode = cv2.erode(image_white, kernel)
        white_dilate = cv2.dilate(image_white, kernel2)

        image_yellow = cv2.inRange(image_hsv,(25,100,100),(50,255,255))
        yellow_erode = cv2.erode(image_yellow, kernel)
        yellow_dilate = cv2.dilate(image_yellow, kernel2)
        
#begin hw8
        edges = cv2.Canny(cv_cropped, 0, 500)

        yellow_edges = cv2.bitwise_and(edges, image_yellow)
        yellow_lines = cv2.HoughLinesP(yellow_edges, rho = 1, theta = (np.pi/180), threshold = 40, minLineLength = 1, maxLineGap = 8)
        output_yellow = self.output_lines(self.original_image, yellow_lines)
        output_yellow = self.bridge.cv2_to_imgmsg(output_yellow, "bgr8")
        self.pub_yellow.publish(output_yellow)

        white_edges = cv2.bitwise_and(edges, image_white)
        white_lines = cv2.HoughLinesP(white_edges, rho = 1, theta = (np.pi/180), threshold = 40, minLineLength = 1, maxLineGap = 8)
        output_white = self.output_lines(self.original_image, white_lines)
        output_white = self.bridge.cv2_to_imgmsg(output_white, "bgr8")
        self.pub_white.publish(output_white)
    
    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output
