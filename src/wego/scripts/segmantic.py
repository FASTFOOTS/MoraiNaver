#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class Segmantic_detection:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("segmantic")
        self.pub = rospy.Publisher("/segmatic/compressed", CompressedImage, queue_size=10)
        rospy.Subscriber(
            "/segmantic_image_jpeg/compressed", CompressedImage, self.img_CB
        )

    def detect_color(self, img):
        # Convert to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # In Morai, segmatic real color in HSV
        # traffic_light : np.array([255, 74, 240])
        # load_sign : np.array([204, 127, 51])
        
        # Define range of traffic_light color in HSV
        traffic_light_lower = np.array([250, 70, 235])
        traffic_light_upper = np.array([260, 80, 245])

        # Define range of road_sign color in HSV
        load_sign_lower = np.array([200, 125, 45])
        load_sign_upper = np.array([210, 135, 55])

        # Threshold the HSV image to get only traffic_light colors
        traffic_light_mask = cv2.inRange(hsv, traffic_light_lower, traffic_light_upper)
        
        # Threshold the HSV image to get only road_sign colors
        load_sign_mask = cv2.inRange(hsv, load_sign_lower, load_sign_upper)

        # Threshold the HSV image to get blended segmantic colors
        segmantic_mask = cv2.bitwise_or(traffic_light_mask, load_sign_mask)
        # cv2.imshow("segmantic_mask", segmantic_mask)
        segmantic_color = cv2.bitwise_and(img, img, mask=segmantic_mask)
        # cv2.imshow("segmantic_color", segmantic_color)
        return segmantic_color

    def img_CB(self, data):
        img = self.bridge.compressed_imgmsg_to_cv2(data)
        segmantic = self.detect_color(img)
        segmantic_msg = self.bridge.cv2_to_compressed_imgmsg(segmantic)
        self.pub.publish(segmantic_msg)
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        cv2.namedWindow("segmantic_color", cv2.WINDOW_NORMAL)
        cv2.imshow("img", img)
        cv2.imshow("segmantic_color", segmantic_color)
        cv2.waitKey(1)


if __name__ == "__main__":
    segmantic_detect = Segmantic_detection()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass