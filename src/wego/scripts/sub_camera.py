#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class Yellow_Line_Detect:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("yellow_line_node")
        self.pub = rospy.Publisher("/yellow/compressed", CompressedImage, queue_size=10)

        self.image_sub = rospy.Subscriber("/segmantic_image_jpeg/compressed", CompressedImage, self.img_CB)
    def detect_color(self, img):
        # Convert to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define range of yellow color in HSV
        yellow_lower = np.array([18, 100, 100])
        yellow_upper = np.array([32, 255, 255])

        # Threshold the HSV image to get only yellow colors
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        yellow_color = cv2.bitwise_and(img, img, mask=yellow_mask)
        return yellow_color

    def img_CB(self, data):
        img = self.bridge.compressed_imgmsg_to_cv2(data)
        yellow_line = self.detect_color(img)
        yellow_line_img_msg = self.bridge.cv2_to_compressed_imgmsg(yellow_line)
        self.pub.publish(yellow_line_img_msg)
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        cv2.namedWindow("yellow_line", cv2.WINDOW_NORMAL)
        cv2.imshow("img", img)
        cv2.imshow("yellow_line", yellow_line)
        cv2.waitKey(1)


if __name__ == "__main__":
    yellow_line_detect = Yellow_Line_Detect()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass