#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class Binary_Line:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("binary_line_node")
        self.pub = rospy.Publisher("/binary/compressed", CompressedImage, queue_size=10)
        rospy.Subscriber(
            "/image_jpeg/compressed", CompressedImage, self.img_CB
        )

    def detect_color(self, img):
        # Convert to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define range of yellow color in HSV
        # yellow_lower = np.array([20, 100, 100])
        # yellow_upper = np.array([30, 255, 255])
        yellow_lower = np.array([18, 100, 100])
        yellow_upper = np.array([32, 255, 255])

        # Threshold the HSV image to get only yellow colors
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

        # Threshold the HSV image to get blend colors
        yellow_color = cv2.bitwise_and(img, img, mask=yellow_mask)
        return yellow_color

    def img_warp(self, img):
        self.img_x, self.img_y = img.shape[1], img.shape[0]
        # print(f'self.img_x:{self.img_x}, self.img_y:{self.img_y}')

        img_size = [640, 480]
        # ROI
        src_side_offset = [0, 240]
        src_center_offset = [250, 1]
        src = np.float32(
            [
                [0, 479],
                [src_center_offset[0], src_center_offset[1]],
                [640 - src_center_offset[0], src_center_offset[1]],
                [639, 479],
            ]
        )
        # 아래 2 개 점 기준으로 dst 영역을 설정합니다.
        dst_offset = [round(self.img_x * 0.125), 0]
        # offset x 값이 작아질 수록 dst box width 증가합니다.
        dst = np.float32(
            [
                [dst_offset[0], self.img_y],
                [dst_offset[0], 0],
                [self.img_x - dst_offset[0], 0],
                [self.img_x - dst_offset[0], self.img_y],
            ]
        )
        # find perspective matrix
        matrix = cv2.getPerspectiveTransform(src, dst)
        matrix_inv = cv2.getPerspectiveTransform(dst, src)
        warp_img = cv2.warpPerspective(img, matrix, (self.img_x, self.img_y))
        return warp_img

    def img_binary(self, blend_line):
        bin = cv2.cvtColor(blend_line, cv2.COLOR_BGR2GRAY)
        binary_line = np.zeros_like(bin)
        binary_line[bin >127] = 255
        return binary_line

    def img_CB(self, data):
        img = self.bridge.compressed_imgmsg_to_cv2(data)
        warp_img = self.img_warp(img)
        yellow_img = self.detect_color(warp_img)

        dst = cv2.GaussianBlur(yellow_img, (0,0), 1)

        binary_img = self.img_binary(dst)
        binary_line_img_msg = self.bridge.cv2_to_compressed_imgmsg(binary_img)
        
        self.pub.publish(binary_line_img_msg)

        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        cv2.namedWindow("binary_img", cv2.WINDOW_NORMAL)

        cv2.imshow("img", img)
        cv2.imshow("binary_img", binary_img)

        cv2.waitKey(1)


if __name__ == "__main__":
    binary_lines = Binary_Line()
    rospy.spin()