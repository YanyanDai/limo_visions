#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2021 PS-Micro, Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
#

import time

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from math import *
from geometry_msgs.msg import Pose

HUE_LOW = 170
HUE_HIGH = 180
SATURATION_LOW = 170
SATURATION_HIGH = 190
VALUE_LOW = 70
VALUE_HIGH = 100


class ImageConverter:
    def __init__(self):
        # 이미지 캐시 관련 변수를 생성
        self.cv_image = None
        self.get_image = False

        # cv_bridge 를 생성
        self.bridge = CvBridge()

        # 이미지 게시자 및 구독자 선언
        self.image_pub = rospy.Publisher("object_detect_image",
                                         Image,
                                         queue_size=1)
        self.target_pub = rospy.Publisher("object_detect_pose",
                                          Pose,
                                          queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image,
                                          self.callback,
                                          queue_size=1)

    def callback(self, data):
        # 현재 이미지가 처리되었는지 확인
        # Determine whether the current image has been processed
        if not self.get_image:
            # cv_bridge를 사용하여 ROS 이미지 데이터를 OpenCV 이미지 형식으로 변환
            # Use cv_bridge to convert ROS image data to OpenCV image format
            try:
                self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print e
            #이미지가 수신되었음을 나타내는 플래그를 설정합니다.
            # Sets the flag indicating that an image was received
            self.get_image = True

    def detect_object(self):
        # Create HSV Threshold List
        boundaries = [([HUE_LOW, SATURATION_LOW,
                        VALUE_LOW], [HUE_HIGH, SATURATION_HIGH, VALUE_HIGH])]

        # Iterate through the list of HSV thresholds
        for (lower, upper) in boundaries:
            # Create an array of threshold values for the upper and lower limits of HSV
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")

        # Gaussian filtering, 이미지 주변의 픽셀 평활화
        # Gaussian filtering, smoothing pixels in the image neighborhood
        hsv_image = cv2.GaussianBlur(self.cv_image, (5, 5), 0)

        # 색 공간 변환, RGB 이미지를 HSV 이미지로 변환
        # Color space conversion, convert RGB image to HSV image
        hsv_image = cv2.cvtColor(hsv_image, cv2.COLOR_BGR2HSV)
        
        # 임계값에 따라 배경을 제거합니다.
        # According to the threshold, remove the background.
        mask = cv2.inRange(hsv_image, lower, upper)
        output = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)

        # 컬러 이미지를 회색 이미지로 변환
        # Convert a color image to a grayscale image
        cvImg = cv2.cvtColor(output, 6)  # cv2.COLOR_BGR2GRAY
        npImg = np.asarray(cvImg)
        thresh = cv2.threshold(npImg, 1, 255, cv2.THRESH_BINARY)[1] # 이진화

        # 대상 물체의  윤곽감지
        # Detect the contours of the target object
        cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST,
                                                cv2.CHAIN_APPROX_NONE)

        # 찾은 모든 윤곽을 반복합니다.
        # Iterate over all found contours
        for c in cnts:
            # 작은 노이즈를 제거합니다
            if c.shape[0] < 150:
                continue

            # 윤곽의 특징 추출
            M = cv2.moments(c)

            if int(M["m00"]) not in range(500, 22500):
                continue

            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            print("x: {}, y: {}, size: {}".format(cX, cY, M["m00"]))

            # 외곽선을 그리고 중심점을 그립니다.
            cv2.drawContours(self.cv_image, [c], -1, (0, 0, 255), 2)
            cv2.circle(self.cv_image, (cX, cY), 1, (0, 0, 255), -1)
            
            # topic를 통해 대상 위치 게시
            objPose = Pose()
            objPose.position.x = cX
            objPose.position.y = cY
            objPose.position.z = M["m00"] # 면적 area
            self.target_pub.publish(objPose)

        # opencv 형식의 데이터를 ros 이미지 형식의 데이터로 변환, 다음에 publish
        # convert the opencv format data into ros image format data, then publish
        try:
            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
        except CvBridgeError as e:
            print e

    def loop(self):
        if self.get_image:
            self.detect_object()
            self.get_image = False


if __name__ == '__main__':
    try:
        # Initialize ros node
        rospy.init_node("object_detect")
        rospy.loginfo("Starting detect object")
        # create image_converter object based on class ImageConverter
        image_converter = ImageConverter()
        rate = rospy.Rate(100) # 100 Hz
        while not rospy.is_shutdown():
            # call loop function
            image_converter.loop()
            rate.sleep()
    except KeyboardInterrupt:
        print "Shutting down object_detect node."
        cv2.destroyAllWindows()
