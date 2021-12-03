#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import red_detection
import rospy
import roslib.packages
import cv2

pkg_name = 'tang_detection'

if __name__ == "__main__":
    rospy.init_node('red_detection', anonymous=True)
    detection_red = red_detection.DetectRed()
    video = cv2.VideoCapture(roslib.packages.get_pkg_dir(pkg_name) + rospy.get_param("/tang_detection/video_path"))
    while not rospy.is_shutdown():
        ret, frame = video.read() 
        try:
            pos, radius = detection_red.red_detection(frame, ret)
            print(pos, radius)
        except:
            pass
    rospy.spin()

