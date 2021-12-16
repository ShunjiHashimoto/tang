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
    video =  "/home/hashimoto/git/tang/tang_detection/figs/dashimaki.png"
    while not rospy.is_shutdown():
        ret, frame = cv2.imread(video) 
        try:
            pos, radius = detection_red.red_detection(video, 1)
            # print(pos, radius)
        except:
            print("error")
            pass
    rospy.spin()

