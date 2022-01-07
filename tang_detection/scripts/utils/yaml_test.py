#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
import numpy as np
import rosparam
import os
import roslib.packages
import math
import time

def main():
    rospy.init_node('yaml_test', anonymous=True)
    rospy.loginfo("start yaml_test node")
    pkg_name = 'tang_detection'
    list = rosparam.load_file(roslib.packages.get_pkg_dir(pkg_name) + "/config/yaml_test.yaml")[0][0]
    right_point1 = list[0]
    print(len(right_point1))
    cnt = 0
    place_max_cnt = 8
    while(not rospy.is_shutdown()):
        place_point = list[cnt%2][(math.floor(cnt/2))%place_max_cnt]
        cnt += 1
        print(place_point[0], place_point[1])
        # time.sleep(0.01)

if __name__ == '__main__':
    main()