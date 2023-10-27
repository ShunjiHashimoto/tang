#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 条件
# 入力：
# ydlidar_ros_driverで出力された/scanトピック
# 出力
# 障害物との最近傍距離[m]、tang_controlで停止するか否かは決定する
# msg type： ObstacleDistance.msg

import numpy as np
# ros
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tang_msgs.msg import ObstacleDistance

class ObstacleDistanceCalculator():
    def __init__(self):
        self.obstacle_distance = ObstacleDistance()
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
    
    def scan_callback(self, msg):
        # 0以外の要素が存在するか確認
        non_zero_numbers = [num for num in msg.ranges if num != 0]
        if non_zero_numbers:
            self.obstacle_distance = min([num for num in msg.ranges if num != 0])
        else:
            self.obstacle_distance = 0.0
    
    def calc_obstacle_distance(self):
        rospy.init_node("obstacle_distance_calulator_node", anonymous=True)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            print(f"obstacle distance[m]: {self.obstacle_distance}")
            rate.sleep()
        rospy.spin()

if __name__ == "__main__":
    obstacle_distance_calculator = ObstacleDistanceCalculator()
    obstacle_distance_calculator.calc_obstacle_distance()