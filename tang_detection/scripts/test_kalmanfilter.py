# -*- coding: utf-8 -*-
#!/usr/bin/env python3

from mymodule import kalmanfilter
from mymodule import red_detection
import numpy as np
import time
from tang_msgs.msg import Command, DetectionResult
import rospy


class TestKalmanFilter():
    def __init__(self):
        rospy.init_node("test_kalmanfilter", anonymous=True)
        rospy.Subscriber("/tang_cmd", Command, self.callback_humanpos, queue_size=1)
        self.result = DetectionResult()
        self.prev_time = 0.0
        self.delta_t = 0.0
        self.human_input = np.array([1.0, 1.0, 0.0, 0.001, 0.0]).T
        self.prev_human_input = np.array([0.0, 0.0, 0.0, 0.001, 0.0]).T

    def callback_humanpos(self, msg):
        rospy.loginfo("callback human position")
        self.delta_t = msg.time - self.prev_time
        self.human_input[0] = msg.position.x
        self.human_input[1] = msg.position.y
        self.human_input[2] = msg.position.z
        self.human_input[3] = (self.human_input[0] -
                               self.prev_human_input[0])/self.delta_t
        self.human_input[4] = (self.human_input[1] -
                               self.prev_human_input[1]) / self.delta_t
        self.prev_human_input = self.human_input
        self.prev_time = msg.time

    def main_loop(self):
        robot_vw = np.array([0.000001, 0.000001])
        kalman = kalmanfilter.KalmanFilter(self.human_input, 0.1)
        rate = rospy.Rate(10)
        detect_pub = rospy.Publisher("/detection_result", DetectionResult, queue_size=1)
        while not rospy.is_shutdown():
            # robot_vwは毎時の値を取得
            # human_inputはカメラから得られた結果、人が同じ位置に居続ける
            # それらをもとに推測した位置と分散を返す
            human_pos_beleif = kalman.main_loop(self.human_input, robot_vw)
            print("HumanPos:x, y", self.human_input[0], self.human_input[1])
            print("BeliefPos:x, y",
                  human_pos_beleif.mean[0], human_pos_beleif.mean[1])
            self.result.obs_pos[0] = self.human_input[0]
            self.result.obs_pos[1] = self.human_input[1]
            self.result.obs_pos[2] = self.human_input[2]
            self.result.est_pos[0] = human_pos_beleif.mean[0]
            self.result.est_pos[1] = human_pos_beleif.mean[1]
            self.result.est_pos[2] = human_pos_beleif.mean[2]
            detect_pub.publish(self.result)
            rate.sleep()


if __name__ == "__main__":
    test_kalmanfilter = TestKalmanFilter()
    test_kalmanfilter.main_loop()
    rospy.spin()
