#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np
from scipy.spatial.transform import Rotation as R

class PIDController(object):
    def __init__(self, p_gain, i_gain, d_gain):
        self._p_gain = p_gain
        self._i_gain = i_gain
        self._d_gain = d_gain

        self._error_1 = 0.0
        self._error_2 = 0.0
        self._output = 0.0

    def update(self, current, target):
        error = target - current

        delta_output = self._p_gain * (error - self._error_1) # 比例
        delta_output += self._i_gain * (error) # 積分
        delta_output += self._d_gain * (error - 2*self._error_1 + self._error_2) # 微分

        self._output += delta_output

        self._error_2 = self._error_1
        self._error_1 = error

        return self._output


class DirectionController(object):
    def __init__(self):
        self._imu_data_raw = Imu()

        # for angle control
        self._omega_pid_controller = PIDController(3.3, 0.004, 6.0)
        self._target_angle = 0.0

        # for heading_angle calculation
        self.ps_msg = PoseStamped()
        self._heading_angle = 0.0

        rospy.Subscriber("imu", Imu, self._imu_callback)
        self.cmdvel_pub = rospy.Publisher("cmdvel_from_imu", Twist, queue_size=10)
    
    def _quaternion_to_euler_zyx(self, q):
        r = R.from_quat([q.x, q.y, q.z, q.w])
        return r.as_euler('xyz', degrees=True)
    
    
    def _imu_callback(self, imu_msg):
        self._imu_data_raw = imu_msg
        self._heading_angle  = self._quaternion_to_euler_zyx(self._imu_data_raw.orientation)[2]
        print("角度", self._heading_angle)

    def _angle_control(self, target_angle=0.0):
        SIGN = -1.0

        cmdvel = Twist()
        cmdvel.angular.z = SIGN * self._omega_pid_controller.update(target_angle, self._heading_angle)
        self.cmdvel_pub.publish(cmdvel)
    
    def _keep_zero_radian(self):
        self._angle_control(0.0)
    
    def update(self):
        self._keep_zero_radian()

def main():
    rospy.init_node("direction_control")   
    controller = DirectionController()

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        controller.update()
        r.sleep()

if __name__ == "__main__":
    main()