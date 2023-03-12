#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file human_detection.py
@brief realsenseで背景処理を行い、人物を推定する、また赤色検出を行う
"""

# ros
import rospy
import roslib.packages
from tang_msgs.msg import HumanInfo, Modechange
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point

# detectnet
import jetson_inference
import jetson_utils
import argparse

# Kalmanfilter
from mymodule import kalmanfilter

# realsense
import pyrealsense2 as rs

# others
import sys
import numpy as np
from numpy.lib.function_base import copy
import time
from scipy.spatial.transform import Rotation
import math
import Jetson.GPIO as GPIO
import matplotlib.pyplot as plt

# decimarion_filterのパラメータ
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 1)

# spatial_filterのパラメータ(平滑化)
spatial = rs.spatial_filter()
spatial.set_option(rs.option.filter_magnitude, 1)
spatial.set_option(rs.option.filter_smooth_alpha, 0.25)
spatial.set_option(rs.option.filter_smooth_delta, 50)

# hole_filling_filterのパラメータ
hole_filling = rs.hole_filling_filter()

# disparity
depth_to_disparity = rs.disparity_transform(True)
disparity_to_depth = rs.disparity_transform(False)

WIDTH = 640
HEIGHT = 480
FPS = 60

# Calc Gear Sensor Parameter
def add_right_gear_count(channel):
    cnt_list[0]+=1

def add_left_gear_count(channel):
    cnt_list[1]+=1

def calc_velocity(count, time):
    return Pitch*count/time #[m/s]

Pitch = 6.1850105367549055/2/1000
cnt_list = [0, 0]
GPIO.setmode(GPIO.BCM)
# 割り込みイベント設定
right_gear_pin = 24
left_gear_pin = 22
GPIO.setup(right_gear_pin, GPIO.IN)
GPIO.setup(left_gear_pin, GPIO.IN)
# bouncetimeは割り込みを行った後設定された時間は割り込みを検知しないという糸
GPIO.add_event_detect(right_gear_pin, GPIO.BOTH, bouncetime=1)
GPIO.add_event_detect(left_gear_pin, GPIO.BOTH, bouncetime=1)
# コールバック関数登録
GPIO.add_event_callback(right_gear_pin, add_right_gear_count) 
GPIO.add_event_callback(left_gear_pin, add_left_gear_count) 

class HumanDetector():
    """
    @class HumanDetector
    @brief 人物検出
    """

    def __init__(self):
        rospy.init_node('human_detection', anonymous=True)
        # create video output object
        self.output = jetson_utils.videoOutput("display://0")
        # load the object detection network
        self.net = jetson_inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
        # publisher
        self.cmd_publisher = rospy.Publisher('tang_cmd', HumanInfo, queue_size=1)
        # subscriber
        rospy.Subscriber("current_param", Modechange, self.mode_callback, queue_size=1)
        rospy.Subscriber("imu", Imu, self._imu_callback)
        # command type
        self.human_info = HumanInfo()
        self.current_mode = Modechange()
        self.human_point_pixel = Point()
        self.human_point_pixel.z = 1.0
        self.current_mode.realsense_depth_thresh = rospy.get_param("/tang_detection/threshold")
        self.current_mode.mode = 1
        self.debug = rospy.get_param("/tang_detection/debug")
        # KalmanFileter Parameter
        self.prev_time = 0.0
        self.human_input = np.array([1.0, 0.0, 0.0, 0.0, 0.0]).T
        self.prev_human_input = np.array([1.0, 0.0, 0.0, 0.001, 0.0]).T
        # IMU Parameter
        self._imu_data_raw = Imu()
        self._heading_angle = 0.0
        # graph setting
        self.X_est = []
        self.Y_est = []
        self.X_true = []
        self.Y_true = []
        self.e_list = []
    
    def _quaternion_to_euler_zyx(self, q):
        r = Rotation.from_quat([q.x, q.y, q.z, q.w])
        return r.as_euler('xyz', degrees=True)
    
    def _imu_callback(self, imu_msg):
        self._imu_data_raw = imu_msg
        self._heading_angle  = self._quaternion_to_euler_zyx(self._imu_data_raw.orientation)[2]

    def get_filtered_frame(self, align, frames, max_dist):
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        filter_frame = decimate.process(depth_frame)
        filter_frame = depth_to_disparity.process(filter_frame)
        filter_frame = spatial.process(filter_frame)
        filter_frame = disparity_to_depth.process(filter_frame)
        filter_frame = hole_filling.process(filter_frame)
        result_frame = filter_frame.as_depth_frame()
        # RGB画像
        color_image = np.asanyarray(color_frame.get_data())
        # 指定距離以上を無視した深度画像
        depth_image = np.asanyarray(result_frame.get_data())
        depth_filtered_image = (depth_image < max_dist) * depth_image
        # 指定距離以上を無視したRGB画像
        color_filtered_image = (depth_filtered_image.reshape((HEIGHT, WIDTH, 1)) > 0)*color_image
        return color_filtered_image, result_frame

    def calc_delta_time(self):
        now = rospy.Time.now().to_sec()
        delta_t = now - self.prev_time
        self.prev_time = now
        return delta_t

    def trans_camera_to_robot(self, pos_3d):
        point = Point()
        point.x = pos_3d[2]
        point.y = -pos_3d[0]
        point.z = -pos_3d[1]
        return point

    def trans_robot_to_camera(self, pos_3d):
        return [-pos_3d[1], -pos_3d[2], pos_3d[0]]

    def mode_callback(self, msg):
        self.current_mode = msg

    def render_image(self, img, cx, cy, color, depth_size):
        # render the image
        size = abs(1/depth_size)*40
        jetson_utils.cudaDrawCircle(img, (cx, cy), size, color)  # (cx,cy), radius, color
        self.output.Render(img)

    def output_image(self, time):
        # update the title bar
        self.output.SetStatus("Object Detection {:.3f} sec | 人の座標 x:{:.3f} y:{:.3f}".format(time, self.human_info.human_point.x, self.human_info.human_point.y))

    def estimate_human_position(self, img):
        """
        @fn estimate_human_position()
        @param img 背景処理された画像
        @details darknetを用いて人検出
        """
        max_area = 0
        self.human_info.is_human = 0
        detections = self.net.Detect(img)
        if (not detections):
            self.human_info.max_area = 0
            return
        for detection in detections:
            if (detection.ClassID != 1):
                self.human_info.max_area = 0
                continue
            if (max_area < detection.Area and detection.ClassID == 1):
                max_area = int(detection.Area)
                human_pos = detection.Center
                self.human_info.is_human = 1

        # exit on input/output EOS
        # TODO: human_posを[m]でpubする
        if self.human_info.is_human == 1:
            self.human_point_pixel.x = human_pos[0]  # [pixel]
            self.human_point_pixel.y = human_pos[1]  # [pixel]
            self.human_info.max_area = max_area
        else:
            self.human_info.max_area = 0
            self.human_info.is_human = 0
        return

    def calc_human_input(self, color_intr, pose, delta_t):
        position_3d = rs.rs2_deproject_pixel_to_point(color_intr, [int(pose.x), int(pose.y)], pose.z)
        position_3d_from_robot = self.trans_camera_to_robot(position_3d)
        # 推定開始
        vx = (position_3d_from_robot.x - self.prev_human_input[0])/delta_t
        vy = (position_3d_from_robot.y - self.prev_human_input[1])/delta_t
        self.human_input = np.array([position_3d_from_robot.x, position_3d_from_robot.y, position_3d_from_robot.z, vx, vy]).T
        return position_3d_from_robot

    def main_loop(self):
        """
        @fn main_loop()
        @brief 背景処理後、numpyからcuda_imgに変換、人物検出を行う
        joyから距離のthreshholdを変更できるようにしたい
        """
        # realsense setting
        pipeline = rs.pipeline()
        config = rs.config()
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.color, WIDTH,
                             HEIGHT, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, WIDTH,
                             HEIGHT, rs.format.z16, 30)
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)
        
        profile = pipeline.start(config)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        # 内部パラメータ取得
        color_intr = rs.video_stream_profile(
            profile.get_stream(rs.stream.color)).get_intrinsics()

        r = rospy.Rate(100)  # 10hz

        # realsensenの認識距離設定
        max_dist = self.current_mode.realsense_depth_thresh/depth_scale

        ## first calculate
        # フレーム取得
        frames = pipeline.wait_for_frames()
        align = rs.align(rs.stream.color)
        frame, depth_frame = self.get_filtered_frame(align, frames, max_dist)
        # copy to CUDA memory
        cuda_mem = jetson_utils.cudaFromNumpy(frame)
        delta_t = self.calc_delta_time()
        self.estimate_human_position(cuda_mem)
        self.human_point_pixel.z = depth_frame.get_distance(int(self.human_point_pixel.x), int(self.human_point_pixel.y))
        self.human_info.human_point = self.calc_human_input(color_intr, self.human_point_pixel, delta_t)

        # KalmanFileter
        robot_vw = np.array([0.000001, 0.000001])
        kalman = kalmanfilter.KalmanFilter(self.human_input)

        rospy.loginfo("Start Human Detection using KF!")

        try:
            while not rospy.is_shutdown():
                frames = pipeline.wait_for_frames()
                frame, depth_frame = self.get_filtered_frame(align, frames, max_dist)
                if not frame.any():
                    print("frame nothing")
                cuda_mem = jetson_utils.cudaFromNumpy(frame)
                delta_t = self.calc_delta_time()

                if(self.current_mode.mode == 0):
                    r.sleep()

                elif (self.current_mode.mode == 1):
                    self.estimate_human_position(cuda_mem)
                    vel_r = calc_velocity(cnt_list[0], delta_t)
                    vel_l = calc_velocity(cnt_list[1], delta_t)
                    robot_vw[0] = (vel_l+vel_r)/2
                    if(robot_vw[0] == 0.0): robot_vw[0] = 0.000001
                    cnt_list[0] = cnt_list[1] = 0
                    robot_vw[1] = self._imu_data_raw.angular_velocity.z
                    if(robot_vw[1] == 0.0):
                        robot_vw[1] = 0.000001

                    # 人の位置をPub、もし人が見えていれば観測値をPub、見えなければ推測値をPubする    
                    if (self.human_info.is_human == 1):
                        self.human_info.human_point = self.calc_human_input(color_intr, self.human_point_pixel, delta_t)
                        human_pos_beleif = kalman.main_loop(self.prev_human_input, self.human_input, robot_vw, delta_t)
                        self.human_point_pixel.z = depth_frame.get_distance(int(self.human_point_pixel.x), int(self.human_point_pixel.y))
                        # print("分散：　", human_pos_beleif.cov)
                    else:
                        human_pos_beleif = kalman.estimation_nothing_human(robot_vw, delta_t)
                        self.human_info.human_point.x = human_pos_beleif.mean[0]
                        self.human_info.human_point.y = human_pos_beleif.mean[1]
                        self.human_info.human_point.z = human_pos_beleif.mean[2] 
                    
                    self.prev_human_input = np.array([human_pos_beleif.mean[0], human_pos_beleif.mean[1],
                                                      human_pos_beleif.mean[2], human_pos_beleif.mean[3], human_pos_beleif.mean[4]]).T
                    self.cmd_publisher.publish(self.human_info)
                    
                    # Debugパラメータ
                    if (self.debug):
                        # rospy.loginfo("human_input: x:%lf, y:%lf, z:%lf", self.human_input[0], self.human_input[1], self.human_input[2])
                        # rospy.logwarn("estimated: x:%lf, y:%lf, z:%lf", human_pos_beleif.mean[0], human_pos_beleif.mean[1], human_pos_beleif.mean[2])
                        e = kalman.sigma_ellipse(human_pos_beleif.mean[0:2], human_pos_beleif.cov[0:2, 0:2], 5)
                        self.e_list.append(e)
                        self.X_true.append(self.human_input[0])
                        self.Y_true.append(self.human_input[1])
                        self.X_est.append(human_pos_beleif.mean[0])
                        self.Y_est.append(human_pos_beleif.mean[1])
                        self.render_image(cuda_mem, self.human_point_pixel.x, self.human_point_pixel.y, (0, 0, 127, 200), self.human_point_pixel.z+0.1)
                        # estimated 3d_pos to 2d_pos
                        estimated_3d_pos = (human_pos_beleif.mean[0], human_pos_beleif.mean[1], human_pos_beleif.mean[2])
                        estimated_3d_pos = self.trans_robot_to_camera(estimated_3d_pos)
                        estimated_point_to_pixel = rs.rs2_project_point_to_pixel(color_intr, estimated_3d_pos)
                        self.render_image(cuda_mem, estimated_point_to_pixel[0], estimated_point_to_pixel[1], (0, 255, 127, 200), human_pos_beleif.mean[0]+0.1)
                        self.output_image(delta_t)

        finally:
            pipeline.stop()
            print("pipelin.stop")
            fig2 = plt.figure(figsize=(8,8)) 
            ax2 = fig2.add_subplot(111)
            ax2.set_aspect('equal')
            ax2.set_xlim(-2, 4)
            ax2.set_ylim(-4, 4)
            ax2.set_xlabel("depth", fontsize=10)
            ax2.set_ylabel("Y", fontsize=10)
            ax2.grid(True)
            ax2.legend(["Estimated", "Observed"])
            for e in self.e_list:
                ax2.add_patch(e)
            ax2.plot(self.X_est, self.Y_est, marker = "*", c = "green")
            ax2.plot(self.X_true,self.Y_true , marker = "o", c = "blue")
            fig2.savefig("/home/hashimoto/catkin_ws/src/tang/tang_detection/scripts", dpi=300)


if __name__ == "__main__":
    human_detector = HumanDetector()
    human_detector.main_loop()
