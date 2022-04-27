#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file rl_detectnet.py
@brief realsenseで背景処理を行い、人物を推定する、また赤色検出を行う
"""

# ros
import rospy
import roslib.packages
from tang_msgs.msg import Command, Modechange
from sensor_msgs.msg import Joy

# detectnet
import jetson.inference
import jetson.utils
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

# decimarion_filterのパラメータ
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 1)

# spatial_filterのパラメータ
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


class PubMsg():
    """
    @class PubMsg
    @brief 目標位置と物体の大きさをPub
    """

    def __init__(self):
        self.publisher = rospy.Publisher('tang_cmd', Command, queue_size=1)
        self.command = Command()

    def pub(self, cmd):
        """
        @fn pub()
        @details 検出した人の位置と大きさをpub
        """
        self.command = cmd
        self.publisher.publish(self.command)


class DetectNet():
    """
    @class DetectNet
    @brief 人物検出&赤検出クラス
    """

    def __init__(self):
        rospy.init_node('human_detection', anonymous=True)
        # create video output object
        self.output = jetson.utils.videoOutput("display://0")
        # load the object detection network
        self.net = jetson.inference.detectNet(
            "ssd-mobilenet-v2", threshold=0.5)
        # create video sources
        self.input = jetson.utils.videoSource("/dev/video2")
        # publisher
        self.pubmsg = PubMsg()
        # subscriber
        self.joy_sub = rospy.Subscriber(
            "current_param", Modechange, self.mode_callback, queue_size=1)
        # command type
        self.command = Command()
        self.param = Modechange()
        self.param.realsense_thresh = 6.0
        self.param.current_mode = 1
        # KalmanFileter Parameter
        self.prev_time = 0.0
        self.delta_t = 0.0
        self.human_input = np.array([1.0, 1.0, 0.0, 0.001, 0.0]).T
        self.prev_human_input = np.array([0.0, 0.0, 0.0, 0.001, 0.0]).T

    def calc_delta_time(self):
        now = rospy.Time.now().to_sec()
        delta_t = now - self.prev_time
        self.prev_time = now
        return delta_t

    def trans_camera_to_robot(self, pos_3d):
        return [pos_3d[2], -pos_3d[0], -pos_3d[1]]

    def mode_callback(self, msg):
        self.param = msg

    def human_estimation(self, img):
        """
        @fn human_estimation()
        @param img 背景処理された画像
        @details darknetを用いて人検出
        """
        max_area = 0
        self.command.is_human = 0
        detections = self.net.Detect(img)
        if (not detections):
            self.command.max_area = 0
            return
        for detection in detections:
            if (detection.ClassID != 1):
                self.command.max_area = 0
                continue
            if (max_area < detection.Area and detection.ClassID == 1):
                max_area = int(detection.Area)
                human_pos = detection.Center
                self.command.is_human = 1
        # render the image
        self.output.Render(img)
        # update the title bar
        self.output.SetStatus(
            "Object Detection | Network {:.0f} FPS".format(self.net.GetNetworkFPS()))
        # exit on input/output EOS
        if not self.input.IsStreaming() and self.command.is_human == 1:
            self.command.pos_x = human_pos[0]  # [pixel]
            self.command.pos_y = human_pos[1]  # [pixel]
            self.command.max_area = max_area
        else:
            self.command.max_area = 0
            self.command.is_human = 0
        return

    def main_loop(self):
        """
        @fn main_loop()
        @brief 背景処理後、numpyからcuda_imgに変換、人物検出を行う
        joyから距離のthreshholdを変更できるようにしたい
        """
        # realsense setting
        align = rs.align(rs.stream.color)
        config = rs.config()
        config.enable_stream(rs.stream.color, WIDTH,
                             HEIGHT, rs.format.bgr8, FPS)
        config.enable_stream(rs.stream.depth, WIDTH,
                             HEIGHT, rs.format.z16, FPS)
        pipeline = rs.pipeline()
        profile = pipeline.start(config)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        # 内部パラメータ取得
        color_intr = rs.video_stream_profile(
            profile.get_stream(rs.stream.color)).get_intrinsics()

        r = rospy.Rate(10)  # 10hz

        # KalmanFileter
        robot_vw = np.array([0.000001, 0.000001])
        kalman = kalmanfilter.KalmanFilter(self.human_input)

        try:
            while not rospy.is_shutdown():
                # realsensenの認識距離設定
                self.command.depth_thresh = self.param.realsense_thresh
                max_dist = self.command.depth_thresh/depth_scale
                # フレーム取得
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                filter_frame = decimate.process(depth_frame)
                filter_frame = depth_to_disparity.process(filter_frame)
                filter_frame = spatial.process(filter_frame)
                filter_frame = disparity_to_depth.process(filter_frame)
                filter_frame = hole_filling.process(filter_frame)
                result_frame = filter_frame.as_depth_frame()
                if not result_frame or not color_frame:
                    continue

                # RGB画像
                color_image = np.asanyarray(color_frame.get_data())

                # 指定距離以上を無視した深度画像
                depth_image = np.asanyarray(result_frame.get_data())
                depth_filtered_image = (depth_image < max_dist) * depth_image

                # 指定距離以上を無視したRGB画像
                color_filtered_image = (depth_filtered_image.reshape(
                    (HEIGHT, WIDTH, 1)) > 0)*color_image

                # copy to CUDA memory
                cuda_mem = jetson.utils.cudaFromNumpy(color_filtered_image)
                delta_t = self.calc_delta_time()

                try:
                    if(self.param.current_mode == 0):
                        # 0 = teleopmode
                        rospy.loginfo("teleop mode")
                        r.sleep()
                        pass

                    elif (self.param.current_mode == 1):
                        self.human_estimation(cuda_mem)
                        if (self.command.is_human == 1):
                            self.command.depth = result_frame.get_distance(
                                int(self.command.pos_x), int(self.command.pos_y))
                            position_3d = rs.rs2_deproject_pixel_to_point(
                                color_intr, [int(self.command.pos_x), int(self.command.pos_y)], self.command.depth)
                            position_3d_from_robot = self.trans_camera_to_robot(
                                position_3d)
                            # 推定開始
                            vx = (position_3d_from_robot[0] -
                                  self.prev_human_input[0])/delta_t
                            vy = (position_3d_from_robot[1] -
                                  self.prev_human_input[1])/delta_t
                            human_input = np.array(
                                [position_3d_from_robot[0], position_3d_from_robot[1], position_3d_from_robot[2], vx, vy]).T
                            human_pos_beleif = kalman.main_loop(
                                human_input, robot_vw, delta_t)
                            self.prev_human_input = np.array([human_pos_beleif.mean[0], human_pos_beleif.mean[1],
                                                              human_pos_beleif.mean[2], human_pos_beleif.mean[3], human_pos_beleif.mean[4]]).T
                            rospy.loginfo(
                                "human_input: x:%lf, y:%lf, z:%lf", human_input[0], human_input[1], human_input[2])
                            rospy.logwarn(
                                "estimated: x:%lf, y:%lf, z:%lf", human_pos_beleif.mean[0], human_pos_beleif.mean[1], human_pos_beleif.mean[2])
                        self.pubmsg.pub(self.command)
                        pass

                    else:
                        pass

                except:
                    rospy.logwarn("nothing target")
                    continue
        finally:
            pipeline.stop()


if __name__ == "__main__":
    detectnet = DetectNet()
    detectnet.main_loop()
