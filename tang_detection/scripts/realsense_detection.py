#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file rl_detectnet.py
@brief realsenseで背景処理を行い、人物を推定する、また赤色検出を行う
"""

# detectnet
import jetson.inference
import jetson.utils
import argparse

# ros
import rospy
import roslib.packages
from tang_detection.msg import Command
from tang_teleop.msg import Modechange
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
import I2C_LCD_driver


# 赤色検出モジュール
import red_detection

# realsense
import pyrealsense2 as rs

# others
import sys
import numpy as np
from numpy.lib.function_base import copy
import cv2
import time
import math

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
        # msg type
        self.command = Command()
        self.detection_red = red_detection.DetectRed()
        # lcd module
        self.lcd = I2C_LCD_driver.lcd()
        self.param = Modechange()
        self.param.realsense_thresh = 4.0
        self.param.current_mode = 1

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
                self.command.is_human = 0
                self.command.max_area = 0
                continue
            if (max_area < detection.Area and detection.ClassID == 1):
                max_area = int(detection.Area)
                human_pos = detection.Center
                self.command.is_human = 1
        # render the image
        self.output.Render(img)
        # # update the title bar
        self.output.SetStatus(
            "Object Detection | Network {:.0f} FPS".format(self.net.GetNetworkFPS()))
        # exit on input/output EOS
        if not self.input.IsStreaming() and self.command.is_human == 1:
            self.command.pos_x = human_pos[0]
            self.command.pos_y = human_pos[1]
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
        align = rs.align(rs.stream.color)
        config = rs.config()
        config.enable_stream(rs.stream.color, WIDTH,
                             HEIGHT, rs.format.bgr8, FPS)
        config.enable_stream(rs.stream.depth, WIDTH,
                             HEIGHT, rs.format.z16, FPS)
        pipeline = rs.pipeline()
        profile = pipeline.start(config)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        r = rospy.Rate(10)  # 10hz

        try:
            while not rospy.is_shutdown():
                # realsensenの認識距離設定
                self.command.depth_thresh = self.param.realsense_thresh
                rospy.logwarn("距離のしきい値： %lf", self.param.realsense_thresh)
                max_dist = self.command.depth_thresh/depth_scale
                # フレーム取得
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)

                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                if not depth_frame or not color_frame:
                    continue
                # RGB画像
                color_image = np.asanyarray(color_frame.get_data())

                # 深度画像
                depth_color_frame = rs.colorizer().colorize(depth_frame)
                depth_color_image = np.asanyarray(depth_color_frame.get_data())

                # 指定距離以上を無視した深度画像
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_filtered_image = (depth_image < max_dist) * depth_image

                # 指定距離以上を無視したRGB画像
                # realsense側の型を調べる -> <class 'numpy.ndarray'>
                color_filtered_image = (depth_filtered_image.reshape(
                    (HEIGHT, WIDTH, 1)) > 0)*color_image

                # copy to CUDA memory
                cuda_mem = jetson.utils.cudaFromNumpy(color_filtered_image)
                try:
                    if(self.param.current_mode == 0):
                        # 0 = teleopmode
                        rospy.loginfo("teleop mode")
                        r = rospy.Rate(10)  # 10hz
                        self.lcd.lcd_display_string("TANG", 1)
                        self.lcd.lcd_display_string("~ Teleop mode ~", 2)
                        r.sleep()
                        pass

                    elif(self.param.current_mode == 1):
                        # 1 = humanmode
                        self.lcd.lcd_display_string("~ Follow mode ~", 1)
                        self.lcd.lcd_display_string(
                            "thresh = " + str(self.command.depth_thresh), 2)
                        rospy.loginfo("human_detection mode")
                        self.human_estimation(cuda_mem)
                        if (self.command.is_human == 1):
                            current_depth = depth_frame.get_distance(int(self.command.pos_x), int(self.command.pos_y))
                            rospy.logwarn("human_depth : %lf", current_depth)
                            self.command.depth = current_depth
                        self.pubmsg.pub(self.command)
                        pass

                    elif(self.param.current_mode == 2):
                        # 2 = redmode
                        r = rospy.Rate(10)  # 10hz
                        self.lcd.lcd_display_string("TANG", 1)
                        self.lcd.lcd_display_string("~ Red mode ~", 2)
                        rospy.loginfo("red_detection mode")
                        self.command.pos_x, self.command.max_area = self.detection_red.red_detection(
                            color_filtered_image, ret=1)
                        print(self.command.max_area)
                        self.pubmsg.pub(self.command)
                        r.sleep()
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
