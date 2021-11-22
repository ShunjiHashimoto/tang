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
from std_msgs.msg import Int16
from std_msgs.msg import String
from sensor_msgs.msg import Joy

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
        self.threshold = rospy.get_param("/tang_detection/threshold")
        self.mode = 0
        # create video output object
        self.output = jetson.utils.videoOutput("display://0")
        # load the object detection network
        self.net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
        # create video sources
        self.input = jetson.utils.videoSource("/dev/video2")
        # publisher
        self.pubmsg = PubMsg()
        # subscriber
        self.joy_sub = rospy.Subscriber("current_mode", Int16, self.mode_callback, queue_size=1)
        # msg type
        self.command = Command()
        self.detection_red = red_detection.DetectRed()

    def mode_callback(self, msg):
        self.mode = msg.data
        print(self.mode)

    def human_estimation(self, img):
        """
        @fn human_estimation()
        @param img 背景処理された画像
        @details darknetを用いて人検出
        """
        detections = self.net.Detect(img)
        if (not detections):
            return
        max_area = 0
        self.command.is_human = 0
        for detection in detections:
            if (detection.ClassID != 1):
                continue
            if (max_area < detection.Area and detection.ClassID == 1):
                max_area = int(detection.Area)
                human_pos = detection.Center
                self.command.is_human = 1
        # render the image
        self.output.Render(img)
        # # update the title bar
        self.output.SetStatus("Object Detection | Network {:.0f} FPS".format(self.net.GetNetworkFPS()))
        # exit on input/output EOS
        if not self.input.IsStreaming() or not self.output.IsStreaming():
            self.command.pos = human_pos[0]
            self.command.max_area = max_area
        return

    def main_loop(self):
        """
        @fn main_loop()
        @brief 背景処理後、numpyからcuda_imgに変換、人物検出を行う
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
        max_dist = self.threshold/depth_scale
        r = rospy.Rate(10) # 10hz

        try:
            while not rospy.is_shutdown():
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
                depth_gray_filtered_image = (
                    depth_filtered_image * 255. / max_dist).reshape((HEIGHT, WIDTH)).astype(np.uint8)

                # 指定距離以上を無視したRGB画像
                # realsense側の型を調べる -> <class 'numpy.ndarray'>
                color_filtered_image = (depth_filtered_image.reshape(
                    (HEIGHT, WIDTH, 1)) > 0)*color_image

                # copy to CUDA memory
                cuda_mem = jetson.utils.cudaFromNumpy(color_filtered_image)
                try:
                    if(self.mode == 0):
                        # 0 = teleopmode
                        rospy.loginfo("teleop mode")
                        r.sleep()
                        pass

                    elif(self.mode == 1):
                        # 1 = humanmode
                        rospy.loginfo("human_detection mode")
                        self.human_estimation(cuda_mem)
                        self.pubmsg.pub(self.command)
                        r.sleep()

                    elif(self.mode == 2):
                        # 2 = redmode
                        rospy.loginfo("red_detection mode")
                        self.command.pos, self.command.max_area = self.detection_red.red_detection(color_filtered_image, ret = 1)
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
        
    def mode_callback(self, msg):
        self.mode = msg.data


if __name__ == "__main__":
    detectnet = DetectNet()
    detectnet.main_loop()
