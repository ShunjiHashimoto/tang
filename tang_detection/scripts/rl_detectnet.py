#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 赤色の物体を検出かつ人を検知すれば追従
# detectnet
import jetson.inference
import jetson.utils
import argparse

import sys  # sysはPythonのインタプリタや実行環境に関する情報を扱うためのライブラリです。
import numpy as np
from numpy.lib.function_base import copy
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import pyrealsense2 as rs
import roslib.packages
import time

WIDTH = 640
HEIGHT = 480
FPS = 60
THRESHOLD = 2.0  # これより遠い距離の画素を無視する


class PubMsg():
    def __init__(self):
        self.publisher = rospy.Publisher('msg_topic', String, queue_size=10)

    def pub(self, center_x, area):
        # print(center_x, radius)
        max_area = 220417

        if(0 <= center_x and center_x <= 80 and area < max_area):
            str = "turn left"
        elif(240 < center_x and center_x <= 320 and area < max_area):
            str = "turn right"
        else:
            if(area >= max_area or area <= 10):
                str = "stop"
            else:
                str = "go ahead"
        # rospy.loginfo(str)
        self.publisher.publish(str)


class DetectNet():
    def __init__(self):
        rospy.init_node('human_detection', anonymous=True)
        # create video output object
        self.output = jetson.utils.videoOutput(
            "display://0")
        # load the object detection network
        self.net = jetson.inference.detectNet(
            "ssd-mobilenet-v2", threshold=0.5)
        # create video sources
        self.input = jetson.utils.videoSource("/dev/video2")
        self.pubmsg = PubMsg()

    ##
    #  @brief darknetを用いて人検出
    #  @param 背景処理された画像
    #
    def human_estimation(self, img):
        detections = self.net.Detect(img)
        # print the detections
        # print("detected {:d} objects in image".format(len(detections)))
        max_area = 0
        for detection in detections:
            if (detection.ClassID != 1): return
            if (max_area < detection.Area):
                max_area = detection.Area
                human_pos = detection.Center
        # render the image
        self.output.Render(img)
        # update the title bar
        self.output.SetStatus(
            "Object Detection | Network {:.0f} FPS".format(self.net.GetNetworkFPS()))
        # print(human_pos, max_area)
        # exit on input/output EOS
        if not self.input.IsStreaming() or not self.output.IsStreaming() and human_pos and max_area:
            return human_pos, max_area
        return

    ##
    #  @brief 背景処理後、numpyからcuda_imgに変換、人物検出を行う
    #  @param self
    #
    def main_loop(self):
        align = rs.align(rs.stream.color)
        config = rs.config()
        config.enable_stream(rs.stream.color, WIDTH,
                             HEIGHT, rs.format.bgr8, FPS)
        config.enable_stream(rs.stream.depth, WIDTH,
                             HEIGHT, rs.format.z16, FPS)

        pipeline = rs.pipeline()
        profile = pipeline.start(config)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        max_dist = THRESHOLD/depth_scale

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
                human_pos, max_area = self.human_estimation(cuda_mem)

                # 検出面積と位置によって動作を決定する
                if (human_pos and max_area):
                    print(human_pos, max_area)
                    self.pubmsg.pub(human_pos[0], max_area)

        finally:
            pipeline.stop()


if __name__ == "__main__":
    detectnet = DetectNet()
    detectnet.main_loop()
