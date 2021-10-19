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
THRESHOLD = 1.5  # これより遠い距離の画素を無視する
BG_PATH = "./image.png"  # 背景画像のパス



class DetectNet():
    def __init__(self):
        # pass
        # create video output object
        self.output = jetson.utils.videoOutput(
            "display://0")
        # load the object detection network
        self.net = jetson.inference.detectNet(
            "ssd-mobilenet-v2", threshold=0.5)
        # create video sources
        self.input = jetson.utils.videoSource("/dev/video2")

    def human_estimation(self, img):
        # while (True):
        # capture the next image
        # darknetの型を調べる -> <class 'jetson.utils.cudaImage'>
        # img = self.input.Capture()
        print("darknetの型", type(img))
        print("image: ", img)
        detections = self.net.Detect(img)
        # print the detections
        print("detected {:d} objects in image".format(len(detections)))
        for detection in detections:
            print(detection)
        # render the image
        self.output.Render(img)
        # update the title bar
        self.output.SetStatus(
            "Object Detection | Network {:.0f} FPS".format(self.net.GetNetworkFPS()))
        # print out performance info
        # self.net.PrintProfilerTimes()
        # exit on input/output EOS
        if not self.input.IsStreaming() or not self.output.IsStreaming():
            return

    def realsense_to_cudaimg(self):
        align = rs.align(rs.stream.color)
        config = rs.config()
        config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
        config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

        pipeline = rs.pipeline()
        profile = pipeline.start(config)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        max_dist = THRESHOLD/depth_scale

        bg_image = cv2.imread(BG_PATH, cv2.IMREAD_COLOR)

        try:
            count = 1
            while True:
                count += 1
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
                # print("realsenseの型", str(color_filtered_image.shape))
                # fill array with test colors
                # for y in range(HEIGHT):
                #     for x in range(WIDTH):
                #         px = [0, float(x)/float(WIDTH)*255, float(y)/float(HEIGHT)*255, 255]
                #         px.pop()
                #         color_filtered_image[y, x] = px
                # copy to CUDA memory
                cuda_mem = jetson.utils.cudaFromNumpy(color_filtered_image)
                self.human_estimation(cuda_mem)

                # img_a = jetson.utils.loadImage(color_filtered_image)

                # 背景合成
                # background_masked_image = (depth_filtered_image.reshape(
                #     (HEIGHT, WIDTH, 1)) == 0)*bg_image
                # composite_image = background_masked_image + color_filtered_image

                # 表示
                # cv2.namedWindow('demo', cv2.WINDOW_AUTOSIZE)
                # cv2.imshow('demo', color_filtered_image)

                if cv2.waitKey(1) & 0xff == 27:
                    break
        finally:
            pipeline.stop()
            cv2.destroyAllWindows()


    def main_loop(self):
        # self.human_estimation()
        self.realsense_to_cudaimg()


if __name__ == "__main__":
    detectnet = DetectNet()
    detectnet.main_loop()
