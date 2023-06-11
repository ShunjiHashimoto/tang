#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import datetime
import math
import numpy as np
from geometry_msgs.msg import Point
from tang_msgs.msg import HumanInfo, Modechange, IsDismiss
from config import CameraConfig
import jetson_utils
import rospy
import roslib.packages
import pyrealsense2 as rs
import human_detection

class RealSenseCamera():
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.profile = None
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        self.config.enable_stream(rs.stream.color, CameraConfig.WIDTH , CameraConfig.HEIGHT, rs.format.rgb8, 30)
        self.config.enable_stream(rs.stream.depth, CameraConfig.WIDTH , CameraConfig.HEIGHT, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.accel)
        self.config.enable_stream(rs.stream.gyro)
    
    def start(self):
        self.profile = self.pipeline.start(self.config)
    
    def get_intrinsics(self):
        return rs.video_stream_profile(self.profile.get_stream(rs.stream.color)).get_intrinsics()
    
    def _get_filtered_frame(self, align, frames, max_dist):
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        filter_frame = CameraConfig.decimate.process(depth_frame)
        filter_frame = CameraConfig.depth_to_disparity.process(filter_frame)
        filter_frame = CameraConfig.spatial.process(filter_frame)
        filter_frame = CameraConfig.disparity_to_depth.process(filter_frame)
        filter_frame = CameraConfig.hole_filling.process(filter_frame)
        result_frame = filter_frame.as_depth_frame()
        # RGB画像
        color_image = np.asanyarray(color_frame.get_data())
        # 指定距離以上を無視した深度画像
        depth_image = np.asanyarray(result_frame.get_data())
        depth_filtered_image = (depth_image < max_dist) * depth_image
        # 指定距離以上を無視したRGB画像
        color_filtered_image = (depth_filtered_image.reshape((CameraConfig.HEIGHT, CameraConfig.WIDTH, 1)) > 0)*color_image
        return color_filtered_image, result_frame
    
    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()
        align = rs.align(rs.stream.color)
        # realsensenの認識距離設定
        max_dist = CameraConfig.depth_thresh/depth_scale
        frame, depth_frame = self._get_filtered_frame(align, frames, max_dist)
        return frame, depth_frame
    
    def stop(self):
        self.pipeline.stop()

class ResultDisplayer():
    def __init__(self, network):
        # load the object detection network
        self.net = network
        # create video output object
        self.output = jetson_utils.videoOutput("display://0")

    # 指定された座標に円を描画する
    def render_image(self, img, cx, cy, color, depth_size):
        size = abs(1/depth_size)*40
        jetson_utils.cudaDrawCircle(img, (cx, cy), size, color)  # (cx,cy), radius, color
        self.output.Render(img)
    
    # 検出結果を画面に表示する
    def output_image(self, time):
        # update the title bar
        self.output.SetStatus("Object Detection {:.3f} sec {:.0f} FPS".format(time, self.net.GetNetworkFPS()))
    
    def trans_robot_to_camera(self, pos_3d):
        return [-pos_3d[1], -pos_3d[2], pos_3d[0]]
    
    def display_inference_result(self, img, human_point_pixel, delta_t):
        self.render_image(img, human_point_pixel.x, human_point_pixel.y, (0, 0, 127, 200), human_point_pixel.z+0.1)
        self.output_image(delta_t)

class HumanFollower():
    def __init__(self):
        rospy.init_node('human_detection', anonymous=True)
        # publisher
        self.cmd_publisher = rospy.Publisher('tang_cmd', HumanInfo, queue_size=1)
        self.is_dismiss_publisher = rospy.Publisher('is_dismiss', IsDismiss, queue_size=1)
        self.is_dismiss = IsDismiss()
        # subscriber
        rospy.Subscriber("current_param", Modechange, self.mode_change_callback, queue_size=1)
        # command type
        self.current_mode = Modechange()
        self.current_mode.realsense_depth_thresh = rospy.get_param("/tang_detection/threshold")
        self.current_mode.mode = 1
        self.debug = rospy.get_param("/tang_detection/debug")
        # KalmanFilter Parameter
        self.prev_time = 0.0
        # Camera
        self.real_sense_camera = RealSenseCamera()
        self.real_sense_camera.start()
        # HumanDetector
        self.human_detector = human_detection.HumanDetector()
        self.result_displayer = ResultDisplayer(self.human_detector.net)
    
    def mode_change_callback(self, msg):
        self.current_mode = msg

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
    
    def calc_human_input(self, color_intr, pose, delta_t):
        position_3d = rs.rs2_deproject_pixel_to_point(color_intr, [int(pose.x), int(pose.y)], pose.z)
        position_3d_from_robot = self.trans_camera_to_robot(position_3d)
        human_input = np.array([position_3d_from_robot.x, position_3d_from_robot.y, position_3d_from_robot.z]).T
        return human_input

    def follow_human(self):
        r = rospy.Rate(100)  # 10hz   
        frame, depth_frame = self.real_sense_camera.get_frame()
        color_intr = self.real_sense_camera.get_intrinsics()
        # copy to CUDA memory
        cuda_mem = jetson_utils.cudaFromNumpy(frame)
        delta_t  = self.calc_delta_time()
        human_info = HumanInfo()
        human_point_pixel = Point()
        # 人物位置を取得
        human_info, human_point_pixel = self.human_detector.detect_human(cuda_mem)
        human_point_pixel.z = depth_frame.get_distance(int(human_point_pixel.x), int(human_point_pixel.y))
        human_input = self.calc_human_input(color_intr, human_point_pixel, delta_t)
        dismiss_human_time = 0.0
        rospy.loginfo("Start Human Detection!")

        while not rospy.is_shutdown():
            self.is_dismiss.flag = False
            frame, depth_frame = self.real_sense_camera.get_frame()
            if not frame.any(): print("frame Nothing")
            cuda_mem = jetson_utils.cudaFromNumpy(frame)
            delta_t = self.calc_delta_time()
            if(self.current_mode.mode == 0):
                r.sleep()
                continue
            human_info, human_point_pixel = self.human_detector.detect_human(cuda_mem)

            # 人の位置をPub、もし人が見えていれば観測値をPub、見えなければ推測値をPubする    
            if (human_info.is_human == 1):
                dismiss_human_time = 0.0
                human_input = self.calc_human_input(color_intr, human_point_pixel, delta_t)
                human_point_pixel.z = depth_frame.get_distance(int(human_point_pixel.x), int(human_point_pixel.y))
            else:
                dismiss_human_time += delta_t
                if dismiss_human_time > 2.0: 
                    human_info.is_human = 0
                    rospy.loginfo("Dissmiss human : %lf", dismiss_human_time)
                    self.is_dismiss.flag = True
                    self.is_dismiss_publisher.publish(self.is_dismiss)
                    continue
            self.is_dismiss.flag = False
            self.is_dismiss_publisher.publish(self.is_dismiss)
            self.cmd_publisher.publish(human_info)
            # rospy.loginfo("human_input: x:%lf, y:%lf, z:%lf", human_info.human_point.x, human_info.human_point.y, human_info.human_point.z)
            if (self.debug): 
                self.result_displayer.display_inference_result(cuda_mem, human_point_pixel, delta_t)

        self.real_sense_camera.stop()

if __name__ == "__main__":
    human_follower = HumanFollower()
    human_follower.follow_human()
