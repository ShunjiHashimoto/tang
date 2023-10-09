#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ros
import roslib.packages
from tang_msgs.msg import HumanInfo
from geometry_msgs.msg import Point
# detectnet
import jetson_inference
import jetson_utils
from config import HumanDetectionConfig

class BoundingBox:
    def __init__(self, top, bottom, left, right):
        self.top = top
        self.bottom = bottom
        self.left = left
        self.right = right

class HumanDetector():
    def __init__(self):
        self.human_info = HumanInfo()
        self.human_point_pixel = Point()
        # load the object detection network
        self.net = jetson_inference.detectNet(HumanDetectionConfig.model, threshold=0.5)
    
    def detect_human(self, img):
        detected_max_box_size = 0
        self.human_info.is_human = 0
        detections = self.net.Detect(img)
        if (not detections):
            self.human_info.detected_max_box_size = 0
            return self.human_info, self.human_point_pixel
        for detection in detections:
            if (detection.ClassID != 1):
                self.human_info.detected_max_box_size = 0
                continue
            if (detected_max_box_size < detection.Area and detection.ClassID == 1):
                # Bounding Box内のデプス情報を取得
                top = int(detection.Top)
                bottom = int(detection.Bottom)
                left = int(detection.Left)
                right = int(detection.Right)
                bbox = BoundingBox(top, bottom, left, right)
                detected_max_box_size = int(detection.Area)
                human_pos = detection.Center
                self.human_info.is_human = 1

        # TODO: human_posを[m]でpubする
        if self.human_info.is_human == 1:
            self.human_point_pixel.x = human_pos[0]  # [pixel]
            self.human_point_pixel.y = human_pos[1]  # [pixel]
            self.human_info.detected_max_box_size = detected_max_box_size
            self.human_info.detected_bbox = bbox
        else:
            self.human_info.detected_max_box_size = 0
            self.human_info.is_human = 0
            self.human_info.detected_bbox = BoundingBox(top=0, bottom=0, left=0, right=0)
        return self.human_info, self.human_point_pixel