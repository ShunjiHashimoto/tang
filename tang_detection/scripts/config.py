#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pyrealsense2 as rs

class CameraConfig:
    WIDTH  = 640
    HEIGHT = 480
    FPS    = 60
    depth_thresh = 6.0
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

class GearConfig:
    Pitch = 6.1850105367549055/2/1000
    right_gear_pin = 24
    left_gear_pin = 22

class HumanDetectionConfig:
    # ssd-mobilenet-v2, peoplenet
    model = "ssd-mobilenet-v2"
