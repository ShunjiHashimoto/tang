#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class Pin:
    r_direction = 18
    l_direction = 17
    teleop_mode = 16
    follow_mode = 21
    r_pwm = 13
    l_pwm = 12
    vrx_channel = 0
    vry_channel = 1
    swt_channel = 2

class PID:
    p_gain = 15.0 
    d_gain = 7.0
    #d_gain = 7.0
    dt = 0.1

class PWM:
    freq = 1000

class HumanFollowParam:
    depth_min_thresh = 0.8
