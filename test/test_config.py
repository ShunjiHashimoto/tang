#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class Pin:
    encoder_r_A  = 5
    encoder_r_B  = 6
    direction_r = 18
    pwm_r        = 13 
    encoder_l_A  = 22
    encoder_l_B  = 27
    direction_l = 17
    pwm_l        = 12

class PID:
    Kp = 1.5
    Kd = 0.001
    Ki = 0.01
    dt = 0.001 # 0.0001がmax
    
class PWM:
    freq = 1000
    
class FIG:
    time_data  = []
    vel_data_r = []
    vel_data_l = []
    target_vel_data = []

class Control:
    w_target = 4
    Ke_l = 1.484813
    Ke_r = 1.529544
    radian_1encoder_r = 0.00306
    radian_1encoder_l = 0.003095165176
    