# -*- coding: utf-8 -*-
#!/usr/bin/env python3

# import sys 
# sys.path.append('/home/hashimoto/catkin_ws/src/hs_ros/tang/tang_detection/mymodule')

# 赤色検出モジュール
from mymodule import kalmanfilter 
import numpy as np


if __name__ == "__main__":
    human_input = np.array([1.0, 0.0, 0.0, 1.0, 0.0]).T 
    robot_vw = np.array([1.0, 1.0])
    kalmanfilter = kalmanfilter.KalmanFilter(human_input)
    human_pos_beleif = kalmanfilter.calc_humanpos_from_robot(human_input, robot_vw)