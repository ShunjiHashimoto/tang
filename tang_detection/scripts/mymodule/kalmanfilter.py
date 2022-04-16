# -*- coding: utf-8 -*-
#!/usr/bin/env python

#################################################################################################################
# 人追従を行うシミュレーションを実装する
#################################################################################################################

import math
import random
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anm
from scipy.stats import multivariate_normal
from matplotlib.patches import Ellipse
import matplotlib.patches as patches

class KalmanFilter():
    """
    @class KalmanFilter
    @brief カルマンフィルタを使って人の位置を推定する
    """
    def __init__(self, human_input):
        self.belief = multivariate_normal(mean=human_input, cov=np.diag([1e-10, 1e-10, 1e-10, 1e-10, 1e-10]))
        pass

    def calc_humanpos_from_robot(self, human_input, robot_vw):
        """
        @fn calc_humanpos_from_robot(self, human_input, delta_theta, robot_vel, robot_omega)
        @param 
        @return belief
        @details 与えられた人の位置、速度、ロボットの速度、角速度を用いて人の位置を推定する
        """
        self.belief = multivariate_normal(mean=human_input, cov=np.diag([1e-10, 1e-10, 1e-10, 1e-10, 1e-10]))
        print("test")
        return self.belief
        
