# -*- coding: utf-8 -*-
#!/usr/bin/env python3

from mymodule import kalmanfilter 
from mymodule import red_detection 
import numpy as np
import time

if __name__ == "__main__":
    human_input = np.array([1.0, 1.0, 0.0, 0.001, 0.0]).T 
    robot_vw = np.array([0.000001, 0.000001])
    kalman = kalmanfilter.KalmanFilter(human_input, 0.1)
    cnt = 0
    while(1):
        # robot_vwは毎時の値を取得
        # human_inputはカメラから得られた結果、人が同じ位置に居続ける
        # それらをもとに推測した位置と分散を返す
        if(cnt%10 == 0): 
            human_input = np.array([0.0, 0.0, 0.0, 0.001, 0.0]).T
        else:
             human_input = np.array([1.0, 1.0, 0.0, 0.001, 0.0]).T 
        cnt += 1
        human_pos_beleif = kalman.main_loop(human_input, robot_vw)
        print("HumanPos:x, y", human_input[0], human_input[1])
        print("BeliefPos:x, y", human_pos_beleif.mean[0], human_pos_beleif.mean[1])
        time.sleep(0.5)