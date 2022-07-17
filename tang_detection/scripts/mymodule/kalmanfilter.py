# -*- coding: utf-8 -*-
#!/usr/bin/env python

import math
import random
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
        self.belief   = multivariate_normal(mean=human_input, cov=np.diag([1e-10, 1e-10, 1e-10, 1e-10, 1e-10]))
        self.w_mean   = 0.0
        self.sigma_w  = 0.05 # 人の速度に対するノイズ
        self.v_mean   = 0.0
        self.sigma_vx = 0.01 # depth
        self.sigma_vy = 0.01 # right and left
        self.sigma_vz = 0.01 # tall
        self.time_interval = 0.17

    # 誤差楕円
    # p：楕円の中心座標（x, y）
    # cov：共分散
    def sigma_ellipse(self, p, cov, n):  
        # 固有値、固有ベクトルを求める
        eig_vals, eig_vec = np.linalg.eig(cov)
        ang = math.atan2(eig_vec[:,0][1], eig_vec[:,0][0])/math.pi*180 # eig_vec[:, 0]は0行目
        return Ellipse(p, angle = ang, width=2*n*math.sqrt(eig_vals[0]), height=2*n*math.sqrt(eig_vals[1]), fill=False, color="green", alpha=0.5)
    
    """
    @fn matG()
    @note 状態方程式におけるG
    """
    def matG(self):
        return np.array([ [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [self.time_interval, 0.0], [0.0, self.time_interval]])

    """
    @fn fx()
    @note 状態方程式におけるfx
    """
    def fx(self, xt_1, robot_v, robot_w):
        if(self.time_interval>1.0): self.time_interval = 0.1
        t = self.time_interval
        delta_theta = robot_w*t
        delta_l = 2*(robot_v/robot_w)*math.sin(delta_theta/2)
        delta_x = delta_l*math.cos(delta_theta/2)
        delta_y = delta_l*math.sin(delta_theta/2)
        xt_1_ = xt_1[0]
        yt_1 = xt_1[1]
        zt_1 = xt_1[2]
        vx = xt_1[3]
        vy = xt_1[4]
        xt =  np.array([ (xt_1_ + t*vx - delta_x)*math.cos(delta_theta) + (yt_1 + t*vy - delta_y)*math.sin(delta_theta), \
                          -(xt_1_ + t*vx - delta_x)*math.sin(delta_theta) + (yt_1 + t*vy - delta_y)*math.cos(delta_theta), \
                          zt_1, \
                          vx*math.cos(delta_theta) + vy*math.sin(delta_theta) -robot_v, \
                          - vx * math.sin(delta_theta) + vy * math.cos(delta_theta)])
        # print("delta_theta", format(delta_theta, '.3f'), "delta_l: ", format(delta_l, '.3f'), "delta_x" , format(delta_x, '.3f'), "delta_y", format(delta_y, '.3f'))
        return xt

    """
    @fn human_state_transition(xt_1, robot_v, robot_w, t)
    @note 状態方程式、人の位置はロボット座標系
    @param xt_1 xt_1[0] = 人の位置x
                xt_1[1] = 人の位置y
                xt_1[2] = 人の位置z
                xt_1[3] = 人の速度vx
                xt_1[4] = 人の速度vy
    """
    def human_state_transition(self, xt_1, robot_vw):
        Fx = self.fx(xt_1, robot_vw[0], robot_vw[1])
        w = np.array([np.random.normal(self.w_mean, self.sigma_w), np.random.normal(self.w_mean, self.sigma_w)]) # 平均0,0 分散1.0
        G = self.matG()
        Gw = G.dot(w)
        xt = Fx + Gw
        return xt

    """
    @fn matH()
    @note 観測方程式におけるHx
    """
    def matH(self):
        return np.array([ [1.0, 0.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0, 0.0] ])

    """
    @fn observation_state_transition(xt_1)
    @note 観測方程式、入力にノイズをのせた値
    @param xt_1 xt_1[0] = 人の位置x
                xt_1[1] = 人の位置y
                xt_1[2] = 人の位置z
                xt_1[3] = 人の速度vx
                xt_1[4] = 人の速度vy
    """
    def observation_state_transition(self, xt):
        Hx = np.dot(self.matH(), xt) # mat_h = matH
        v  = np.array([np.random.normal(self.v_mean, self.sigma_vx), 
                      np.random.normal(self.v_mean, self.sigma_vy), 
                      np.random.normal(self.v_mean, self.sigma_vz)])
        zt = Hx + v
        return zt

    # 移動の誤差
    def matM(self):
        return  np.array([ [self.sigma_w**2, 0.0], [0.0, self.sigma_w**2] ])   
    
    def matF(self, robot_omega):
        delta_theta = robot_omega*self.time_interval
        cos_ = math.cos(delta_theta)
        sin_ = math.sin(delta_theta)
        t = self.time_interval
        if(robot_omega < 0.01 and robot_omega > -0.01):
            return np.array([ [1.0, 0.0, 0.0, 1*t, 0.0], 
                          [0.0, 1.0, 0.0, 0.0, 1*t], 
                          [0.0, 0.0, 1.0, 0.0, 0.0],
                          [1.0, 0.0, 0.0, 1.0, 0.0],
                          [0.0, 1.0, 0.0, 0.0, 1.0] ])
        return np.array([ [cos_, sin_, 0.0, t*cos_, t*sin_], 
                          [-sin_, cos_, 0.0, -t*sin_, t*cos_], 
                          [0.0, 0.0, 1.0, 0.0, 0.0],
                          [cos_, sin_, 0.0, cos_, sin_],
                          [-sin_, cos_, 0.0, -sin_, cos_] ])
    
    def matA(self, xt, v, w):
        delta_theta = w*self.time_interval
        cos_ = math.cos(delta_theta)
        sin_ = math.sin(delta_theta)
        t = self.time_interval
        x = xt[0]
        y = xt[1]
        vx = xt[3]
        vy = xt[4]
        if(w < 0.01 and w > -0.01):
            return np.array([[-t, 0.0],
                             [0.0, 0.0],
                             [0.0, 0.0],
                             [-1.0, 0.0],
                             [0.0, 0.0] ])
        return np.array([ [-sin_/w, -(x + vx*t)*t*sin_ + (y + vy*t)*t*cos_ + (v/w)*(sin_/w - t*cos_)], 
                          [(1-cos_)/w, -(x + vx*t)*t*cos_ - (y + vy*t)*t*sin_ - (v/w)*(cos_/w + t*sin_)], 
                          [0.0, 0.0],
                          [-1.0, -vx*t*sin_+vy*t*cos_],
                          [0.0, -vx*t*cos_-vy*t*sin_] ])

    """
    @fn motion_update()
    @note 推測した人の位置と共分散を更新する
    @param  
    """
    def motion_update(self, mean_t_1, cov_t_1, robot_vw):
        # 入力による位置の変化f(x, y, z, x', y'), ノイズなし
        self.belief.mean = self.fx(mean_t_1, robot_vw[0], robot_vw[1])
        M = self.matM() # 入力のばらつき(x, yの速度のばらつき)
        F = self.matF(robot_vw[1]) # xがずれたときに移動後のxがどれだけずれるか
        A = self.matA(self.belief.mean, robot_vw[0], robot_vw[1]) # 人への入力u(x, yの速度)がずれたとき、xがどれだけずれるか 
        self.belief.cov = np.dot(F, np.dot(cov_t_1, F.T)) + np.dot(A, np.dot(M, A.T))

    def mat_h(self):
        return np.array([ [1.0, 0.0, 0.0, self.time_interval, 0.0], [0.0, 1.0, 0.0, 0.0, self.time_interval], [0.0, 0.0, 1.0, 0.0, 0.0] ])

    def matQ(self):
        return np.array([ [self.sigma_vx**2, 0.0, 0.0], [0.0, self.sigma_vy**2, 0.0], [0.0, 0.0, self.sigma_vz**2] ])
 
    """
    @fn observation_update()
    @note 観測した人の位置と共分散を更新する、zはカメラからの観測値（ロボット座標系）
    @param  
    """
    def observation_update(self, mean_t_1, cov_t_1, xt_1):
        z = np.array([ xt_1[0] , xt_1[1] , xt_1[2]])
        H = self.mat_h()
        Q = self.matQ()
        I = np.eye(5)
        z_error = z - np.dot(self.matH(), mean_t_1)
        # calc maharanobis distance
        S = Q + np.dot(np.dot(H, cov_t_1), H.T)
        d2 = np.dot(np.dot(z_error, np.linalg.inv(S)), z_error.reshape(-1, 1))
        if(d2 > 5.9 and xt_1[0] != 0.000): 
            print("out layer", d2, "観測値z", z)
            return 
        K = np.dot(np.dot(cov_t_1, H.T), np.linalg.inv(S))
        self.belief.mean += np.dot(K, z_error)  # 平均値更新
        self.belief.cov = (I - K.dot(H)).dot(self.belief.cov) # 共分散更新
    
    def main_loop(self, xt_1, zt_1, robot_vw, loop_rate):
        self.time_interval = loop_rate
        # 人の次の位置を計算
        xt = self.human_state_transition(xt_1, robot_vw)
        # 観測値の次の値を計算
        zt = self.observation_state_transition(zt_1)
        # 人の動きを推測、平均と分散を求める
        self.motion_update(self.belief.mean, self.belief.cov, robot_vw)
        # 観測方程式：カルマンゲインK
        self.observation_update(self.belief.mean, self.belief.cov, zt)
        self.belief.mean[2] = zt[2]
        return self.belief
    
    def estimation_nothing_human(self, robot_vw, loop_rate):
        self.time_interval = loop_rate
        # 人の次の位置を計算
        self.belief.mean = self.human_state_transition(
            self.belief.mean, robot_vw)
        return self.belief
