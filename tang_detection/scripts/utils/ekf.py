# -*- coding: utf-8 -*-
#!/usr/bin/env python

#################################################################################################################
# EKFを使った推定を行って見ます。
# 2次元平面を等速円運動「しようと」している物体があるとします。この物体には外乱が加わっているとします。
# 原点にいる観測者からは、各時刻で、物体への距離と方位角が計測できるとします。この観測には観測ノイズが加わります。
# 円運動の半径はL=100[m]とし、角速度はω=π/10、物体位置に対するノイズの分散はσ2w=1.02、
# 物体への距離の観測のノイズの分散はσ2r=10.02、物体の方位角の分散はσ2b=(5.0π/180)2とします。
#################################################################################################################

import numpy as np
import math
import random
import time
import matplotlib.pyplot as plt
import matplotlib.animation as anm
from scipy.stats import multivariate_normal
from matplotlib.patches import Ellipse

class KalmanFilter:
    def __init__(self):
        # 円運動の半径
        self.radius = 100.0
        self.dt = 1.0
        self.ex_mean = math.pi/10
        self.ez_mean = 100.0
        # ロボットの姿勢に関するノイズ
        self.sigma_b = 5.0*math.pi/180
        # 入力に関するノイズ(w)
        self.sigma_omega = 0.01
        # 観測のノイズ(l, Φ)
        self.εz_t = np.array([ 0.0, 0.0]).T
        self.sigma_r = 10.0
        # 角速度
        self.omega = math.pi/10
        # ロボットの実際の値（x, y, theta）
        self.pos = np.array([  self.radius  , 0.0])
        self.z = np.array([ 0.0 , 0.0 ])
        # 入力(v, omega)
        # self.u_t = np.array([ self.omega ])
        # 平均
        self.mean_t_1 = np.array([ 0.0 , 0.0 ]).T
        # 信念分布
        self.belief = multivariate_normal(mean=np.array([self.radius, 0.0]), cov=np.diag([1e-10, 1e-10]))
        # 評価指標
        self.sum_observation = self.sum_estimation = 0
        
    def get_distance(self, x1, y1, x2, y2):
        d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return d

    def matI(self):
        return np.array([ [1.0, 0.0] , [0.0 ,1.0] ])
    
    # 移動の誤差
    def matM(self):
        return  np.array([ [self.sigma_omega**2] ])   
   
    # 入力の誤差
    def matA(self, theta_t_1):
        x = self.radius*math.sin(theta_t_1) + self.radius*theta_t_1*math.cos(theta_t_1)
        y = self.radius*math.cos(theta_t_1) - self.radius*theta_t_1*math.sin(theta_t_1)
        return np.array( [x, y] )

    def matF(self, theta_t_1):
        return np.array([ [1.0, 0.0] , [0.0 ,1.0] ])
    
    def matH(self, x, y):
        return  np.array([ [x/math.sqrt(x**2 + y**2), y/math.sqrt(x**2 + y**2)], [0.0, 0.0] ])
    
    # 観測の共分散
    def matQ(self):
        return  np.array([ [self.sigma_r**2, 0.0], [0.0, self.sigma_b**2] ])

    # 誤差楕円
    def sigma_ellipse(self, p, cov, n):  
        eig_vals, eig_vec = np.linalg.eig(cov)
        ang = math.atan2(eig_vec[:,0][1], eig_vec[:,0][0])/math.pi*180
        return Ellipse(p, width=2*n*math.sqrt(eig_vals[0]), height=2*n*math.sqrt(eig_vals[1]), fill=False, color="green", alpha=0.5)

    # 状態遷移方程式、ノイズ有り(x, y, theta)
    def state_transition(self, xt_1, t):
        # 入力のノイズ
        εp_t = np.random.normal(self.ex_mean, self.sigma_omega) # 平均0,0 分散1.0
        # 実際の入力（ノイズ有り）
        omega = εp_t
        # 入力による位置の変化f(x, y, theta)
        f = np.array([ -(omega*self.radius)*math.sin(omega*t) , (omega*self.radius)*math.cos(omega*t) ])
        xt =  xt_1 + f
        theta_t = omega*t
        return xt, theta_t
    
    # 観測方程式、ノイズ有り(l, Φ)
    def state_observation(self, zt_1, theta):
        # zt_1 = (x, y, theta)
        if(abs(zt_1[0]) < 1e-5 ): zt_1[0] = 1e-5
        self.εz_t[0] = np.random.normal(self.get_distance(0, 0, zt_1[0], zt_1[1]), self.sigma_r)
        self.εz_t[1] = np.random.normal(theta, self.sigma_b)
        zt = np.array([ self.εz_t[0], self.εz_t[1]])
        return zt
        
    # 推測したロボットの位置と共分散を更新する
    def motion_update(self, mean_t_1, cov_t_1, t):
        # 入力による位置の変化f(x, y, theta)
        f = np.array([ -(self.omega*self.radius)*math.sin(self.omega*t) , (self.omega*self.radius)*math.cos(self.omega*t) ])
        theta_t = self.omega*t
        self.belief.mean = mean_t_1 + f
        M = self.matM()
        A = self.matA(theta_t)
        F = self.matF(theta_t)
        self.belief.cov = F*cov_t_1*F.T + A*M*A.T
    
    def one_step(self, i, elems, ax1):
        ## 前回の図を削除
        while elems: elems.pop().remove()
        if(i == 0): time.sleep(2.0)
        # print(elems, "\n")

        ## 実際の値 ########################################################################################
        ## 状態方程式で解いた現在のpos(x, y, theta)、誤差が乗ってる実際のデータ
        self.pos, theta = self.state_transition(self.pos, i)
        elems += ax1.plot(self.pos[0], self.pos[1], "blue", marker = 'o', markersize = 8)
        ## 観測方程式で解いた現在の観測値、ノイズ有り(x, y, theta)
        self.z = self.state_observation(self.pos, theta)
        elems += ax1.plot(self.z[0]*math.cos(self.z[1]), self.z[0]*math.sin(self.z[1]), "red", marker = '*', markersize = 8, label="test")
       
        ## 推測 ########################################################################################    
        ## 推定したロボットの動き、平均と分散を求める、誤差が乗っていない推定したデータ
        self.motion_update(self.belief.mean, self.belief.cov, i)
        estimated_theta = self.omega*i
        # 観測方程式：カルマンゲインK
        H = self.matH(self.belief.mean[0], self.belief.mean[1])
        Q = self.matQ()
        I = self.matI()
        K = self.belief.cov.dot(H.T).dot(np.linalg.inv(Q + H.dot(self.belief.cov).dot(H.T)))
        # # 観測誤差
        pos_to_ltheta = np.array([ self.get_distance(0, 0, self.belief.mean[0], self.belief.mean[1]), estimated_theta ])
        z_error =  self.z - pos_to_ltheta
        # # 平均値更新
        self.belief.mean += np.dot(K, z_error)
        # # 共分散更新
        self.belief.cov = (I - K.dot(H)).dot(self.belief.cov)

        e = self.sigma_ellipse(self.belief.mean, self.belief.cov, 2)
        elems.append(ax1.add_patch(e))
        elems += ax1.plot(self.belief.mean[0], self.belief.mean[1], "green", marker = 'o', markersize = 8)
        self.sum_observation += self.get_distance(self.z[0], self.z[1], self.pos[0], self.pos[1])
        self.sum_estimation  += self.get_distance(self.belief.mean[0], self.belief.mean[1], self.pos[0], self.pos[1])
        print("観測値の誤差: " , self.sum_observation, "推定値の誤差: ", self.sum_estimation)
        ax1.legend(["Truth", "Observed", "Estimated"])
        ax1.set_title(f"test /timestep={i}")
        ax1.set_title("例題\n \
            2次元平面を等速円運動「しようと」している物体があるとします。この物体には外乱が加わっているとします。\n \
            原点にいる観測者からは、各時刻で、物体への距離と方位角が計測できるとします。この観測には観測ノイズが加わります。\n  \
            円運動の半径はL=100[m]とし、角速度はω=π/10、物体位置に対するノイズの分散はσw=1.0、\n \
            物体への距離の観測のノイズの分散はσr=10.0、物体の方位角の分散はσb=5.0π/180とします。\n \
            t=" + str(i) + "[sec]", fontname="IPAexGothic")
        elems += ax1.plot(0.0, 0.0, "black", marker = 'x', markersize = 8)
        # ax1.legend(["RealRobot", "Estimated"])

    def draw(self):
        fig = plt.figure(figsize=(10,10))     #10〜16行目はそのまま
        ax = fig.add_subplot(111)
        ax.set_aspect('equal')
        ax.set_xlim(-180, 180)
        ax.set_ylim(-180, 180)
        ax.set_xlabel("X", fontsize=10)
        ax.set_ylabel("Y", fontsize=10)
        
        elems = []
        self.ani = anm.FuncAnimation(fig, self.one_step, fargs=(elems, ax), frames=51, interval=200, repeat=False) # 100[m/s]
        plt.show()
        
if __name__ == "__main__":
    kalman = KalmanFilter()
    kalman.draw()