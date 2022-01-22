# -*- coding: utf-8 -*-
#!/usr/bin/env python
import numpy as np
import math
import random
import time
import matplotlib.pyplot as plt
import matplotlib.animation as anm
from scipy.stats import multivariate_normal
from matplotlib.patches import Ellipse
# from matplotlib.font_manager import FontProperties

class KalmanFilter:
    def __init__(self):
        self.dt = 1.0
        self.ex_mean = 0.0
        self.sigma_p = 1.0
        self.sigma_z = 5.0
        self.εp_t = self.εz_t = np.array([ 0.0, 0.0 ]).T
        # ロボットの実際の値
        self.pos = np.array([ 0.0 , 0.0 ])
        self.z = np.array([ 0.0 , 0.0 ])
        # 入力
        self.u_t = np.array([ 2.0*math.cos(math.radians(45))*self.dt , 2.0*math.sin(math.radians(45))*self.dt ])
        # 平均
        self.mean_t_1 = np.array([ 0.0 , 0.0 ]).T
        # 信念分布
        self.belief = multivariate_normal(mean=np.array([0.0, 0.0]), cov=np.diag([1e-10, 1e-10]))
        # 評価指標
        self.sum_observation = self.sum_estimation = 0
        
    def get_distance(self, x1, y1, x2, y2):
        d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return d

    def matI(self):
        return np.array([ [1.0, 0.0] , [0.0 ,1.0] ])
    
    # 移動の誤差
    def matM(self, sigma_p):
            return  np.array([ [sigma_p**2, 0.0], [0.0, sigma_p**2] ])
    
    def matH(self):
        return  np.array([ [1.0, 0.0], [0.0, 1.0] ])
    
    # 観測の共分散
    def matQ(self, sigma_z):
        return  np.array([ [sigma_z**2, 0.0], [0.0, sigma_z**2] ])

    # 誤差楕円
    def sigma_ellipse(self, p, cov, n):  
        eig_vals, eig_vec = np.linalg.eig(cov)
        ang = math.atan2(eig_vec[:,0][1], eig_vec[:,0][0])/math.pi*180
        return Ellipse(p, width=2*n*math.sqrt(eig_vals[0]), height=2*n*math.sqrt(eig_vals[1]), fill=False, color="green", alpha=0.5)

    # 状態遷移方程式、ノイズ有り
    def state_transition(self, xt_1):
        self.εp_t[0] = np.random.normal(self.ex_mean, self.sigma_p)
        self.εp_t[1] = np.random.normal(self.ex_mean, self.sigma_p)
        xt =  xt_1 + self.u_t + self.εp_t
        return xt
    
    # 観測方程式、ノイズ有り
    def state_observation(self, zt_1):
        self.εz_t[0] = np.random.normal(self.ex_mean, self.sigma_z)
        self.εz_t[1] = np.random.normal(self.ex_mean, self.sigma_z)
        zt = zt_1 + self.εz_t
        return zt
        
    # 推測したロボットの位置と共分散を更新する
    def motion_update(self, mean_t_1, cov_t_1):
        self.belief.mean = mean_t_1 + self.u_t
        self.belief.cov = cov_t_1 + self.matM(self.sigma_p)
    
    def one_step(self, i, elems, ax1):
        ## 前回の図を削除
        # if elems: elems.pop()
        # print(elems, "\n")

        ## 実際の値 ########################################################################################
        ## 状態方程式で解いた現在のpos(x, y)、誤差が乗ってる実際のデータ
        self.pos = self.state_transition(self.pos)
        elems += ax1.plot(self.pos[0], self.pos[1], "blue", marker = 'o', markersize = 5)
        ## 観測方程式で解いた現在の観測値、ノイズ有り
        self.z = self.state_observation(self.pos)
        elems += ax1.plot(self.z[0], self.z[1], "red", marker = 'x', markersize = 5, label="test")
       
        ## 推測 ########################################################################################    
        ## 推定したロボットの動き、平均と分散を求める、誤差が乗っていない推定したデータ
        self.motion_update(self.belief.mean, self.belief.cov)
        # 観測方程式：カルマンゲインK
        H = self.matH()
        Q = self.matQ(self.sigma_z)
        I = self.matI()
        K = self.belief.cov.dot(H.T).dot(np.linalg.inv(Q + H.dot(self.belief.cov).dot(H.T)))
        # 観測誤差
        z_error =  self.z -self.belief.mean
        # 平均値更新
        self.belief.mean += np.dot(K, z_error)
        # 共分散更新
        self.belief.cov = np.dot((I - np.dot(K, H)), self.belief.cov)

        # e = self.sigma_ellipse(self.belief.mean, self.belief.cov, 2)
        # elems.append(ax1.add_patch(e))
        elems += ax1.plot(self.belief.mean[0], self.belief.mean[1], "green", marker = 'o', markersize = 5)
        self.sum_observation += self.get_distance(self.z[0], self.z[1], self.pos[0], self.pos[1])
        self.sum_estimation  += self.get_distance(self.belief.mean[0], self.belief.mean[1], self.pos[0], self.pos[1])
        print("観測値の誤差: " , self.sum_observation, "推定値の誤差: ", self.sum_estimation)
        ax1.legend(["Truth", "Observed", "Estimated"])
        ax1.set_title(f"test /timestep={i}")
        ax1.set_title("例題\n XY平面上でt=0[sec]に原点を出発し、45度方向に秒速2[m/sec]で移動している物体がある。\n  \
            この移動には外乱による影響がある。また、1秒ごとに位置座標の観測がある。\n \
            この観測にもノイズが含まれる。この物体の位置をカルマンフィルタを用いて推定する。\n \
            t=" + str(i) + "[sec]", fontname="IPAexGothic")
        # ax1.legend(["RealRobot", "Estimated"])

    def draw(self):
        fig = plt.figure(figsize=(10,10))     #10〜16行目はそのまま
        ax = fig.add_subplot(111)
        ax.set_aspect('equal')
        ax.set_xlim(0, 102)
        ax.set_ylim(0, 102)
        ax.set_xlabel("X", fontsize=10)
        ax.set_ylabel("Y", fontsize=10)
        
        elems = []
        
        self.ani = anm.FuncAnimation(fig, self.one_step, fargs=(elems, ax), frames=51, interval=200, repeat=False) # 100[m/s]
        plt.show()
        
if __name__ == "__main__":
    kalman = KalmanFilter()
    kalman.draw()