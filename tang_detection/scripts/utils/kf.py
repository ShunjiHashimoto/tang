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

class KalmanFilter:
    def __init__(self):
        self.dt = 0.1
        self.ex_mean = 0.0
        self.ex_sigma = 0.1
        self.ex = np.array([ [0.0], [0.0] ])
        self.u = np.array([ [2.0*math.cos(math.radians(45))*self.dt] , [2.0*math.sin(math.radians(45))*self.dt] ])
        self.xk_1 = np.array([ [0.0] , [0.0] ])
        self.I = np.array([ [1.0, 0.0] , [0.0 ,1.0] ])
        self.estimation_xk_1 = np.array([ [0.0] , [0.0] ])
        self.belief = multivariate_normal(mean=np.array([0.0, 0.0]), cov=np.diag([1e-10, 1e-10]))
    
    def state_transition(self, xk_1):
        self.ex[0] = np.random.normal(self.ex_mean, self.ex_sigma)
        self.ex[1] = np.random.normal(self.ex_mean, self.ex_sigma)
        xk =  xk_1 + self.u + self.ex
        return xk

    def sigma_ellipse(self, p, cov, n):  ###kf3calculation
        eig_vals, eig_vec = np.linalg.eig(cov)
        ang = math.atan2(eig_vec[:,0][1], eig_vec[:,0][0])/math.pi*180
        return Ellipse(p, width=2*n*math.sqrt(eig_vals[0]),height=2*n*math.sqrt(eig_vals[1]), angle=ang, fill=False, color="green", alpha=0.5)

    def matM(self):
        return  np.array([ [0.1, 0.0], [0.0, 0.1] ])

    def matH(self):
        return  np.array([ [1.0, 0.0], [0.0, 1.0] ])
    
    def matQ(self):
        return  np.array([ [0.1, 0.0], [0.0, 0.1] ])
        
    def motion_update(self):
        self.belief.mean = self.estimation_xk_1 + self.u
        self.belief.cov = self.belief.cov + self.matM()
        self.estimation_xk_1 = self.belief.mean
    
    def one_step(self, i, elems, ax1):
        ## 前回の図を削除
        while elems: elems.pop().remove()

        ## 状態方程式で解いた現在のx, y、誤差が乗ってる実際のデータ
        xk_sys = self.state_transition(self.xk_1)
        ## データを更新
        self.xk_1 = xk_sys
        ## 青色の丸としてプロットする
        txt2 = ax1.annotate("RealRobot", (self.xk_1[0], self.xk_1[1]), (self.xk_1[0], self.xk_1[1]+0.2), fontsize=8, color = "blue")
        elems += ax1.plot(xk_sys[0], xk_sys[1], "blue", marker = 'o', markersize = 10)
        elems.append(txt2)

        ## 推定したロボットの動き、平均と分散を求める、誤差が乗っていない推定したデータ
        self.motion_update()
        
        e = self.sigma_ellipse(self.belief.mean, self.belief.cov, 1)
        elems += ax1.plot(self.belief.mean[0], self.belief.mean[1], "green", marker = 'o', markersize = 10)
        txt1 = ax1.annotate("Estimation", (self.belief.mean[0], self.belief.mean[1]), (self.belief.mean[0], self.belief.mean[1]+0.2), fontsize=8, color = "green")
        elems.append(txt1)
        elems.append(ax1.add_patch(e))
        print(self.belief.mean)

        # 観測方程式
        # カルマンゲインK
        if(i%2 == 0):
            K = self.belief.cov.dot(self.matH().T).dot(np.linalg.inv(self.matQ() + self.matH().dot(self.belief.cov).dot(self.matH().T)))
            z_error = np.array([np.random.normal(self.ex_mean, self.ex_sigma), np.random.normal(self.ex_mean, self.ex_sigma)]) + self.xk_1 - self.belief.mean
            self.belief.mean = self.belief.mean + z_error*K
            self.belief.cov = np.dot((self.I - np.dot(K, self.matH())), self.belief.cov)

    def draw(self):
        fig = plt.figure(figsize=(4,4))     #10〜16行目はそのまま
        ax = fig.add_subplot(111)
        ax.set_aspect('equal')
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.set_xlabel("X", fontsize=10)
        ax.set_ylabel("Y", fontsize=10)
        
        elems = []
        
        self.ani = anm.FuncAnimation(fig, self.one_step, fargs=(elems, ax), frames=51, interval=100, repeat=False) # 100[m/s]
        plt.show()
        
if __name__ == "__main__":
    kalman = KalmanFilter()
    kalman.draw()