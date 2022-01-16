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


# 単位行列
I = np.matrix([[1 , 0],
               [0, 1]])

class KalmanFilter:
    def __init__(self):
        self.mx_timestep = 30
        self.dt = 0.1
        self.ex_mean = 0.0
        self.ex_sigma = 0.1
        self.ex = np.array([ [0.0], [0.0] ])
        self.u = np.array([ [2.0*math.cos(math.radians(45))*self.dt] , [2.0*math.sin(math.radians(45))*self.dt] ])
        self.xk_1 = np.array([ [0.0] , [0.0] ])
        self.estimation_xk_1 = np.array([ [0.0] , [0.0] ])
        self.belief = multivariate_normal(mean=np.array([0.0, 0.0]), cov=np.diag([1e-10, 1e-10]))
    
    def system(self, xk_1):
        self.ex[0] = np.random.normal(self.ex_mean, self.ex_sigma)
        self.ex[1] = np.random.normal(self.ex_mean, self.ex_sigma)
        xk =  xk_1 + self.u + self.ex
        return xk

    def sigma_ellipse(self, p, cov, n):  ###kf3calculation
        eig_vals, eig_vec = np.linalg.eig(cov)
        ang = math.atan2(eig_vec[:,0][1], eig_vec[:,0][0])/math.pi*180
        return Ellipse(p, width=2*n*math.sqrt(eig_vals[0]),height=2*n*math.sqrt(eig_vals[1]), angle=ang, fill=False, color="blue", alpha=0.5)

    def matM(self):
        return  np.array([ [0.1, 0.0], [0.0, 0.1] ])
    
    def motion_update(self):
        self.belief.mean = self.estimation_xk_1 + self.u
        self.belief.cov = self.belief.cov + self.matM()
        self.estimation_xk_1 = self.belief.mean
    
    def one_step(self, i, elems, ax1):
        while elems: elems.pop().remove()
        xk_sys = self.system(self.xk_1) ## 状態方程式で解いた現在のx, y
        self.xk_1 = xk_sys    
        elems += ax1.plot(xk_sys[0], xk_sys[1], "blue", marker = 'o', markersize = 10)
        ## 推定したロボットの動き
        self.motion_update()
        elems += ax1.plot(self.belief.mean[0], self.belief.mean[1], "green", marker = 'o', markersize = 10)
    
    def draw(self):
        fig = plt.figure(figsize=(4,4))     #10〜16行目はそのまま
        ax = fig.add_subplot(111)
        ax.set_aspect('equal')
        ax.set_xlim(0,10)
        ax.set_ylim(0,10)
        ax.set_xlabel("X",fontsize=10)
        ax.set_ylabel("Y",fontsize=10)
        
        elems = []
        
        self.ani = anm.FuncAnimation(fig, self.one_step, fargs=(elems, ax), frames=51, interval=100, repeat=False)    
        plt.show()

    # def main(self):
    #     # グラフ作成
    #     fig = plt.figure()
    #     ax1 = fig.add_subplot(1, 1, 1)
    #     # 直線描画
    #     ax1.set_xlim([0, 10])
    #     ax1.set_ylim([0, 10])
    #     # トロッコ描画
    #     ims = []
        # timestep = 0
        # while(timestep < self.mx_timestep):
        #     timestep += 1
        #     ## 実際のロボットの動き
        #     xk_sys = self.system(self.xk_1) ## 状態方程式で解いた現在のx, y
        #     self.xk_1 = xk_sys    
        #     im1 = ax1.plot(xk_sys[0], xk_sys[1], "blue", marker = 'o', markersize = 10)

        #     ## 推定したロボットの動き
        #     self.motion_update()
        #     im2 = ax1.plot(self.belief.mean[0], self.belief.mean[1], "green", marker = 'o', markersize = 10)

        #     ## 誤差楕円の描画
        #     e = self.sigma_ellipse(self.belief.mean, self.belief.cov, 1)
        #     im3 = ax1.add_patch(e)
        #     ims.append(im1+im2)

        # ani = animation.FuncAnimation(fig, self.one_step, ims, interval=300)
        # plt.show()
        

if __name__ == "__main__":
    kalman = KalmanFilter()
    kalman.main()
