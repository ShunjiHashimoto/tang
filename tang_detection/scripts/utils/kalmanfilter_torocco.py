# -*- coding: utf-8 -*-
#!/usr/bin/env python
import numpy as np
import math
import random
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# 単位行列
I = np.matrix([[1 , 0],
               [0, 1]])

class KalmanFilter:
    def __init__(self):
        self.mx_timestep = 50
        self.dt = 0.1
        self.mean_a = 0
        self.sigma_a = 1
        self.mean_z = 0
        self.sigma_z = 1
        # 状態方程式
        self.fk = np.array([ [1, self.dt], [0, 1] ])
        self.gk = np.array([ [self.dt**2/2] , [self.dt] ] )
        self.xk_1 = np.array([ [0.0] , [0.0] ])
        self.wk = 0.0 ## 加速度のノイズ
        # 観測方程式
        self.hk = np.matrix([
                1,
                0
             ])
        self.zk = np.array([ [0.0] , [0.0] ])
        self.vk = 0.0 ## 観測値のノイズ
        # 予測
        self.pk_1 = np.array([ [1, 0], [0, 0] ]) ## 位置の共分散: P
        self.qk   = (self.sigma_a**2) * self.gk * self.gk.T ## 加速度の共分散: Q
        self.rk   = np.array([self.sigma_z**2]) ## 観測値の共分散: R
    
    def system(self, xk_1):
        self.wk = np.random.normal(self.mean_a, self.sigma_a)
        xk = np.dot(self.fk, xk_1) + self.gk*self.wk
        return xk

    def observation(self, xk):
        self.vk = np.random.normal(self.mean_z, self.sigma_z)
        zk = np.dot(self.hk, xk) + self.vk
        return zk
    
    def prediction(self, zk , xk):
        pk = np.dot(np.dot(self.fk, self.pk_1), self.fk.T) + self.qk
        ek = zk - np.dot(self.hk, xk)
        sk = self.rk + np.dot(np.dot(self.hk, pk),self.hk.T)
        kk = pk * self.hk.T/ sk
        xk = xk + kk*ek
        pk = (I - kk * self.hk) * pk
        return xk, pk

    def main(self):
        # グラフ作成
        fig = plt.figure()
        ax1 = fig.add_subplot(1, 1, 1)
        # 直線描画
        ax1.set_xlim([-2, 2])
        ax1.axhline(y=1)
        # トロッコ描画
        ims = []
        timestep = 0
        obs_error = 0
        kalman_error = 0
        while(timestep < self.mx_timestep):
            timestep += 1
            xk_sys = self.system(self.xk_1) ## 状態方程式で解いた現在の位置、速度
            zk_obs = self.observation(xk_sys) ## 観測方程式から得られた現在の位置、速度
            self.xk_1, self.pk_1 = self.prediction(zk_obs, xk_sys) ## 予測・更新から得られた位置、速度
            obs_error += (xk_sys[0] - zk_obs)**2
            kalman_error += (self.xk_1[0] - xk_sys[0])**2
            
            txt1 = ax1.annotate("Torocco", (xk_sys[0], 1.0), (xk_sys[0], 1.01), arrowprops=dict(arrowstyle="->"), fontsize=8, color = "blue")
            txt2 = ax1.annotate("Observed", (zk_obs[0], 1.0), (zk_obs[0], 1.01), arrowprops=dict(arrowstyle="->"), fontsize=8, color = "red")
            txt3 = ax1.annotate("Estimation", (self.xk_1[0], 1.0), (self.xk_1[0], 1.01), arrowprops=dict(arrowstyle="->"), fontsize=8, color = "green")
            loop_time = ax1.text(-1.5, 1.03,"t = "+str(timestep), fontsize = 10)
            im1 = ax1.plot(xk_sys[0], 1.0, "blue", marker = 'o', markersize = 10)
            im2 = ax1.plot(zk_obs[0], 1.0, "red", marker = 'o', markersize = 10)
            im3 = ax1.plot(self.xk_1[0], 1.0, "green", marker = 'o', markersize = 10 )
            ims.append(im1 + im2 + [txt1] + [txt2] + [loop_time] + im3 + [txt3])
            time.sleep(0.1)

        ani = animation.ArtistAnimation(fig, ims, interval=300)
        plt.show()
        print(obs_error, kalman_error)

if __name__ == "__main__":
    kalman = KalmanFilter()
    kalman.main()
