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
import matplotlib.patches as patches


# 単位行列
I = np.matrix([[1 , 0],
               [0, 1]])

class World:
    def __init__(self):
        self.objects = []
    
    def append(self,obj): 
        self.objects.append(obj)
    
    # 描画用
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

    # アニメーション用
    def one_step(self, i, elems, ax):
        while elems: elems.pop().remove()
        elems.append(ax.text(1.0, 8.0, "t = "+str(round(i*0.1, 1)), fontsize=10)) 
        for obj in self.objects: # 追加
            obj.draw(ax, elems)  # 追加
            if hasattr(obj, "one_step"): obj.one_step(0.1)     # 時間を経過させる
        return

class IdealRobot:
    def __init__(self, pose, ex_mean, ex_sigma, color="black"):
        self.pose = pose
        self.r = 0.2
        self.color = color
        self.ex_mean = ex_mean
        self.ex_sigma = ex_sigma
    
    def draw(self, ax, elems):                ### fig:append_elements (7-13行目)
        x, y = self.pose
        theta = math.radians(45)
        xn = x + self.r * math.cos(theta)
        yn = y + self.r * math.sin(theta)
        elems += ax.plot([x,xn], [y,yn], color=self.color) # elems += を追加
        c = patches.Circle(xy=(x, y), radius=self.r, fill=False, color=self.color) # c = を追加
        elems.append(ax.add_patch(c))   # elem.appendで包む
    
    @classmethod
    def state_transition(cls, u, time, pose, ex): 
        return pose + u*time + ex

    def one_step(self, time_interval):
        u = np.array( [2.0*math.cos(math.radians(45)) , 2.0*math.sin(math.radians(45)) ]).T
        ex = np.array( [np.random.normal(self.ex_mean, self.ex_sigma) , np.random.normal(self.ex_mean, self.ex_sigma) ]).T
        self.pose = self.state_transition(u, time_interval, self.pose, ex)
        return

class Kalmanfilter:
    def __init__(self, pose, ex_mean, ex_sigma, color="black"):
        self.pose = pose
        self.r = 0.2
        self.color = color
        self.ex_mean = ex_mean
        self.ex_sigma = ex_sigma
        self.belief = multivariate_normal(mean=np.array([0.0, 0.0]), cov=np.diag([1e-10, 1e-10]))
    
    def draw(self, ax, elems):                ### fig:append_elements (7-13行目)
        x, y = self.belief.mean
        theta = math.radians(45)
        xn = x + self.r * math.cos(theta)
        yn = y + self.r * math.sin(theta)
        elems += ax.plot([x,xn], [y,yn], color=self.color) # elems += を追加
        c = patches.Circle(xy=(x, y), radius=self.r, fill=False, color=self.color) # c = を追加
        e = self.sigma_ellipse(self.belief.mean, self.belief.cov, 1)
        elems.append(ax.add_patch(c))   # elem.appendで包む
        elems.append(ax.add_patch(e))   # elem.appendで包む
    
    def sigma_ellipse(self, p, cov, n):  ###kf3calculation
        eig_vals, eig_vec = np.linalg.eig(cov)
        ang = math.atan2(eig_vec[:,0][1], eig_vec[:,0][0])/math.pi*180
        return Ellipse(p, width=2*n*math.sqrt(eig_vals[0]),height=2*n*math.sqrt(eig_vals[1]), angle=ang, fill=False, color="blue", alpha=0.5)

    @classmethod
    def state_transition(cls, u, time, pose, ex): 
        return pose + u*time + ex

    def matM(self):
        return  np.array([ [0.1, 0.0], [0.0, 0.1] ])

    def one_step(self, time_interval):
        u = np.array( [2.0*math.cos(math.radians(45)) , 2.0*math.sin(math.radians(45)) ]).T
        ex = np.array( [np.random.normal(self.ex_mean, self.ex_sigma) , np.random.normal(self.ex_mean, self.ex_sigma) ]).T
        self.pose = self.state_transition(u, time_interval, self.pose, ex)
        self.belief.mean = self.pose
        self.belief.cov = self.belief.cov + self.matM()

        # 観測方程式(time_interval = 1, 2, 3, 4, ,,)
        return

if __name__ == "__main__":
    world = World()
    robot1 = IdealRobot( np.array([0, 0]).T , 0.0, 0.1) 
    robot2 = Kalmanfilter( np.array([0, 0]).T , 0.0, 0.0, color="blue") 
    world.append(robot1)
    world.append(robot2)
    world.draw()
