# -*- coding: utf-8 -*-
#!/usr/bin/env python
import numpy as np
import numpy.random as random
import matplotlib.pyplot as plt
import math
# 関数
def model(x, A, B, u):
    v = 2.0
    return np.dot(A, x) + B * v

def true(x):
    noise = random.normal(0.0, 2.0, (2, 1))
    return x + noise

def system(x, A, B, u):
    true_val = true(model(x, A, B, u))
    obs_val = true(true_val)
    return true_val, obs_val

def Kalman_Filter(m, V, y, A, B, u, Q, R):
    # 予測
    m_est = model(x, A, B, u)
    V_est = np.dot(np.dot(A, V), A.transpose()) + Q
    
    # 観測更新
    K = np.dot(V_est, np.linalg.inv(V_est + R))
    m_next = m_est + np.dot(K, (y - m_est))
    V_next = np.dot((np.identity(2) - K), V_est)
    
    return m_next, V_next

def get_distance(x1, y1, x2, y2):
        d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return d

if __name__ == "__main__":
    A = np.identity(2) # 状態方程式のA
    B = np.ones((2, 1)) # 状態方程式のB
    u = 1.4 # 速度
    x = np.zeros((2, 1)) # 真値
    m = np.zeros((2, 1)) # 推定値
    V = np.identity(2) # 推定値の初期共分散行列(勝手に設定して良い)
    Q = 4 * np.identity(2) # 入力誤差の共分散行列(今回はtrue()の中でnoiseの分散を2.0に設定したので)
    R = 4 * np.identity(2) # 上と同じ
    error_real = error_est = 0.0
    
    # 記録用
    rec = np.empty((4, 51))
    
    # main loop
    for i in range(51):
        x, y = system(x, A, B, u)
        m, V = Kalman_Filter(m, V, y, A, B, u, Q, R)
        error_real += get_distance(x[0], x[1],  y[0], y[1])
        error_est += get_distance(x[0], x[1],  m[0], m[1])
        
        rec[0, i] = x[0]
        rec[1, i] = x[1]
        rec[2, i] = m[0]
        rec[3, i] = m[1]
        print("観測値の誤差: " , error_real, "推定値の誤差: ", error_est)
        
    # 描画
    plt.plot(rec[0, :], rec[1, :], color="blue", marker="o", label="true")
    plt.plot(rec[2, :], rec[3, :], color="red", marker="^", label="estimated")
    plt.legend()
    plt.show()