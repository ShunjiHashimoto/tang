# -*- coding: utf-8 -*-
#!/usr/bin/env python

import numpy as np
import numpy.random as random
import matplotlib.pyplot as plt

# 関数
def state_eq(x, L, omega, t):
    x_new = x + np.array([-L*omega*np.sin(omega*t),
                          L*omega*np.cos(omega*t)]).reshape([2, 1])
    print(t)
    return x_new
    
def obs_eq(x, obs_noise_y1, obs_noise_y2):
    x1 = x[0]
    x2 = x[1]
    y = np.array([np.sqrt(x1**2 + x2**2),
                  np.arctan(x2/x1)]).reshape([2, 1])
    noise = np.array([random.normal(0, obs_noise_y1), 
                      random.normal(0, obs_noise_y2)]).reshape([2, 1])
    return y + noise
    
def obs_eq_noiseless(x):
    x1 = x[0]
    x2 = x[1]
    y = np.array([np.sqrt(x1**2 + x2**2),
                  np.arctan(x2/x1)]).reshape([2, 1])
    return y
    
def true(x, input_noise):
    noise = random.normal(0, input_noise, (2, 1))
    return x + noise

def state_jacobian():
    jacobian = np.identity(2)
    return jacobian
    
def obs_jacobian(x):
    jacobian = np.empty((2, 2))
    jacobian[0][0] = x[0] / np.sqrt(x[0]**2 + x[1]**2)
    jacobian[0][1] = x[1] / np.sqrt(x[0]**2 + x[1]**2)
    jacobian[1][0] = -x[1] / (x[0]**2 + x[1]**2)
    jacobian[1][1] = x[0] / (x[0]**2 + x[1]**2)
    
    return jacobian
    
def system(x, L, omega, t, input_noise, obs_noise_y1, obs_noise_y2):
    true_state = true(state_eq(x, L, omega, t), input_noise)
    obs = obs_eq(true_state, obs_noise_y1, obs_noise_y2)
    return true_state, obs
    
def EKF(m, V, y, Q, R, L, omega, t):
    # 予測ステップ
    m_est = state_eq(m, L, omega, t)
    A = state_jacobian()
    V_est = np.dot(np.dot(A, V), A.transpose()) + Q
    
    # 観測更新ステップ
    C = obs_jacobian(m_est)
    temp = np.dot(np.dot(C, V_est), C.transpose()) + R
    K = np.dot(np.dot(V_est, C.transpose()), np.linalg.inv(temp))
    m_next = m_est + np.dot(K,(y - obs_eq_noiseless(m_est)))
    V_next = np.dot(np.identity(V_est.shape[0]) - np.dot(K, C), V_est)
    
    return m_next, V_next

if __name__ == "__main__":
    x = np.array([100, 0]).reshape([2, 1])
    L = 100
    omega = np.pi / 10
    input_noise = 1.0**2
    obs_noise_y1 = 10.0**2
    obs_noise_y2 = (5.0 * np.pi / 180)**2
    m = np.array([100, 0]).reshape([2, 1])
    t = 0.0
    dt = 1.0
    V = np.identity(2) * 1.0**2
    Q = np.identity(2) * input_noise
    R = np.array([[obs_noise_y1, 0],
                  [0, obs_noise_y2]])
                  
    # 記録用
    rec = np.empty([4, 21])
    
    for i in range(21):
        rec[0, i] = x[0]
        rec[1, i] = x[1]
        rec[2, i] = m[0]
        rec[3, i] = m[1]
        
        x, y = system(x, L, omega, t, input_noise, obs_noise_y1, obs_noise_y2)
        m, V = EKF(m, V, y, Q, R, L, omega, t)
        
        t += dt
        
    plt.plot(rec[0, :], rec[1, :], color="blue", marker="o", label="true")
    plt.plot(rec[2, :], rec[3, :], color="red", marker="^", label="estimated")
    plt.legend()
    plt.show()