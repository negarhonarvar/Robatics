import numpy as np
import math
import matplotlib.pyplot as plt

step = 0.01
n = int(input())
seq = math.pi*2 / n
beta = math.pi * 2 * (-1)
ans_x = []
ans_y = []

for i in range(n):
        
    p = 50
    alpha = math.pi / 2
    k_p = 3
    k_alpha = 8
    k_beta = -1.5

    plt_walk_x = np.array([])
    plt_walk_y = np.array([])

    p_round = -k_p * p
    alpha_round = -k_alpha * alpha + k_p * alpha - k_beta * beta
    beta_round = -k_p * alpha

    tmp_beta = beta
    while p > 0:
        plt_walk_x = np.append(plt_walk_x, p * math.sin(tmp_beta))
        plt_walk_y = np.append(plt_walk_y, p * math.cos(tmp_beta))
        p += p_round * step
        alpha += alpha_round * step
        tmp_beta += beta_round * step

    ans_x = np.append(ans_x, plt_walk_x)
    ans_y = np.append(ans_y, plt_walk_y)
    beta += seq


plt.plot(ans_x, ans_y)
plt.show()
