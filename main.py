from collections import deque
import math
import os
import random
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import ArtistAnimation
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d import Axes3D
import networkx as nx
import numpy as np
import scipy.stats as st
import seaborn as sns

import GossipAveraging as GAV

if __name__ == '__main__':
    args = sys.argv
    sns.set()
    if len(args) == 4:
        n_list = np.arange(args[1], args[2], args[3])
    else:
        n_list = np.arange(400, 2001, 200)
    def rads(x): return math.sqrt(6*math.log(x)/x)

    lambda_val = 0.002
    ret = [[], []]
    for n in n_list:
        print('n: {}'.format(n))
        emp_list = [[], []]
        for i in range(10):
            model = GAV.GossipGraph(n, rads, lambda_val)
            for j in range(4):
                np.random.seed(n-i-j)
                random.seed(n+i+j)
                rets_o = model.oneWayAveraging(t_1=750)
                np.random.seed(n-i-j)
                random.seed(n+i+j)
                rets_p = model.pathAveraging(t_1=750)
                emp_list[0].append(calc_C_emp(model.x_ave, rets_o[1], rets_o[2]))
                emp_list[1].append(calc_C_emp(model.x_ave, rets_p[1], rets_p[2]))
        ret[0].append(np.mean(emp_list[0]))
        ret[1].append(np.mean(emp_list[1]))
        print('ret: {}'.format([np.mean(emp_list[0]), np.mean(emp_list[1])]))

    plt.plot(n_list, ret[0])
    plt.plot(n_list, ret[1])

    c_emp_list_o, c_emp_list_p = ret

    plt.figure(figsize=(10, 6))
    plt.xlabel('node size n')
    plt.ylabel('C_emp')
    plt.plot(n_list, c_emp_list_o, marker='o', label='one-way')
    plt.plot(n_list, c_emp_list_p, marker='o', label='path')
    legend = plt.legend(frameon=1)
    frame = legend.get_frame()
    frame.set_color('white')
    plt.savefig('./c_emp.png')
