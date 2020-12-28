#!/usr/bin/env python

import numpy as np
import matplotlib
matplotlib.use('GTKAgg')
import matplotlib.pyplot as plt
import time

class Live_plot():
    def __init__(self):
        self.x_vec = [0]
        self.y_vec = [0]
        self.ind = 0
        plt.rc('axes', labelsize=8)
        plt.figure(figsize=(5.25, 3.5))
        plt.plot(self.x_vec, self.y_vec, 'o-')
        plt.subplots_adjust(left=0.12, right=0.95, top=0.95, bottom=0.13)

    def update(self, y):
        self.x_vec.append(self.ind)
        self.y_vec.append(y)
        plt.gca().lines[0].set_xdata(self.x_vec)
        plt.gca().lines[0].set_ydata(self.y_vec)
        plt.gca().relim()
        plt.gca().autoscale_view()
        plt.pause(0.001)
        self.ind += 1

if __name__ == "__main__":
    p = Live_plot()
    for _ in range(100):
        y = np.random.random()
        s = time.time()
        p.update(y)
        print(time.time()-s)
    plt.show()