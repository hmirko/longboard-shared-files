#!/usr/bin/env python
import rospy
from time import sleep
from skateboard.msg import Plot
import numpy as np
import matplotlib.pyplot as plt

def redraw():
    global state

    plt.clf()
    ax = plt.gca()
    ax.set_ylim(bottom = 0, top = 1024)
    ax.set_xlim(left = 0, right = 1024)
    ax.autoscale_view()
    x = np.linspace(0, 1024, 1024)
    brk = lambda x: state.brk_a * x**2 + state.brk_b * x + state.brk_c
    fx_brk = brk(x)
    print(fx_brk)
    fx_brk[512:] = np.nan

    #acc = lambda x: state.acc_a * x*x + state.acc_b * x + state.acc_c
    acc = lambda x: state.acc_a * (1024 - x) ** 2 + state.acc_b * (1024 - x) + state.acc_c
    fx_acc = acc(x)
    print(fx_acc)
    fx_acc[:511] = np.nan

    plt.style.use('classic')
    plt.plot(fx_acc, color='r')
    plt.plot(fx_brk, color='b')
#    plt.show(block=False)
    plt.draw

def callback(data):
    global redraw_required
    global state

    redraw_required = True
    state = data

if __name__ == '__main__':
    plt.ion()
    plt.clf()
    global redraw_required
    redraw_required = False

    rospy.init_node('plot_subscriber')

    rospy.Subscriber("/skateboard/Plot", Plot, callback)

    while(True):
        sleep(1)
        if redraw_required:
                redraw()
                plt.pause(.5)
                redraw_required = False
