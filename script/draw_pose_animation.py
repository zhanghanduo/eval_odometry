#!/usr/bin/python

"""
This script plots a 3D trajectory.
"""

# import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as p3
# from matplotlib.colors import cnames
from numpy import *
import scipy
import sys
import os

def update_lines(num, pos_est, lines):
    # for line, data in zip(lines, dataLines):
        # NOTE: there is no .set_data() for 3 dim data...
    x ,y, z = pos_est[:num].T
    lines.set_data(x, z)
    # print(x)
    lines.set_3d_properties(y)
    return lines

if __name__ == "__main__":

    # if len(sys.argv) == 2 :
    #     filename = sys.argv[1]
    # else:
    # print "Usage: python draw_results.py [filename]"
    # exit(0)
    # trajectory = file('/home/zh/Dataset/data/FrameTrajectory_1132_noimu.txt')
    trajectory = file('/home/zh/catkin_ws/src/ugv_slam/testbag/Loop/Key_Mar_24_17_18_39.txt')
    # file = open(filename, 'r')
    # file.readline()
    results = array([map(float, line.split(' ')) for line in trajectory if line.strip() !=""])
    pos_est = results[:,1:4]
    # rot_est = results[:,4:8]
    frame_num = len(pos_est)

    fig = plt.figure()
    ax = p3.Axes3D(fig)

    # Creating fifty line objects.
    # NOTE: Can't pass empty arrays into 3d version of plot()
    lines = ax.plot(pos_est[0, 0:1], pos_est[1, 0:1], pos_est[2, 0:1])[0]
    # print pos_est

    # Setting the axes properties
    ax.set_xlim3d([-5, 55])
    ax.set_xlabel('X')

    ax.set_ylim3d([-45, 45])
    ax.set_ylabel('Y')

    ax.set_zlim3d([-15, 20])
    ax.set_zlabel('Z')

    ax.set_title('3D Test')

    # Creating the Animation object
    line_ani = animation.FuncAnimation(fig, update_lines, frame_num, fargs=(pos_est, lines),
                                       interval=10, blit=False)

    plt.show()
