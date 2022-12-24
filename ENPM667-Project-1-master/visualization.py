"""visualization.py - visualizes data generated during main simulation loop"""

from mpl_toolkits import mplot3d
import numpy
import matplotlib.pyplot as plt

figure, axis = plt.subplots(2, 1)
axes = plt.axes(projection="3d")
axes.set_xlabel("X")
axes.set_ylabel("Y")
axes.set_zlabel("Z")
axes.view_init(15, 45)

def plot_desired_point(point):
    axes.scatter(point[0, 0], point[1, 0], point[2, 0], c="blue", s=4)

def plot_actual_point(point):
    axes.scatter(point[0, 0], point[1, 0], point[2, 0], c="red", s=4)

def show_plot():
    plt.show()

def plot_pos_error(error, t):
    axis[0].scatter(t, error, c="blue", s=2)
    axis[0].set_title("Positional Error")


def plot_rot_error(error, t):
    axis[1].scatter(t, error, c="blue", s=2)
    axis[1].set_title("Rotational Error")
