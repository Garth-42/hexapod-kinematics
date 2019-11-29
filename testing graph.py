# 3D Graphing

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from math import sqrt


def circle(r, z, h = 0, k = 0, num_points = 200):
    """
    Returns a dictionary containing the points needed to graph a circle
    """

    z_vals = np.full(num_points, z)
    x_vals = np.linspace(h-r, h+r, num_points)
    r_array = np.full(num_points, r)
    h_array = np.full(num_points, h)
    k_array = np.full(num_points, k)

    y_pos_vals = np.sqrt( np.square(r_array) - np.square(x_vals-h_array) ) + k_array
    y_neg_vals=-np.sqrt( np.square(r_array) - np.square(x_vals-h_array) ) + k_array
    
    return [x_vals], [y_pos_vals], [y_neg_vals], [z_vals]

def main():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    xs, pos_ys, neg_ys, zs = circle(r = 10, z = 0, h = 7, k = 8)

    # Graph the circle
    for x, pos_y, neg_y, z in zip(xs, pos_ys, neg_ys, zs):
        ax.plot(x, pos_y, z)
        ax.plot(x, neg_y, z)

    plt.show()

if __name__ == "__main__":
    main()