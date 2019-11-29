# 3D Graphing

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from math import sqrt


def circle(r, z, h = 0, k = 0, num_points = 200):
    #https://stackoverflow.com/questions/56870675/how-to-do-a-3d-circle-in-matplotlib
    """
    Returns a dictionary containing the points needed to graph a circle
    """
    theta = np.linspace(0, 2*np.pi, num_points)
    x = r * np.cos(theta)+h
    y = r * np.sin(theta)+k
    z= np.full(num_points, z)

    return [x, y, z]

def main():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x, y, z = circle(r = 2, z = 1, h = 5, k = 3)
    ax.plot(x, y, z)

    plt.show()

if __name__ == "__main__":
    main()