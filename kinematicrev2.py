# This code contains the kinematic calculations to move a hexapod milling or additive machine.
# equations from: https://www.janssenprecisionengineering.com/page/hexapod-kinematics/
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from numpy import linalg as LA
from math import cos, sin, radians    
from itertools import repeat

### Machine Definition ###

# Angle between hinge locations of a leg-pair
alpha_p = radians(110)  # deg to rad, platform
alpha_b = radians(110)  # deg to rad, base

# Pitch circle radius
R_p = 40   # cm, Radius of the circle on which the platform hinges are located
R_b = 200  # cm, radius of the circle on which the base hinges are located

def circle(r, z, h = 0, k = 0, num_points = 200):
    #https://stackoverflow.com/questions/56870675/how-to-do-a-3d-circle-in-matplotlib
    """
    Returns a list containing points to graph a circle
    """
    theta = np.linspace(0, 2*np.pi, num_points)
    x = r * np.cos(theta)+h
    y = r * np.sin(theta)+k
    z= np.full(num_points, z)

    return [x, y, z]

def P_nom(R, theta, Z):
    """
    Input:

    R: pitch circle radius
    theta: angle????
    Z: relative height with resp to machine origin

    Output: Nominal position of hinges (base or platform)
    """
    return np.array( [R*cos(theta), R*sin(theta), Z] )

def thetas(alpha): # in radians, out radians
    """
    Is this the angle between the platform/base and the leg?
    Input: angle between hinge locations. platform/Base
    Output: theta for all six legs?
    """
    return [ radians(270) - alpha/2, radians(270) + alpha/2, radians(30) - alpha/2, radians(30) + alpha/2, radians(150) - alpha/2, radians(150) + alpha/2 ]

def dP(dx, dy, dz, dRx, dRy, dRz, M, P_nom):
    """

    """
    def Q(dRx, dRy, dRz):
        # Rotation matrix
        # use local variables in the func?
        A = np.array( [ [ 1, 0, 0 ], [ 0, cos(dRx), -sin(dRx) ], [ 0, sin(dRx), cos(dRx) ] ] )
        B = np.array( [ [ cos(dRy), 0, sin(dRy) ], [ 0, 1, 0 ], [ -sin(dRy), 0, cos(dRy) ] ] )
        C = np.array( [ [ cos(dRz), -sin(dRz), 0 ], [ sin(dRz), cos(dRz), 0 ], [ 0, 0, 1 ] ] )
        return A * B * C
    return np.array([dx, dy, dz]) + np.matmul( Q(dRx, dRy, dRz) -  np.identity(3), (P_nom - M) )

def s(P_delta_p, P_nom_b, P_nom_p):
    """
    Calculates actuator lengths after a differential move
    """
    s = []
    for P_delta, P_b, P_p, in zip(P_delta_p, P_nom_b, P_nom_p):
        s.append( LA.norm(P_delta - P_b) - LA.norm(P_p - P_b) ) # Norm calculates the magnitude of the vectors which is the link length?
    return s

def main():

    # Kinematic Calc Inputs
    # location before the differential move (time = t_0)
    M = np.array([0,0,0])
    Z_p = M[2]
    
    # Differential Move
    dx, dy, dz =    10, 10, 10
    dRx, dRy, dRz = radians(49), radians(0), radians(0) # Rotation isn't working

    # Calculate arrays containing the theta values
    thetas_b = np.array( thetas(alpha_b) )
    thetas_p = np.array( thetas(alpha_p) )

    # Calculate nominal hinge location
    P_nom_p = np.array([P_nom(R_p, theta, Z_p) for theta in thetas_p])
    P_nom_b = np.array([P_nom(R_b, theta, 0) for theta in thetas_b])
    
    # Calculate position differential for the platform hinges
    dP_p = [dP(dx, dy, dz, dRx, dRy, dRz, M, P) for P in P_nom_p]

    # New platform hinge locs
    P_delta_p = P_nom_p + dP_p

    # Plot the platform
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Nominal Position
    for base, platform in zip(P_nom_b, P_nom_p):
        ax.plot([base[0],platform[0]], [base[1], platform[1]], [base[2], platform[2]])
    x, y, z = circle(h = M[0],k =  M[1], z = M[2], r = R_p) # Plot the platform
    ax.plot(x, y,z)

    # New Position
    for base, platform in zip(P_nom_b, P_delta_p):
        ax.plot([base[0],platform[0]], [base[1], platform[1]], [base[2], platform[2]])
    x, y, z = circle(h = M[0] + dx,k = M[1] + dy,z = M[2] + dz, r = R_p)
    ax.plot(x,y, z)
    x, y, z = circle(r = R_b, z = 0) # Plot the base
    ax.plot(x, y, z)

    plt.show()

if __name__ == "__main__":
    main()