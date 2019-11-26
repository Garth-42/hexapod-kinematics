# This code contains the kinematic calculations to move a hexapod milling or additive machine.
# equations from: https://www.janssenprecisionengineering.com/page/hexapod-kinematics/

import numpy as np
from numpy import linalg as LA

from math import cos, sin, radians    

from itertools import repeat

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
    def Q():
        # Rotation matrix
        # use local variables in the func?
        A = np.array( [ [ 1, 0, 0 ], [ 0, cos(dRx), -sin(dRx) ], [ 0, sin(dRx), cos(dRx) ] ] )
        B = np.array( [ [ cos(dRy), 0, sin(dRy) ], [ 0, 1, 0 ], [ -sin(dRy), 0, cos(dRy) ] ] )
        C = np.array( [ [ cos(dRz), -sin(dRz), 0 ], [ sin(dRz), cos(dRz), 0 ], [ 0, 0, 1 ] ] )
        return A * B * C
    return np.array([dx, dy, dz]) + np.matmul( Q() -  np.identity(3), (P_nom - M) )

def main():
    # Machine Definition
    # Angle between hinge locations of a leg-pair
    alpha_p = radians(110)  # degrees to radian, platform
    alpha_b = radians(110)  # deg to rad, base

    # Pitch circle radius
    R_p = 40   # cm
    R_b = 200  # cm

    # Variable Definitions
    Z_p = 11
    P_nom_p = []
    dx = 1
    dy = 1
    dz = 1
    dRx = 0
    dRy = 0
    dRz = 0
    # "location before the differential move at time t = 0?""
    M = np.array([1,2,4])
    s = []
    
    # Calculate arrays containing the theta values
    thetas_b = np.array( thetas(alpha_b) )
    thetas_p = np.array( thetas(alpha_p) )
    #print(thetas_b)
    #print(thetas_p)

    # Calculate nominal hinge location
    P_nom_p = np.array([P_nom(R_p, theta, Z_p) for theta in thetas_p])
    P_nom_b = np.array([P_nom(R_b, theta, 0) for theta in thetas_b])
    # Equivalent ways of doing the same calc
    #P_nom_p = np.array( list(map( P_nom, repeat(R_p), thetas_p, repeat(Z_p) ) ) )
    #for theta in thetas_p:
    #    P_nom_p.append(P_nom(R_p, theta, Z_p))
    print(P_nom_p)
    # Calculate position differential for the platform hinges
    dP_p = [dP(dx, dy, dz, dRx, dRy, dRz, M, P) for P in P_nom_p]
    print(dP_p)

    # New platform hinge locs
    P_delta_p = P_nom_p + dP_p
    print(P_delta_p)

    # Actuator Displacement each leg
    # take the magnitude of the vector?
    for P_delta, P_b, P_p, in zip(P_delta_p, P_nom_b, P_nom_p):
        s.append( LA.norm(P_delta - P_b) - LA.norm(P_p - P_b) )
    print(s)

if __name__ == "__main__":
    main()