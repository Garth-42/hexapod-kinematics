# This code contains the kinematic calculations to move a hexapod milling or additive machine.
# equations from: https://www.janssenprecisionengineering.com/page/hexapod-kinematics/

import numpy as np

from math import cos, sin, radians
    
# Machine Definition
# Angle between hinge locations of a leg-pair
alpha_p = radians(110)  # degress to radian, platform
alpha_b = radians(110)  # deg to rad, base

# Pitch circle radius
R_p = 40   # cm
R_b = 200  # cm

# Hinge locations
# Not needed?
#P_p = [1, 2, 3]
#P_b = [3, 4, 5]
    
def theta(alpha): # in radians, out radians
    """
    Is this the angle between the platform/base and the leg?
    Input: angle between hinge locations. platform/Base
    Output: theta for all six legs?
    """
    return [ radians(270) - alpha/2, radians(270) + alpha/2, radians(30) - alpha/2, radians(30) + alpha/2, radians(150) - alpha/2, radians(150) + alpha/2 ]

def P(R, theta, Z):
    """
    Calculates hinge position
    
    Inputs:
    R: float, pitch circle radius (platform or base)
    theta: list of floats, some angle for all six legs. This is a list containing an angle for each leg.
    Z = float, Z-height of platform from base (if calc'ing P for base, = 0 )

    Outputs:
    P: list of lists of floats, X, Y, Z for each leg's hinge platform/base
    """
    
    P = []
    
    for angle in theta:
        P.append( [ R * cos( angle ), R * sin( angle ), Z ] )

    return P

def dP_pi(dx, dy, dz, Q, M)
"""
Input:
dx, dy, dz: float, translation of M from time t_0 to time t
Q: 3 x 3 rotation matrix of M from time t_0 to time t
M: desired postion at time t
P_p: hinge location at time t_0?

Output:
dP_pi: 6 x 3 array of displacement of platform hinge positions from time t_0 to time t
"""
pass

def P_delta_pi():
    """
    P_delta_pi is the displaced platform hinge locations as 6 x 3 matrix
    """
    pass

def s_i(dM):
    """
    actuator displacement each leg
    dM: 3 x 3 displacement, 3 x 3 rotation matrices as a list? of M from time t_0 to time t
    """
    pass

def Q(theta_x, theta_y, theta_z):
    """
    Input: rotation of platform from time t_0 to t in x, y, z (theta_x, ...)
    Output: numpy 3 x 3 array
    """
    # will need to make these actual matrices in numpy...
    dRx = np.array( [ [ 1, 0, 0 ], [ 0, cos(theta_x), -sin(theta_x) ], [ 0, sin(theta_x), cos(theta_x) ] ] )
    dRy = np.array( [ [ cos(theta_y), 0, sin(theta_y) ], [ 0, 1, 0 ], [ -sin(theta_y), 0, cos(theta_y) ] ] )
    dRz = np.array( [ [ cos(theta_z), -sin(theta_z), 0 ], [ sin(theta_z), cos(theta_z), 0 ], [ 0, 0, 1 ] ] )
    return dRx * dRy * dRz # NOT SURE IF THIS REALLY IS DRX * DRY, ETC AS THAT IS THE INPUT TO THIS fcn

def T_inv():
    pass
    
def main():
    
    theta_p = theta(alpha_p)

    print(theta_p)
    
    P_p = P(10, theta_p, 10)

    print(P_p)

if __name__ == "__main__":
    main()