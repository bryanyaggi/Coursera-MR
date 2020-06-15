#!/usr/bin/python3

import numpy as np
import modern_robotics as mr
import math

'''
Course 2 Project
'''

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev,
        filename='iterates.csv'):
    """
    Computes inverse kinematics in the body frame for an open chain robot
    outputting information at each iteration.
    
    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :param filename: The CSV filename to which joint value for each iteration
                     are output. Default is 'iterates.csv'.
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances,
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.

    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.
    
    Example Input:
        Blist = np.array([[0, 0, -1, 2, 0,   0],
                          [0, 0,  0, 0, 1,   0],
                          [0, 0,  1, 0, 0, 0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        eomg = 0.01
        ev = 0.001

    Output:
        (np.array([1.57073819, 2.999667, 3.14153913]), True)

    Writes information about each iteration to console and writes joint values
    for each iteration to CSV file.
    """

    '''
    Prints iteration information to console.
    '''
    def printIterationSummary(i, thetalist, Tb, Vb, errAng, errLin):
        print('Iteration %d' %i)
        print('joint vector: %s' %thetalist)
        print('end effector configuration:')
        print('%s' %Tb)
        print('twist error: %s' %Vb)
        print('angular error magnitude: %s' %errAng)
        print('linear error magnitude: %s' %errLin)
        print('')

    '''
    Writes iteration joint values to CSV file.
    '''
    def writeJointValues(f, thetalist):
        for i in range(len(thetalist)):
            f.write('%s' %thetalist[i])
            if i < len(thetalist) - 1:
                f.write(',')
        f.write('\n')

    f = open(filename, 'w')

    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    err = True
    Vb = np.zeros(6) # twist error
    while err and i < maxiterations:
        thetalist = thetalist + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, thetalist)), Vb)
        Tb = mr.FKinBody(M, Blist, thetalist) # current end effector configuration
        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tb), T)))
        errAng = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) # twist angular error
        errLin = np.linalg.norm([Vb[3], Vb[4], Vb[5]]) # twist linear error
        err = errAng > eomg or errLin > ev
        printIterationSummary(i, thetalist, Tb, Vb, errAng, errLin)
        writeJointValues(f, thetalist)
        i += 1
    f.close()

    return (thetalist, not err)

def test():
    Blist = np.array([[0, 0, -1, 2, 0,   0],
                      [0, 0,  0, 0, 1,   0],
                      [0, 0,  1, 0, 0, 0.1]]).T
    M = np.array([[-1, 0,  0, 0],
                  [ 0, 1,  0, 6],
                  [ 0, 0, -1, 2],
                  [ 0, 0,  0, 1]])
    T = np.array([[0, 1,  0,     -5],
                  [1, 0,  0,      4],
                  [0, 0, -1, 1.6858],
                  [0, 0,  0,      1]])
    thetalist0 = np.array([1.5, 2.5, 3])
    eomg = 0.01
    ev = 0.001
    thetalist, success = IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)
    
    np.testing.assert_almost_equal(thetalist,
            np.array([1.57073819, 2.999667, 3.14153913]))
    assert(success == True)

def example4_5():
    W_1 = 0.109 # meters
    W_2 = 0.082
    L_1 = 0.425
    L_2 = 0.392
    H_1 = 0.089
    H_2 = 0.095

    M = np.array([[-1, 0, 0, L_1 + L_2],
                  [0, 0, 1, W_1 + W_2],
                  [0, 1, 0, H_1 - H_2],
                  [0, 0, 0, 1]])
    Blist = np.array([[0, 1, 0, W_1 + W_2, 0, L_1 + L_2],
                      [0, 0, 1, H_2, -L_1 - L_2, 0],
                      [0, 0, 1, H_2, -L_2, 0],
                      [0, 0, 1, H_2, 0, 0],
                      [0, -1, 0, -W_2, 0, 0],
                      [0, 0, 1, 0, 0, 0]]).T
    T = np.array([[0, 1, 0, -0.5],
                  [0, 0, -1, 0.1],
                  [-1, 0, 0, 0.1],
                  [0, 0, 0, 1]])
    eomg = 0.001
    ev = 0.0001
    #thetalist0 = np.array([0, 5*math.pi/4, 3*math.pi/2, 3*math.pi/4, math.pi, 3*math.pi/2])
    #thetalist0 = np.array([-0.1, 4.2, 4.6, 3.9, 3.0, 4.7])
    thetalist0 = np.array([0, 4, 5, 4, 3, 5])
    
    IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)

if __name__ == '__main__':
    example4_5()
