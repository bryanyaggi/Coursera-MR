#!/usr/bin/python3

import numpy as np
import modern_robotics as mr
import math

'''
Course 2, Week 3 Quiz
'''

def p1():
    print('P1')

    '''
    Function
    '''
    def f(x, y):
        return np.array([[x ** 2 - 9],
                         [y ** 2 - 4]])
    '''
    Jacobian
    '''
    def J(x, y):
        return np.array([[2 * x, 0],
                         [0, 2 * y]])

    theta = np.array([[1],
                      [1]])
    for i in range(2):
        theta = theta - np.dot(np.linalg.inv(J(theta[0, 0], theta[1, 0])),
                f(theta[0, 0], theta[1, 0]))
    print(theta)

def p2():
    print('P2')
    L = 1
    Slist = np.array([[0, 0, 1, 0, 0, 0],
                      [0, 0, 1, 0, -L, 0],
                      [0, 0, 1, 0, -2*L, 0]]).T # joint screw axes
    Blist = np.array([[0, 0, 1, 0, 3*L, 0],
                      [0, 0, 1, 0, 2*L, 0],
                      [0, 0, 1, 0, L, 0]]).T # joint screw axes
    M = np.array([[1, 0, 0, 3*L],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]]) # home configuration
    T = np.array([[-0.585, -0.811, 0, 0.076],
                    [0.811, -0.585, 0, 2.608],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]]) # desired configuration
    thetalist0 = np.array([math.pi/4, math.pi/4, math.pi/4]) # initial guess
    eomg = 0.001 # orientation error in radians
    ev = 0.0001 # linear error in meters

    thetalist, success = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
    print('Space Inverse Kinematics')
    print('success: %s' %success)
    print('thetalist: %s' %thetalist)
    
    thetalist, success = mr.IKinBody(Blist, M, T, thetalist0, eomg, ev)
    print('Body Inverse Kinematics')
    print('success: %s' %success)
    print('thetalist: %s' %thetalist)

def printDivider():
    print('-' * 80)

if __name__ == '__main__':
    p1()
    printDivider()
    p2()
