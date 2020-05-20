#!/usr/bin/python3

import numpy as np
import modern_robotics as mr

'''
Course 1, Week 3 Quiz
'''

def p9():
    print('P9')
    omega = np.array([1, 2, 0])
    so3mat = mr.VecToso3(omega)
    result = mr.MatrixExp3(so3mat)
    print(result)

def p10():
    print('P10')
    omega = np.array([1, 2, .5])
    result = mr.VecToso3(omega)
    print(result)

def p11():
    print('P11')
    so3mat = np.array([[0, .5, -1],
                       [-.5, 0, 2],
                       [1, -2, 0]])
    result = mr.MatrixExp3(so3mat)
    print(result)

def p12():
    print('P12')
    R = np.array([[0, 0, 1],
                  [-1, 0, 0],
                  [0, -1, 0]])
    result = mr.MatrixLog3(R)
    print(result)

if __name__ == '__main__':
    p9()
    p10()
    p11()
    p12()
