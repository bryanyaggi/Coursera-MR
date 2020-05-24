#!/usr/bin/python3

import numpy as np
import modern_robotics as mr

'''
Course 1, Week 4 Quiz
'''

def p5():
    print('P5')
    pb = np.array([1, 2, 3, 1]) # modified dimension
    Tsb = np.array([[1, 0, 0, 0],
                    [0, 0, 1, 2],
                    [0, -1, 0, 0],
                    [0, 0, 0, 1]])
    result = np.dot(Tsb, pb)
    print(result)

def p7():
    print('P7')
    Vs = np.array([3, 2, 1, -1, -2, -3])
    Tas = np.array([[0, 0, 1, -1],
                    [-1, 0, 0, 0],
                    [0, -1, 0, 0],
                    [0, 0, 0, 1]])
    Tsa = np.array([[0, -1, 0, 0],
                    [0, 0, -1, 0],
                    [1, 0, 0, 1],
                    [0, 0, 0, 1]])
    assert(np.array_equal(Tas, mr.TransInv(Tsa)))
    result = np.dot(mr.Adjoint(Tas), Vs)
    print(result)

def p8():
    print('P8')
    Tsa = np.array([[0, -1, 0, 0],
                    [0, 0, -1, 0],
                    [1, 0, 0, 1],
                    [0, 0, 0, 1]])
    Rsa = Tsa[:3, :3]
    so3mat = mr.MatrixLog3(Rsa)
    expc3 = mr.so3ToVec(so3mat)
    omghat, theta = mr.AxisAng3(expc3)
    print(theta)

def p9():
    print('P9')
    Stheta = np.array([0, 1, 2, 3, 0, 0])
    se3mat = mr.VecTose3(Stheta)
    result = mr.MatrixExp6(se3mat)
    print(result)

def p10():
    print('P10')
    Fb = np.array([1, 0, 0, 2, 1, 0])
    Tbs = np.array([[1, 0, 0, 0],
                    [0, 0, -1, 0],
                    [0, 1, 0, -2],
                    [0, 0, 0, 1]])
    Fs = np.dot(np.transpose(mr.Adjoint(Tbs)), Fb)
    print(Fs)

def p11():
    print('P11')
    T = np.array([[0, -1, 0, 3],
                  [1, 0, 0, 0],
                  [0, 0, 1, 1],
                  [0, 0, 0, 1]])
    result = mr.TransInv(T)
    print(result)

def p12():
    print('P12')
    V = np.array([1, 0, 0, 0, 2, 3])
    result = mr.VecTose3(V)
    print(result)

def p13():
    print('P13')
    s = np.array([1, 0, 0])
    p = np.array([0, 0, 2])
    h = 1
    result = mr.ScrewToAxis(p, s, h)
    print(result)

def p14():
    print('P14')
    se3mat = np.array([[0, -1.5708, 0, 2.3562],
                       [1.5708, 0, 0, -2.3562],
                       [0, 0, 0, 1],
                       [0, 0, 0, 0]])
    result = mr.MatrixExp6(se3mat)
    print(result)

def p15():
    print('P15')
    T = np.array([[0, -1, 0, 3],
                  [1, 0, 0, 0],
                  [0, 0, 1, 1],
                  [0, 0, 0, 1]])
    result = mr.MatrixLog6(T)
    print(result)

if __name__ == '__main__':
    p5()
    p7()
    p8()
    p9()
    p10()
    p11()
    p12()
    p13()
    p14()
    p15()
