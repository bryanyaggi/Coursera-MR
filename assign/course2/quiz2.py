#!/usr/bin/python3

import numpy as np
import modern_robotics as mr
import math

'''
Course 2, Week 2 Quiz
'''

def p1():
    print('P1')
    Slist = np.array([[0, 0, 0],
                      [0, 0, 0],
                      [1, 1, 1],
                      [0, 0, 0],
                      [0, -1, -2],
                      [0, 0, 0]])
    thetalist = np.array([0, math.pi/4, 5*math.pi/4])
    F_s = np.array([0, 0, 0, 2, 0, 0])
    J_s = mr.JacobianSpace(Slist, thetalist)
    tau = np.dot(np.transpose(J_s), F_s)
    print(tau)

def p2():
    print('P2')
    L_1 = 1
    L_2 = 1
    L_3 = 1
    L_4 = 1
    thetalist = np.array([0, 0, math.pi/2, -math.pi/2])
    s_4 = math.sin(thetalist[3])
    s_34 = math.sin(thetalist[2] + thetalist[3])
    s_234 = math.sin(thetalist[1] + thetalist[2] + thetalist[3])
    c_4 = math.cos(thetalist[3])
    c_34 = math.cos(thetalist[2] + thetalist[3])
    c_234 = math.cos(thetalist[1] + thetalist[2] + thetalist[3])
    J_b = np.array([[0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [1, 1, 1, 1],
                    [L_3 * s_4 + L_2 * s_34 + L_1 * s_234,
                        L_3 * s_4 + L_2 * s_34, L_3 * s_4, 0],
                    [L_4 + L_3 * c_4 + L_2 * c_34 + L_1 * c_234,
                        L_4 + L_3 * c_4 + L_2 * c_34, L_4 + L_3 * c_4, L_4],
                    [0, 0, 0, 0]])
    F_b = np.array([0, 0, 10, 10, 10, 0])
    tau = np.dot(np.transpose(J_b), F_b)
    print(tau)

def p3():
    print('P3')
    Slist = np.array([[0, 1, 0],
                      [0, 0, 0],
                      [1, 0, 0],
                      [0, 0, 0],
                      [0, 2, 1],
                      [0, 0, 0]])
    thetalist = np.array([math.pi/2, math.pi/2, 1])
    J_s = mr.JacobianSpace(Slist, thetalist)
    print(J_s)

def p4():
    print('P4')
    Blist = np.array([[0, -1, 0],
                      [1, 0, 0],
                      [0, 0, 0],
                      [3, 0, 0],
                      [0, 3, 0],
                      [0, 0, 1]])
    thetalist = np.array([math.pi/2, math.pi/2, 1])
    J_b = mr.JacobianBody(Blist, thetalist)
    print(J_b)

def p5and6():
    print('P5')
    J_b = np.array([[0, -1, 0, 0, -1, 0, 0],
                    [0, 0, 1, 0, 0, 1, 0],
                    [1, 0, 0, 1, 0, 0, 1],
                    [-0.105, 0, 0.006, -0.045, 0, 0.006, 0],
                    [-0.889, 0.006, 0, -0.844, 0.006, 0, 0],
                    [0, -0.105, 0.889, 0, 0, 0, 0]])
    J_bv = J_b[3:, :]
    A = np.dot(J_bv, np.transpose(J_bv))
    eigenVals, eigenVecs = np.linalg.eig(A)
    index = np.argmax(eigenVals)
    pricAxis = eigenVecs[:, index]
    pricAxis = mr.Normalize(pricAxis)
    print(pricAxis)
    print('P6')
    print(math.sqrt(eigenVals[index]))

def printDivider():
    print('-' * 80)

if __name__ == '__main__':
    p1()
    printDivider()
    p2()
    printDivider()
    p3()
    printDivider()
    p4()
    printDivider()
    p5and6()
