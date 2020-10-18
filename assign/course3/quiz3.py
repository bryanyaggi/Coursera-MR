#!/usr/bin/python3

import numpy as np
import modern_robotics as mr
import math
import sympy as sym
import matplotlib.pyplot as plt

'''
Course 3, Week 3 Quiz
'''

def lc1():
    print('LC1')
    x = []
    y = []
    steps = 100
    for i in range(steps + 1):
        s = i / steps
        x.append(1 + 2 * math.cos(math.pi * s))
        y.append(2 * math.sin(math.pi * s))
    
    fig, ax = plt.subplots()
    ax.plot(x, y)
    plt.show()

def p1():
    print('P1')
    x = []
    y = []
    steps = 100
    for i in range(steps + 1):
        s = i / steps
        x.append(1.5 * (1 - math.cos(2 * math.pi * s)))
        y.append(math.sin(2 * math.pi * s))

    fig, ax = plt.subplots()
    ax.plot(x, y)
    plt.show()

def p2():
    print('P2')
    t = sym.symbols('t')
    T = sym.symbols('T')
    a_0 = 0
    a_1 = 0
    a_2 = 0
    a_3 = 10 / (T**3)
    a_4 = -15 / (T**4)
    a_5 = 6 / (T**5)
    s = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
    s_dot = sym.Derivative(s, t).doit()
    s_dotdot = sym.Derivative(s_dot, t).doit()
    assert(s.subs(t, T) == 1)
    assert(s.subs(t, 0) == 0)
    assert(s_dot.subs(t, 0) == 0)
    assert(s_dot.subs(t, T) == 0)
    assert(s_dotdot.subs(t, 0) == 0)
    assert(s_dotdot.subs(t, T) == 0)

def p5():
    print('P5')
    Tf = 5
    t = 3
    ans = mr.QuinticTimeScaling(Tf, t)
    print('s(3) = %s' %ans)

def p6():
    print('P6')
    X_start = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
    X_end = np.array([[0, 0, 1, 1],
                      [1, 0, 0, 2],
                      [0, 1, 0, 3],
                      [0, 0, 0, 1]])
    Tf = 10
    N = 10
    method = 3
    traj = mr.ScrewTrajectory(X_start, X_end, Tf, N, method)

    ans = traj[-2]
    print('second to last via configuration:\n%s' %ans)

def p7():
    print('P7')
    X_start = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
    X_end = np.array([[0, 0, 1, 1],
                      [1, 0, 0, 2],
                      [0, 1, 0, 3],
                      [0, 0, 0, 1]])
    Tf = 10
    N = 10
    method = 5
    traj = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)

    ans = traj[-2]
    print('second to last via configuration:\n%s' %ans)

def printDivider():
    print('-' * 80)

if __name__ == '__main__':
    #lc1()
    p1()
    printDivider()
    p2()
    printDivider()
    p5()
    printDivider()
    p6()
    printDivider()
    p7()
