#!/usr/bin/python3

import numpy as np
import modern_robotics as mr
import math
import sympy as sym

'''
Course 3, Week 1 Quiz
'''

def p1():
    print('P1')
    density = 5600 # kg/m^3
    
    # bell
    radius_bell = .1 # m
    mass_bell = 4/3 * radius_bell**3 * math.pi * density
    offset_bell = .2 # m
    I_bell_xx = mass_bell * 2 * radius_bell**2 / 5 + mass_bell * offset_bell**2
    I_bell_yy = mass_bell * 2 * radius_bell**2 / 5 + mass_bell * offset_bell**2
    I_bell_zz = mass_bell * 2 * radius_bell**2 / 5

    # handle
    radius_handle = .02 # m
    length_handle = .2 # m
    mass_handle = math.pi * radius_handle**2 * length_handle
    I_handle_xx = mass_handle * (3 * radius_handle**2 + length_handle**2) / 12
    I_handle_yy = mass_handle * (3 * radius_handle**2 + length_handle**2) / 12
    I_handle_zz = mass_handle * radius_handle**2 / 2

    I = np.array([[I_handle_xx + 2 * I_bell_xx, 0, 0],
                  [0, I_handle_yy + 2 * I_bell_yy, 0],
                  [0, 0, I_handle_zz + 2 * I_bell_zz]])

    print('I = \n %s' %I)

def p2and3():
    print('P2')

    t = sym.symbols('t')
    m = sym.symbols('m')
    theta_1 = sym.Function('theta_1')(t)
    theta_2 = sym.Function('theta_2')(t)
    theta_1_dot = sym.Derivative(theta_1, t)
    theta_2_dot = sym.Derivative(theta_2, t)
    L_1 = m * theta_1_dot * theta_2_dot * sym.sin(theta_2)
    tau_1_1 = sym.diff(sym.diff(L_1, theta_1_dot), t) - sym.diff(L_1, theta_1)
    print('tau_1_1 = %s' %tau_1_1)

    printDivider()
    print('P3')

    tau_1_2 = sym.diff(sym.diff(L_1, theta_2_dot), t) - sym.diff(L_1, theta_2)
    print('tau_1_2 = %s' %tau_1_2)

def p5():
    print('P5')
    
    M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
    M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
    M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
    M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
    M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
    M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
    M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
    G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
    G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
    G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
    G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
    G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
    G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879,
        0.1879])
    Glist = [G1, G2, G3, G4, G5, G6]
    Mlist = [M01, M12, M23, M34, M45, M56, M67] 
    Slist = [[0,         0,         0,         0,        0,        0],
	     [0,         1,         1,         1,        0,        1],
	     [1,         0,         0,         0,       -1,        0],
	     [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
	     [0,         0,         0,         0,  0.81725,        0],
	     [0,         0,     0.425,   0.81725,        0,  0.81725]]
    thetalist = [0, math.pi/6, math.pi/4, math.pi/3, math.pi/2, 2*math.pi/3]
    dthetalist = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
    ddthetalist = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    g = [0, 0, -9.81]
    Ftip = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

    taulist = mr.InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip,
            Mlist, Glist, Slist)
    
    print('tau = %s' %taulist)

def printDivider():
    print('-' * 80)

if __name__ == '__main__':
    p1()
    printDivider()
    p2and3()
    printDivider()
    p5()
