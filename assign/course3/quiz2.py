#!/usr/bin/python3

import numpy as np
import modern_robotics as mr
import math
import sympy as sym

'''
Course 3, Week 2 Quiz
'''

class Quiz2:
    def __init__(self):
        M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
        M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
        M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
        M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
        M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
        M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
        M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
        G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
        G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393,
            8.393])
        G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275,
            2.275])
        G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219,
            1.219])
        G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219,
            1.219])
        G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879,
            0.1879, 0.1879])
        self.Glist = [G1, G2, G3, G4, G5, G6]
        self.Mlist = [M01, M12, M23, M34, M45, M56, M67] 
        self.Slist = [[0,         0,         0,         0,        0,        0],
                 [0,         1,         1,         1,        0,        1],
                 [1,         0,         0,         0,       -1,        0],
                 [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
                 [0,         0,         0,         0,  0.81725,        0],
                 [0,         0,     0.425,   0.81725,        0,  0.81725]]

        self.thetalist = [0, math.pi/6, math.pi/4, math.pi/3, math.pi/2,
                2*math.pi/3]
        self.dthetalist = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
        self.ddthetalist = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        self.g = [0, 0, -9.81]
        self.Ftip = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    
    def p1(self):
        print('P1')
        M = mr.MassMatrix(self.thetalist, self.Mlist, self.Glist, self.Slist)
        print('M =\n%s' %M)

    def p2(self):
        print('P2')
        c = mr.VelQuadraticForces(self.thetalist, self.dthetalist, self.Mlist,
                self.Glist, self.Slist)
        print('c = \n%s' %c)

    def p3(self):
        print('P3')
        grav = mr.GravityForces(self.thetalist, self.g, self.Mlist, self.Glist,
                self.Slist)
        print('grav = \n%s' %grav)
    
    def p4(self):
        print('P4')
        tau_ftip = mr.EndEffectorForces(self.thetalist, self.Ftip, self.Mlist,
                self.Glist, self.Slist)
        print('tau_ftip = \n%s' %tau_ftip)
    
    def p5(self):
        print('P5')
        taulist = [0.0128, -41.1477, -3.7809, 0.0323, 0.0370, 0.1034]
        ddthetalist = mr.ForwardDynamics(self.thetalist, self.dthetalist,
                taulist, self.g, self.Ftip, self.Mlist, self.Glist, self.Slist)
        print('ddthetalist = \n%s' %ddthetalist)

def printDivider():
    print('-' * 80)

if __name__ == '__main__':
    q = Quiz2()
    q.p1()
    printDivider()
    q.p2()
    printDivider()
    q.p3()
    printDivider()
    q.p4()
    printDivider()
    q.p5()
