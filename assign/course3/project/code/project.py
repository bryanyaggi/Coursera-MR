#!/usr/bin/python3

import modern_robotics as mr
import numpy as np
import math

'''
Course 3 Project
'''

'''
Class for UR5 arm described by project.
'''
class UR5:
    def __init__(self):
        M01 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159],
            [0, 0, 0, 1]])
        M12 = np.array([[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0],
            [0, 0, 0, 1]])
        M23 = np.array([[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395],
            [0, 0, 0, 1]])
        M34 = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225],
            [0, 0, 0, 1]])
        M45 = np.array([[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0],
            [0, 0, 0, 1]])
        M56 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465],
            [0, 0, 0, 1]])
        M67 = np.array([[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0],
            [0, 0, 0, 1]])
        G1 = np.diag([0.010267495893, 0.010267495893,  0.00666,
            3.7, 3.7, 3.7])
        G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074,
            8.393, 8.393, 8.393])
        G3 = np.diag([0.049443313556, 0.049443313556, 0.004095,
            2.275, 2.275, 2.275])
        G4 = np.diag([0.111172755531, 0.111172755531, 0.21942,
            1.219, 1.219, 1.219])
        G5 = np.diag([0.111172755531, 0.111172755531, 0.21942,
            1.219, 1.219, 1.219])
        G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822,
            0.1879, 0.1879, 0.1879])
        self.Glist = np.array([G1, G2, G3, G4, G5, G6])
        self.Mlist = np.array([M01, M12, M23, M34, M45, M56, M67])
        self.Slist = np.array(
                [[0,         0,         0,         0,        0,        0],
                 [0,         1,         1,         1,        0,        1],
                 [1,         0,         0,         0,       -1,        0],
                 [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
                 [0,         0,         0,         0,  0.81725,        0],
                 [0,         0,     0.425,   0.81725,        0,  0.81725]])

    '''
    Runs dynamic simulation of arm with no joint torques, friction, or external
    forces, just gravity. Uses forward dynamics and first-order Euler
    integration.

    thetalistInit is the vector of intial joint positions.
    filename is the name of the CSV file to be generated.
    timestep is set to 10 ms by default.
    '''
    def simulate(self, thetalistInit, filename, time, timestep=.01):
        thetalist = thetalistInit
        dthetalist = np.array([0, 0, 0, 0, 0, 0])
        taulist = np.array([0, 0, 0, 0, 0, 0])
        g = np.array([0, 0, -9.81])
        Ftip = np.array([0, 0, 0, 0, 0, 0])

        def writeJointValues(f, thetalist):
            for i in range(len(thetalist)):
                f.write('%s' %thetalist[i])
                if i < len(thetalist) - 1:
                    f.write(',')
            f.write('\n')

        with open(filename, 'w') as f:
            t = 0
            while t < time:
                writeJointValues(f, thetalist)
                ddthetalist = mr.ForwardDynamics(thetalist, dthetalist, taulist,
                        g, Ftip, self.Mlist, self.Glist, self.Slist)
                thetalist, dthetalist = mr.EulerStep(thetalist, dthetalist,
                        ddthetalist, timestep)
                t += timestep

    '''
    Runs Simulation 1.
    '''
    def simulation1(self):
        thetalistInit = np.array([0, 0, 0, 0, 0, 0])
        filename = 'simulation1.csv'
        time = 3
        self.simulate(thetalistInit, filename, time)

    '''
    Runs Simulation 2.
    '''
    def simulation2(self):
        thetalistInit = np.array([0, -1, 0, 0, 0, 0])
        filename = 'simulation2.csv'
        time = 5
        self.simulate(thetalistInit, filename, time)

if __name__ == '__main__':
    ur5 = UR5()
    ur5.simulation1()
    ur5.simulation2()
