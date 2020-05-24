#!/usr/bin/python3

import numpy as np
import modern_robotics as mr
import math

'''
Course 2, Week 1 Quiz
'''

class Robot:
    def __init__(self, numJoints):
        self.M = np.identity(4)
        self.Slist = np.zeros((6, numJoints))
        self.Blist = np.zeros((6, numJoints))



'''
Calculate M matrix for given URRPR robot.
'''
def p1():
    print('P1')
    L = 1
    M = np.array([[1, 0, 0, (2 + math.sqrt(3)) * L],
                  [0, 1, 0, 0],
                  [0, 0, 1, (1 + math.sqrt(3)) * L],
                  [0, 0, 0, 1]])
    print(M)
    return M

'''
Calculate screw axes in the spacial frame for the same robot when in its zero
position.
'''
def p2():
    print('P2')
    L = 1
    screwAxesS = []
    screwAxesS.append(np.array([0, 0, 1, 0, -L, 0]))
    screwAxesS.append(np.array([0, 1, 0, 0, 0, L]))
    screwAxesS.append(np.array([0, 1, 0, L, 0, (1 + math.sqrt(3)) * L]))
    screwAxesS.append(np.array([0, 1, 0, -(math.sqrt(3) - 1) * L, 0, (2 +
        math.sqrt(3)) * L]))
    screwAxesS.append(np.array([0, 0, 0, 0, 0, 1]))
    screwAxesS.append(np.array([0, 0, 1, 0, -(2 + math.sqrt(3)) * L, 0]))

    screwAxesSMatrix = np.zeros((len(screwAxesS[0]), len(screwAxesS)))
    for i in range(len(screwAxesS)):
        print('S%d: %s' %(i + 1, screwAxesS[i]))
        screwAxesSMatrix[:, i] = screwAxesS[i]

    print(screwAxesSMatrix)
    return screwAxesSMatrix

'''
Calculate the screw axes in the end effector frame for the same robot when in
its zero position.
'''
def p3():
    print('P3')
    L = 1
    screwAxesB = []
    screwAxesB.append(np.array([0, 0, 1, 0, (1 + math.sqrt(3)) * L, 0]))
    screwAxesB.append(np.array([0, 1, 0, (1 + math.sqrt(3)) * L, 0, -(1 +
        math.sqrt(3)) * L]))
    screwAxesB.append(np.array([0, 1, 0, (2 + math.sqrt(3)) * L, 0, -L]))
    screwAxesB.append(np.array([0, 1, 0, 2 * L, 0, 0]))
    screwAxesB.append(np.array([0, 0, 0, 0, 0, 1]))
    screwAxesB.append(np.array([0, 0, 1, 0, 0, 0]))

    screwAxesBMatrix = np.zeros((len(screwAxesB[0]), len(screwAxesB)))
    for i in range(len(screwAxesB)):
        print('B%d: %s' %(i + 1, screwAxesB[i]))
        screwAxesBMatrix[:, i] = screwAxesB[i]

    print(screwAxesBMatrix)
    return screwAxesBMatrix

def p4(robot):
    print('P4')
    thetas = np.array([-math.pi/2, math.pi/2, math.pi/3, -math.pi/4, 1, math.pi/6])
    T = mr.FKinSpace(robot.M, robot.Slist, thetas)
    print(T)

def p5(robot):
    print('P5')
    thetas = np.array([-math.pi/2, math.pi/2, math.pi/3, -math.pi/4, 1, math.pi/6])
    T = mr.FKinBody(robot.M, robot.Blist, thetas)
    print(T)

def printDivider():
    print('-' * 80)

if __name__ == '__main__':
    robot = Robot(6)
    robot.M = p1()
    printDivider()
    robot.Slist = p2()
    printDivider()
    robot.Blist = p3()
    printDivider()
    p4(robot)
    printDivider()
    p5(robot)
