#!/usr/bin/python3

import random
import math
import matplotlib.pyplot as plt

import unittest

'''
Course 4 Sample-Based Planning Project
'''

'''
Class for storing node information.
'''
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

    '''
    Calculates Euclidean squared distance between nodes.

    other is another node.
    '''
    def sqDist(self, other):
        return (other.x - self.x) ** 2 + (other.y - self.y) ** 2

    def __repr__(self):
        return 'Node((%s,%s))' %(self.x, self.y)

'''
Class for storing obstacle information.
'''
class Obstacle:
    def __init__(self, x, y, diameter):
        self.x = x
        self.y = y
        self.radius = diameter / 2

    def __repr__(self):
        return 'Obstacle((%s,%s), %s)' %(self.x, self.y, self.size)

'''
Sample parameters for alternative sampling method.
'''
class SampleParam:
    def __init__(self, xRange, yRange, delta):
        self.xRange = xRange
        self.yRange = yRange
        self.delta = delta
        self.xStart = xRange[0]
        self.yStart = yRange[0]

    def sampleRange(self, x=True):
        if x:
            return self.xStart, self.xStart + self.delta
        return self.yStart, self.yStart + self.delta
    
    def xSampleRange(self):
        return self.sampleRange(x=True)

    def ySampleRange(self):
        return self.sampleRange(x=False)

    def update(self):
        self.xStart += self.delta
        if self.xStart >= self.xRange[1]:
            self.xStart = self.xRange[0]
            self.yStart += self.delta
            if self.yStart >= self.yRange[1]:
                self.yStart = self.yRange[0]

'''
Class that implements rapidly exploring random trees (RRTs).
'''
class RRT:
    def __init__(self, obstaclesFile='obstacles.csv', xRange=(-0.5, 0.5),
            yRange=(-0.5, 0.5), start=(-0.5, -0.5), goal=(0.5, 0.5)):
        self.xRange = xRange
        self.yRange = yRange
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacles = []
        self.readObstacles(obstaclesFile)
        self.nodes = {1: self.start} # initialize with start node
        self.path = []
        self.sp = SampleParam(xRange, yRange, 0.1)

    '''
    Reads obstacle information from file.
    '''
    def readObstacles(self, obstaclesFile):
        with open(obstaclesFile) as f:
            lines = f.readlines()
        for line in lines:
            if line[0] != '#':
                line = line.split(',')
                self.obstacles.append(Obstacle(float(line[0]), float(line[1]),
                    float(line[2])))

    '''
    Creates node at a randomly sampled location in the configuration space. The
    goal node is selected with the specified probability.

    goalSampleProbability is the probability for selecting the goal node.
    '''
    def sampleUniform(self, goalSampleProbability=0.1):
        if random.random() <= goalSampleProbability:
            return self.goal
        x = random.uniform(*self.xRange)
        y = random.uniform(*self.yRange)
        return Node(x, y)

    def sampleAlternate(self, goalSampleProbability=0.1):
        if random.random() <= goalSampleProbability:
            return self.goal
        x = random.uniform(*self.sp.xSampleRange())
        y = random.uniform(*self.sp.ySampleRange())
        self.sp.update()
        return Node(x, y)

    '''
    Finds nearest RRT node to specified node. Uses brute force approach.
    Returns node ID of nearest node.

    node is the node for which the query is made.
    '''
    def nearest(self, node):
        nearest = None
        minVal = self.goal.sqDist(self.start) + 1 # greater than maximum possible
        for nodeId in self.nodes:
            val = node.sqDist(self.nodes[nodeId])
            if val < minVal:
                nearest = nodeId
                minVal = val
        return nearest
    
    '''
    Checks if straight-line path between nodes intersects the specified
    obstacle. Returns True if collision, False otherwise. Algebraically solves
    for intersection locations. See link for details.
    https://math.stackexchange.com/questions/275529/check-if-line-intersects-
    with-circles-perimeter

    node1 is the first node.
    node2 is the second node.
    obstacle is the obstacle.
    '''
    @staticmethod
    def collisionCheck(node1, node2, obstacle):
        a_x = node1.x - obstacle.x
        a_y = node1.y - obstacle.y
        b_x = node2.x - obstacle.x
        b_y = node2.y - obstacle.y
        a = (b_x - a_x) ** 2 + (b_y - a_y) ** 2
        b = 2 * (a_x * (b_x - a_x) + a_y * (b_y - a_y))
        c = a_x ** 2 + a_y ** 2 - obstacle.radius ** 2
        discriminant = b ** 2 - 4 * a * c

        if discriminant < 0:
            return False

        sqrtDisc = math.sqrt(discriminant)
        t_1 = (-b + sqrtDisc) / (2 * a)
        t_2 = (-b - sqrtDisc) / (2 * a)
        if (t_1 >= 0 and t_1 <= 1) or (t_2 >= 0 and t_2 <= 1):
            return True
        
        return False

    '''
    Determines if straight-line path exists between nodes. Returns True is so,
    False otherwise.
    '''
    def localPlanner(self, node1, node2):
        for obstacle in self.obstacles:
            if self.collisionCheck(node1, node2, obstacle):
                return False
        return True

    '''
    Finds path using RRT algorithm.

    maximum nodes is the maximum number of nodes used before stopping if a path
    is not found.
    '''
    def findPath(self, maximumNodes=100):
        self.path = [] # reset path
        nodeId = 2 # start node has ID 1
        pathFound = False
        while len(self.nodes) < maximumNodes:
            candidateNode = self.sampleUniform()
            nearestNodeId = self.nearest(candidateNode)
            if self.localPlanner(self.nodes[nearestNodeId], candidateNode):
                candidateNode.parent = nearestNodeId
                self.nodes[nodeId] = candidateNode
                if self.goal.sqDist(candidateNode) == 0:
                    pathFound = True
                    break
                nodeId += 1

        if pathFound:
            while nodeId != None:
                self.path.append(nodeId)
                nodeId = self.nodes[nodeId].parent
            self.path.reverse()

    '''
    Creates output files including path, nodes, and edges files.
    '''
    def writeOutputFiles(self, pathFile='path.csv', nodesFile='nodes.csv',
            edgesFile='edges.csv'):
        with open(pathFile, 'w') as f:
            for i in range(len(self.path)):
                f.write('%s' %self.path[i])
                if i < len(self.path) - 1:
                    f.write(',')

        with open(nodesFile, 'w') as f:
            for nodeId in self.nodes:
                f.write('%s,%s,%s,0\n' %(nodeId, self.nodes[nodeId].x,
                    self.nodes[nodeId].y))

        with open(edgesFile, 'w') as f:
            for nodeId in self.nodes:
                parentId = self.nodes[nodeId].parent
                if parentId is not None:
                    f.write('%s,%s,0\n' %(nodeId, parentId))
    
    '''
    Plots results using matplotlib.
    '''
    def plot(self):
        fig, ax = plt.subplots()

        for obstacle in self.obstacles:
            circle = plt.Circle((obstacle.x, obstacle.y), obstacle.radius,
                    color='black')
            ax.add_artist(circle)
        for nodeId in self.nodes:
            ax.scatter(self.nodes[nodeId].x, self.nodes[nodeId].y,
                    color='blue')
            parentId = self.nodes[nodeId].parent
            if parentId is not None:
                ax.plot([self.nodes[parentId].x, self.nodes[nodeId].x],
                        [self.nodes[parentId].y, self.nodes[nodeId].y],
                        linewidth=0.5, color='blue')
        for i in range(1, len(self.path)):
            childId = self.path[i]
            parentId = self.path[i - 1]
            ax.plot([self.nodes[parentId].x, self.nodes[childId].x],
                    [self.nodes[parentId].y, self.nodes[childId].y],
                    color='green')

        ax.axis('equal')
        ax.set_xlim(left = -0.5, right = 0.5)
        ax.set_ylim(bottom = -0.5, top = 0.5)
        plt.show()

class Test(unittest.TestCase):
    def setUp(self):
        self.rrt = RRT()
    
    def testSample(self):
        for _ in range(10):
            node = self.rrt.sample()
            self.assertGreaterEqual(node.x, self.rrt.xRange[0])
            self.assertGreaterEqual(node.y, self.rrt.yRange[0])
            self.assertLessEqual(node.x, self.rrt.xRange[1])
            self.assertLessEqual(node.y, self.rrt.yRange[1])

    def testNearest(self):
        self.rrt.nodes[1] = Node(0, 0)
        self.rrt.nodes[2] = Node(-0.25, -0.25)
        self.rrt.nodes[3] = Node(0.25, 0.25)
        result = self.rrt.nearest(Node(0.5, 0.5))
        self.assertEqual(result, 3)

    def testLocalPlanner(self):
        result = self.rrt.localPlanner(self.rrt.start, self.rrt.goal)
        #print(result)

if __name__ == '__main__':
    rrt = RRT()
    rrt.findPath()
    rrt.writeOutputFiles()
    rrt.plot()
