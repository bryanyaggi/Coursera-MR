#!/usr/bin/python3

from heapdict import heapdict
import sys

'''
Course 4 A* Search Project
'''

'''
Class for storing node information
'''
class Node:
    def __init__(self, x, y, heuristic):
        self.x = x
        self.y = y
        self.heuristic = heuristic

    def __repr__(self):
        return 'Node((%s,%s), %s)' %(self.x, self.y, self.heuristic)

'''
Class for storing graph information and running A* search
'''
class Graph:
    def __init__(self, nodesFile='nodes.csv', edgesFile='edges.csv'):
        self.nodesFile = nodesFile
        self.edgesFile = edgesFile
        self.nodes = {}
        self.nonNode = -1
        self.startNode = self.nonNode
        self.goalNode = self.nonNode
        self.edges = {}
        self.readNodes()
        self.readEdges()

    '''
    Reads node information from nodes file.
    '''
    def readNodes(self):
        with open(self.nodesFile) as f:
            lines = f.readlines()

        id_ = self.nonNode
        for line in lines:
            if line[0] != '#':
                line = line.split(',')
                id_ = int(line[0])
                if self.startNode == self.nonNode:
                    self.startNode = id_ # first node in file is start
                self.nodes[id_] = Node(float(line[1]), float(line[2]),
                        float(line[3]))
                self.edges[id_] = {}
        self.goalNode = id_ # last node in file is goal
        if self.startNode == self.nonNode:
            print('No nodes read from file. Exiting...')
            sys.exit()
        if self.goalNode == self.startNode:
            print('Goal and start nodes are the same. Exiting...')
            sys.exit()

    '''
    Reads edge information from edges file.
    '''
    def readEdges(self):
        with open(self.edgesFile) as f:
            lines = f.readlines()
        for line in lines:
            if line[0] != '#':
                line = line.split(',')
                node1 = int(line[0])
                node2 = int(line[1])
                cost = float(line[2])
                self.edges[node1][node2] = cost
                self.edges[node2][node1] = cost

    '''
    Executes A* search and outputs path to path file.
    '''
    def aStar(self, pathFile='path.csv', maxScore=99):
        heap = heapdict()
        for node in self.edges:
            if node == self.startNode:
                heap[node] = self.nodes[node].heuristic
            else:
                heap[node] = maxScore
        distances = {self.startNode: 0}
        predecessors = {}

        while len(heap) > 0 and self.goalNode in heap:
            node, score = heap.popitem()
            for neighbor in self.edges[node]:
                if neighbor in heap:
                    distance = distances[node] + self.edges[node][neighbor]
                    newScore = distance + self.nodes[neighbor].heuristic
                    if newScore < heap[neighbor]:
                        heap[neighbor] = newScore
                        distances[neighbor] = distance
                        predecessors[neighbor] = node

        # Create path
        node = self.goalNode
        revPath = [self.goalNode]
        while node != self.startNode:
            predecessor = predecessors[node]
            revPath.append(predecessor)
            node = predecessor

        # Write path file
        with open(pathFile, 'w') as f:
            for i in range(len(revPath) - 1, -1, -1):
                f.write('%s' %revPath[i])
                if i > 0:
                    f.write(',')

if __name__ == '__main__':
    g = Graph()
    g.aStar()
