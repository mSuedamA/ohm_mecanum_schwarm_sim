from robot import Robot
import numpy as np
import time
import random

class Node:
    def __init__(self, x, y, g=0, h=0):
        self.coords = [x,y]
        self.root = None
        self.g = g
        self.h = h

    def manhattan(self, destination):
        self.h = abs(destination[0]-self.coords[0])+abs(destination[1]-self.coords[1])

    def move(self, x, y):
        self.coords[0] += x
        self.coords[1] += y

class A_star:
    def __init__(self, robot, endNode, robots):
        self.openList = []
        self.closeList = []
        self.startNode = Node(robot._coords[0],robot._coords[1])
        self.endNode = Node(endNode[0],endNode[1],100)
        self.currentNode = Node(robot._coords[0],robot._coords[1])
        self.pathlist = []
        self.map = []
        self.robot = robot
        self.robots = robots
        self.points = robot.get_points()

    def get_min_f(self):
        nodeTemp = self.openList[0]
        for node in self.openList:
            if node.g + node.h < nodeTemp.g + nodeTemp.h: 
                nodeTemp = node
        return nodeTemp

    def node_in_openlist(self,node):
        for nodeTmp in self.openList:  
            if nodeTmp.coords[0] == node.coords[0] and nodeTmp.coords[1] == node.coords[1]:  
                return True  
        return False

    def node_in_closelist(self,node):
        for nodeTmp in self.closeList:  
            if nodeTmp.coords[0] == node.coords[0] and nodeTmp.coords[1] == node.coords[1]:  
                return True  
        return False

    def endnode_in_openlist(self):  
        for nodeTmp in self.openList:  
            if nodeTmp.coords[0] == self.endNode.coords[0] and nodeTmp.coords[1] == self.endNode.coords[1]:  
                return True  
        return False

    def get_node_from_openlist(self,node):  
        for nodeTmp in self.openList:  
            if nodeTmp.coords[0] == node.coords[0] and nodeTmp.coords[1] == node.coords[1]:  
                return nodeTmp  
        return None

    def checkcollision(self, node):
        distance = [node.coords[0] - self.startNode.coords[0], node.coords[1] - self.startNode.coords[1]]
        points = []
        for i in range(0, len(self.points)): 
            points.append([self.points[i][0]+distance[0],self.points[i][1]+distance[1]])
        for i in range(0, len(self.map)):
            if(self.check_points(points,self.map[i])): 
                return True
        return False

    def searchNode(self, node):
        if(self.checkcollision(node)): return
        if self.node_in_closelist(node): return

        temp_g = node.root.g + abs(node.coords[0]-node.root.coords[0]) + abs(node.coords[1]-node.root.coords[1])
        
        if self.node_in_openlist(node) == False:
            node.g =temp_g
            node.manhattan(self.endNode.coords)
            self.openList.append(node)
            node.root = self.currentNode
        else:
            nodeTmp = self.get_node_from_openlist(node)
            if self.currentNode.g + temp_g < nodeTmp.g :
                nodeTmp.g = self.currentNode.g + temp_g
                nodeTmp.root = self.currentNode

    def searchNear(self):
        x = self.currentNode.coords[0]
        y = self.currentNode.coords[1]
        move = [[0,0.5],[0,-0.5],[0.5,0],[-0.5,0]]
        for m in move:
            newNode = Node(x+m[0],y+m[1])
            newNode.root = self.currentNode
            self.searchNode(newNode)

    def find_temp_node(self, endNode):
        node_list = []
        node_list.append(endNode)
        i = 0
        move = [[0,0.5],[0,-0.5],[0.5,0],[-0.5,0]]
        while(i < len(node_list)):
            node = node_list[i]
            i += 1
            if(self.checkcollision(node) == False): 
                print(self.robot._num,": temp Node:", node.coords)
                return node
            x = node.coords[0]
            y = node.coords[1]
            random.shuffle(move)
            for m in move:
                newNode = Node(x+m[0],y+m[1])
                dupl = False
                for j in node_list:
                    if(j.coords[0]==newNode.coords[0] and j.coords[1]==newNode.coords[1]):
                        dupl = True
                        break
                if(dupl==False): node_list.append(newNode)
        return endNode

    def build_current_map(self):
        self.map = []
        self.map.append([[0,0],[0,10],[16,10],[16,0],[0,0]])
        for j in self.robots:
            if(j != self.robot):
                self.map.append(j.get_points())

    def start(self):
        self.build_current_map()

        if(self.checkcollision(self.endNode)):
            N = self.find_temp_node(self.endNode)
            if self.endNode == N:
                return False
            self.endNode = N

        self.startNode.manhattan(self.endNode.coords)
        self.openList.append(self.startNode)

        while True:
            self.currentNode = self.get_min_f()
            self.closeList.append(self.currentNode)
            self.openList.remove(self.currentNode)

            self.searchNear()

            if self.endnode_in_openlist():
                nodeTemp = self.get_node_from_openlist(self.endNode)
                while True:
                    self.pathlist.append(nodeTemp.coords)
                    if nodeTemp.root != None:
                        nodeTemp = nodeTemp.root
                    else: return True
            elif len(self.openList) == 0:
                N = self.find_temp_node(self.startNode)
                if self.startNode != N:
                    self.closeList = []
                    if(self.start()): return True
                return False

    def cross(self, p1,p2,p3):
        x1=p2[0]-p1[0]
        y1=p2[1]-p1[1]
        x2=p3[0]-p1[0]
        y2=p3[1]-p1[1]
        return x1*y2-x2*y1 

    def IsIntersec(self, p1,p2,p3,p4):
        if(max(p1[0],p2[0])>=min(p3[0],p4[0]) and max(p3[0],p4[0])>=min(p1[0],p2[0]) and max(p1[1],p2[1])>=min(p3[1],p4[1]) and max(p3[1],p4[1])>=min(p1[1],p2[1])):
            if(self.cross(p1,p2,p3)*self.cross(p1,p2,p4)<=0 and self.cross(p3,p4,p1)*self.cross(p3,p4,p2)<=0):
                return True
        return False

    def check_points(self, points1, points2):
        for i in range(1, len(points1)):
            for j in range(1, len(points2)):
                if self.IsIntersec(points1[i-1],points1[i],points2[j-1],points2[j]):
                    return True
        return False