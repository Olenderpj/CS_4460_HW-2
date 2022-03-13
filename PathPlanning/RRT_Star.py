import random
import math
import pygame
import numpy as np
from Utilities.SimulationUtils import *
from Constants.EnvironmentConstants import *

show_animation = True

XDIM = 1000
YDIM = 1000
windowSize = [XDIM, YDIM]

pygame.init()
fpsClock = pygame.time.Clock()

screen = pygame.display.set_mode(windowSize)
pygame.display.set_caption('RRT* Path Planning')


class RRT():
    """
    Class for RRT* Path Planning
    """

    def __init__(self, start, goal, obstacleList,
                 randArea, expandDis=RRT_NODE_SPACING, goalSampleRate=10, maxIter=1500):

        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.Xrand = randArea[0]
        self.Yrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=True):
        self.nodeList = {0: self.start}
        i = 0

        while True:
            print(i)

            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(rnd)  # get nearest node index to random point
            newNode = self.steer(rnd, nind)  # generate new node from that nearest node in direction of random point

            if self.__CollisionCheck(newNode, self.obstacleList):  # if it does not collide

                nearinds = self.find_near_nodes(newNode, 5)  # find nearest nodes to newNode
                newNode = self.choose_parent(newNode,
                                             nearinds)  # from that nearest nodes find the best parent to newNode
                self.nodeList[i + 100] = newNode  # add newNode to nodeList
                self.rewire(i + 100, newNode, nearinds)  # make newNode a parent of another node if necessary
                self.nodeList[newNode.parent].children.add(i + 100)

                if len(self.nodeList) > self.maxIter:
                    leaves = [key for key, node in self.nodeList.items() if
                              len(node.children) == 0 and len(self.nodeList[node.parent].children) > 1]
                    if len(leaves) > 1:
                        ind = leaves[random.randint(0, len(leaves) - 1)]
                        self.nodeList[self.nodeList[ind].parent].children.discard(ind)
                        self.nodeList.pop(ind)
                    else:
                        leaves = [key for key, node in self.nodeList.items() if len(node.children) == 0]
                        ind = leaves[random.randint(0, len(leaves) - 1)]
                        self.nodeList[self.nodeList[ind].parent].children.discard(ind)
                        self.nodeList.pop(ind)
            i += 1

            if animation and i % 25 == 0:
                self.DrawGraph(rnd)

            for e in pygame.event.get():
                if e.type == pygame.MOUSEBUTTONDOWN:
                    if e.button == 1:
                        self.start.x = e.pos[0]
                        self.start.y = e.pos[1]
                        self.path_validation()
                    elif e.button == 3:
                        self.end.x = e.pos[0]
                        self.end.y = e.pos[1]
                        self.path_validation()

    def path_validation(self):
        lastIndex = self.get_best_last_index()
        if lastIndex is not None:
            while self.nodeList[lastIndex].parent is not None:
                nodeInd = lastIndex
                lastIndex = self.nodeList[lastIndex].parent

                dx = self.nodeList[nodeInd].x - self.nodeList[lastIndex].x
                dy = self.nodeList[nodeInd].y - self.nodeList[lastIndex].y
                d = math.sqrt(dx ** 2 + dy ** 2)
                theta = math.atan2(dy, dx)
                if not self.check_collision_extend(self.nodeList[lastIndex].x, self.nodeList[lastIndex].y, theta, d):
                    self.nodeList[lastIndex].children.discard(nodeInd)
                    self.remove_branch(nodeInd)

    def remove_branch(self, nodeInd):
        for ix in self.nodeList[nodeInd].children:
            self.remove_branch(ix)
        self.nodeList.pop(nodeInd)

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            dx = newNode.x - self.nodeList[i].x
            dy = newNode.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeList[i].x, self.nodeList[i].y, theta, d):
                dlist.append(self.nodeList[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode.cost = mincost
        newNode.parent = minind
        return newNode

    def steer(self, rnd, nind):

        # expand tree
        nearestNode = self.nodeList[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
        newNode = Node(nearestNode.x, nearestNode.y)
        newNode.x += self.expandDis * math.cos(theta)
        newNode.y += self.expandDis * math.sin(theta)

        newNode.cost = nearestNode.cost + self.expandDis
        newNode.parent = nind
        return newNode

    def get_random_point(self):
        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(0, self.Xrand), random.uniform(0, self.Yrand)]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y]
        return rnd

    def get_best_last_index(self):

        distanceToGoalList = [(key, self.calc_dist_to_goal(node.x, node.y)) for key, node in self.nodeList.items()]
        goalinds = [key for key, distance in distanceToGoalList if distance <= self.expandDis]

        if len(goalinds) == 0:
            return None

        mincost = min([self.nodeList[key].cost for key in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalIndex):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalIndex].parent is not None:
            node = self.nodeList[goalIndex]
            path.append([node.x, node.y])
            goalIndex = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode, value):
        r = self.expandDis * value

        dlist = np.subtract(np.array([(node.x, node.y) for node in self.nodeList.values()]),
                            (newNode.x, newNode.y)) ** 2
        dlist = np.sum(dlist, axis=1)
        nearinds = np.where(dlist <= r ** 2)
        nearinds = np.array(list(self.nodeList.keys()))[nearinds]

        return nearinds

    def rewire(self, newNodeInd, newNode, nearinds):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode.x, nearNode.y, theta, d):
                    self.nodeList[nearNode.parent].children.discard(i)
                    nearNode.parent = newNodeInd
                    nearNode.cost = scost
                    newNode.children.add(i)

    def check_collision_extend(self, nix, niy, theta, d):

        tmpNode = Node(nix, niy)

        for i in range(int(d / 5)):
            tmpNode.x += 5 * math.cos(theta)
            tmpNode.y += 5 * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, self.obstacleList):
                return False

        return True

    def DrawGraph(self, rnd=None):

        # Fill the background with white
        screen.fill((255, 255, 255))
        for node in self.nodeList.values():
            if node.parent is not None:
                pygame.draw.line(screen, (192, 192, 192), [self.nodeList[node.parent].x, self.nodeList[node.parent].y],
                                 [node.x, node.y])

        # Draw a circle at all child nodes
        for node in self.nodeList.values():
            if len(node.children) == 0:
                pygame.draw.circle(screen, (255, 204, 153), [int(node.x), int(node.y)], 2)

        # Draw all rectangle objects
        for (rectangleX, rectangleY, width, height) in self.obstacleList:
            pygame.draw.rect(screen, (0, 0, 0), [(rectangleX, rectangleY), (width, height)])

        pygame.draw.circle(screen, (0, 255, 0), [self.start.x, self.start.y], 10)
        pygame.draw.circle(screen, (255, 0, 0), [self.end.x, self.end.y], 10)

        lastIndex = self.get_best_last_index()
        if lastIndex is not None:
            path = self.gen_final_course(lastIndex)

            ind = len(path)
            while ind > 1:
                pygame.draw.line(screen, (255, 0, 0), path[ind - 2], path[ind - 1])
                ind -= 1

        pygame.display.update()

    def GetNearestListIndex(self, rnd):
        dlist = np.subtract(np.array([(node.x, node.y) for node in self.nodeList.values()]), (rnd[0], rnd[1])) ** 2
        dlist = np.sum(dlist, axis=1)
        minind = list(self.nodeList.keys())[np.argmin(dlist)]
        return minind

    # Evaulate all nodes against the obstacles and determine if it collides with an obstacle
    def __CollisionCheck(self, node, obstacleList):

        for (rectangleX, rectangleY, width, height) in obstacleList:
            rectangleX, rectangleY, width, height = rectangleX + 2, rectangleY + 2, width + 2, height + 2
            if node.x > rectangleX and node.x < rectangleX + width:
                if node.y > rectangleY and node.y < rectangleY + height:
                    return False

        return True  # safe


class Node():

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None
        self.children = set()


def planRRTStarPath(simulation):
    print("start RRT path planning")

    # ====Search Path with RRT====

    if OBJECT_TEXT_FILE_FLAG == 0:
        obstacleList = getAllSceneShapes(simulation) # [x,y,size]
    else:
        obstacleList = readSceneObjectsFromFile()

    print(obstacleList)
    # Set Initial parameters
    if COURSE_FLAG == 0:
        if MAZE_START_FLAG == 1:
            rrt = RRT(start=[MAZE_START_X1, MAZE_START_Y1], goal=[MAZE_END_X, MAZE_END_Y], randArea=[XDIM, YDIM], obstacleList=obstacleList)
        elif MAZE_START_FLAG == 2:
            rrt = RRT(start=[MAZE_START_X2, MAZE_START_Y2], goal=[MAZE_END_X, MAZE_END_Y], randArea=[XDIM, YDIM], obstacleList=obstacleList)
        elif MAZE_START_FLAG == 3:
            rrt = RRT(start=[MAZE_START_X3, MAZE_START_Y3], goal=[MAZE_END_X, MAZE_END_Y], randArea=[XDIM, YDIM], obstacleList=obstacleList)
    elif COURSE_FLAG == 1:
        if OC_START_FLAG == 1:
            rrt = RRT(start=[OC_START_X1, OC_START_Y1], goal=[OC_END_X, OC_END_Y], randArea=[XDIM, YDIM], obstacleList=obstacleList)
        elif OC_START_FLAG == 2:
            rrt = RRT(start=[OC_START_X2, OC_START_Y2], goal=[OC_END_X, OC_END_Y], randArea=[XDIM, YDIM], obstacleList=obstacleList)
        elif OC_START_FLAG == 3:
            rrt = RRT(start=[OC_START_X3, OC_START_Y3], goal=[OC_END_X, OC_END_Y], randArea=[XDIM, YDIM], obstacleList=obstacleList)

    path = rrt.Planning(animation=show_animation)
    print(path)
