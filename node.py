# -*- coding: utf-8 -*-
"""
Created on Mon Aug 13 10:27:38 2018

@author: Morgan.Li
"""
import numpy as np
import logging
from utility import check_in_area, is_exist_same_loc
from utility import cal_Euclidean_dist, cal_Manhattan_dist


logger = logging.getLogger("pathPlan")

class Node(object):
    def __init__(self, loc, space_boundary, walk_dirs):
        self.parent = None
        self.walk_dirs = walk_dirs
        self.loc = loc
        # total cost: f = g + h
        self.f = 0.
        # g: the cost from start point to current point
        self.g = 0.
        # h: the cost from current point to goal
        self.h = 0.
        self.space_boundary = space_boundary

    def __eq__(self, other):
        return np.array_equal(self.loc, other.loc)

    def __hash__(self):
        return hash((self.loc[0], self.loc[1], self.loc[2]))


    def cost(self, new_loc):
        return cal_Manhattan_dist(self.loc, new_loc)

    def find_next_nodes(self, cspace, goal):

        available = []
        for direction in self.walk_dirs:
            loc = self.loc + direction
            #check whether node is inside the frame
            # and it is not obstacle
            if (not check_in_area(loc, self.space_boundary)) or is_exist_same_loc(loc, cspace):
#                print("OverStep or exist same loc")
                continue
            else:
                n = Node(loc, self.space_boundary, self.walk_dirs)
                n.parent = self
                # g = g_parent + g_move
                n.g = self.g + self.cost(loc)
                n.h = cal_Manhattan_dist(n.loc, goal)
                n.f = n.g + n.h
                available.append(n)

        return available

class Agent(Node):
    def __init__(self, space_boundary, walk_dirs):
        super(Agent,self).__init__(None, space_boundary, walk_dirs)
        self.name = ''
        self.id = 0
        self.color = 0



