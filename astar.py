# -*- coding: utf-8 -*-
"""
Created on Fri Aug 10 11:39:45 2018

@author: Morgan.Li
"""

import numpy as np
from node import Node
from utility import cal_Euclidean_dist, cal_Manhattan_dist
import logging

logger = logging.getLogger("pathPlan")

class Astar:
    def __init__(self, start, goal, cspace, space_boundary, walk_dirs):
        """
        start and goal are in degree
        """
        self.real_start = start
        self.real_goal = goal
        self.start = start
        self.goal = goal
        self.cspace = cspace
        self.space_boundary = space_boundary
        self.walk_dirs = walk_dirs

    def search(self):
        openset = set()
        closedset = set()

        start = Node(self.start, self.space_boundary, self.walk_dirs)
        start.h = start.cost(self.goal)
        openset.add(start)

        final_node = None
        reached = False
        while openset and not reached:
            q = min(openset, key=lambda o:o.f)
            openset.remove(q)
            final_node = q
            successors = q.find_next_nodes(self.cspace, self.goal)
            for successor in successors:
                if np.array_equal(successor.loc, self.goal):
                    final_node = successor
                    reached = True
                    logger.info("Reached!")
                    break
                if successor in closedset:
                    continue
                if successor in openset:
                    for s in openset:
                        if np.array_equal(s.loc, successor.loc) and successor.f < s.f:
                            s.parent = successor.parent
                            s.g = successor.g
                            s.h = successor.h
                            s.f = successor.f
                            break
                else:
                    openset.add(successor)
            closedset.add(q)

        path = self.real_goal
        while final_node.parent is not None:
            path = np.vstack((final_node.loc, path))
            final_node = final_node.parent
        path = np.vstack((self.real_start, path))
        return path.T
