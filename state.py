# -*- coding: utf-8 -*-
"""
Created on Wed Aug 15 09:11:15 2018

@author: Morgan.Li
"""

import numpy as np
import logging
from utility import cal_Euclidean_dist, cal_Manhattan_dist


logger = logging.getLogger("pathPlan")

class State(object):
    def __init__(self, loc):
        #backpointer
        self.bp = None
        self.loc = loc
        # h: the path cost from goal to point
        self.h = 0.
        # since there are many paths from goal to current point,
        # k is the minmal value of those path cost
        self.k = 0.
        self.t = "NEW"
        self.is_obstacle = False


    def cost(self, other):
        if self.is_obstacle or other.is_obstacle:
            return np.inf
        return cal_Manhattan_dist(self.loc, other.loc)




