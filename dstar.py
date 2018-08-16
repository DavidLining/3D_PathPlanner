# -*- coding: utf-8 -*-
"""
Created on Tue Aug 14 09:08:43 2018

@author: Morgan.Li
"""

import numpy as np
from state import State
from utility import cal_Euclidean_dist, cal_Manhattan_dist
from utility import check_in_area, is_exist_same_loc, remove_same_loc
import logging


logger = logging.getLogger("pathPlan")

class Map:

    def __init__(self, space_boundary, walk_dirs, obstacle = None):
        self.space_boundary = space_boundary
        self.walk_dirs = walk_dirs
        self.map = np.zeros(space_boundary,  dtype=State)
        self.obstacle = obstacle
        self.init_map()


    def init_map(self):
        for x in np.arange(self.space_boundary[0]):
            for y in np.arange(self.space_boundary[1]):
                for z in np.arange(self.space_boundary[2]):
                    self.map[x, y, z] = State(np.array([x, y, z]))

    def find_next_nodes(self, state):
        available = []
        for direction in self.walk_dirs:
            new_loc = state.loc + direction
#            if np.array_equal(new_loc, np.array([4, 6, 2])):
#                print("self.obstacle: ", self.obstacle)
            if check_in_area(new_loc, self.space_boundary) and not is_exist_same_loc(new_loc, self.obstacle):
                n = self.map[new_loc[0], new_loc[1], new_loc[2]]
                available.append(n)
        return available


class Dstar:
    """
    The Focussed D* Algorithm for Real-Time Replanning:
    http://www.cs.cmu.edu/afs/cs/Web/People/motionplanning/papers/sbp_papers/s/stentz_D2.pdf
    http://www.doc88.com/p-9582112351454.html
    """
    def __init__(self, space_map, space_boundary):
        """
        start and goal are in degree
        """
        self.space_map = space_map
        self.space_boundary = space_boundary
        self.openset = set()
        self.closedset = set()
        self.obstacle = space_map.obstacle

    def get_min_state(self):
        if not self.openset:
            return None
        else:
            min_state = min(self.openset, key=lambda x: x.k)
            return min_state

    def get_min_k(self):
        min_s = self.get_min_state()
        if min_s is not None:
            return min_s.k
        else:
            return -1

    def delete(self, state):
        state.t = "CLOSED"
        self.openset.remove(state)

    def insert(self, state, h_new):
        if state.t == "NEW":
            state.k = h_new
        else:
            if state.t == "OPEN":
                state.k = min(state.k, h_new)
            else:
                #"CLOSED"
                state.k = min(state.h, h_new)
        state.h = h_new
        state.t = "OPEN"
        self.openset.add(state)

    def modify_cost(self, x, y):
        if x.t == "CLOSED":
            #if is_obstacle == True, the cost(x, y) will be inf
            self.insert(x, y.h + y.cost(x))
        return self.get_min_k()

    def process_state(self):
        x = self.get_min_state()
        if x is None:
            return -1
        #get the cost last time it was on the openset
        k_x = x.k
        self.delete(x)
        if k_x < x.h:
            #Raise state, path cost increased,

            for y in self.space_map.find_next_nodes(x):
                #if x.h == np.inf, it means the backpoint of x is an obstacle and must be replaced.
                if y.t != "NEW" and (y.h <= k_x or x.h == np.inf)and x.h > y.h + x.cost(y):
                    x.bp = y
                    x.h = y.h + y.cost(x)
        elif k_x == x.h:
            #Lower state, it means found a path with lesser cost
            for y in self.space_map.find_next_nodes(x):
                if y.t == "NEW" or (y.bp == x and y.h != x.h + x.cost(y)) \
                        or (y.bp != x and y.h > x.h + x.cost(y)):
                    y.bp = x
                    self.insert(y, x.h + x.cost(y))

        else:
            #
            for y in self.space_map.find_next_nodes(x):
                if(y.t == "NEW") or (y.bp == x and y.h != x.h + x.cost(y)):
                    y.bp = x
                    self.insert(y, x.h + x.cost(y))
                else:
                    if y.bp != x and y.h > x.h + x.cost(y) and x.t == "CLOSED":
                        self.insert(x, x.h)
                    elif y.bp != x and x.h > y.h + y.cost(x) \
                        and y.t == "CLOSED" and k_x < y.h:
                        self.insert(y, y.h)

        return self.get_min_k()

    def get_init_path(self, start_s, goal_s):
        self.openset.clear()
        self.openset.add(goal_s)
        while start_s.t != "CLOSED":
            #get the path with few information( unknown obstacle)
            self.process_state()
        path = start_s.loc
        tmp_state = start_s.bp
        while tmp_state != goal_s:
            path = np.vstack((path, tmp_state.loc))
            tmp_state = tmp_state.bp
        path = np.vstack((path, goal_s.loc))
        return path.T


    def re_cal_path(self, start_s, goal_s):
        R = start_s
        while(R!=goal_s):
            #check whether the next state is overstep or obstacle
            if is_exist_same_loc(R.bp.loc, self.obstacle):
                R.bp.is_obstacle = True
                val = self.modify_cost(R, R.bp)
#                print(val, R.h, R.k)
                while ((val <= R.h) and (val != -1)):
                    val = self.process_state()
            R = R.bp
        path = start_s.loc
        tmp_state = start_s.bp
        while tmp_state != goal_s:
            path = np.vstack((path, tmp_state.loc))
            tmp_state = tmp_state.bp
        path = np.vstack((path, goal_s.loc))
        return path.T

    def search(self, start, goal, obstacle):
        goal_s = self.space_map.map[goal[0], goal[1], goal[2]]
        start_s = self.space_map.map[start[0], start[1], start[2]]

        init_path = self.get_init_path(start_s, goal_s)

        if start_s.t == "NEW":
            return None
        self.obstacle = obstacle
        self.space_map.obstacle = obstacle
        re_path = self.re_cal_path(start_s, goal_s)

        return init_path, re_path
