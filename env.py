# -*- coding: utf-8 -*-
"""
Created on Mon Aug 13 10:04:45 2018

@author: Morgan.Li
"""
import numpy as np
import random, logging
from node import Agent

logger = logging.getLogger("pathPlan")


class Env(object):
    def __init__(self, space_boundary, agents_init = None, agents_targ=None,
                 agents_num=None, seed=None, is_seperate=None, filename=None, safe_R=0):

        # position dimensionality
        self.dim_p = 3
        # space boundary in x, y, z axises
        self.space_boundary = space_boundary
        self.color_list = (0, 1, 2)
        self.filename = filename
        self.safe_R = safe_R
        self.walk_dirs = [np.array([1, 0, 0]), np.array([-1, 0, 0]),
                          np.array([0, 1, 0]), np.array([0, -1, 0]),
                          np.array([0, 0, 1]), np.array([0, 0, -1])]

        if agents_num is None:
            self.agents_num = agents_targ.shape[1]
        else:
            self.agents_num = agents_num
        self.agents_targ = agents_targ
        self.agents_init = agents_init
        self.reset(seed = seed, is_seperate= is_seperate)
        self.conditions = {'axis': (1,2),
                           '1': ((0,0), ('>=', '>=')), '2': ((0,0), ('<', '>=')),
                           '3': ((0,0), ('<=', '<')), '4': ((0,0), ('>', '<'))}

    def reset(self, is_keep_pos_setting=True, seed=None, is_seperate=None):
        if is_seperate is None:
            self.is_seperate = random.choice([False, True])
        else:
            self.is_seperate = is_seperate
        if seed is None:
            seed = random.randint(0, 99999)
        logger.info('seed: {0}'.format(seed))
        self.seed = seed
        np.random.seed(seed)
        self.start = False

        self.agents_pos = np.zeros((self.dim_p, self.agents_num),  dtype=np.int)
        self.curr_state  = np.full(self.space_boundary,  -1)

        if is_keep_pos_setting:
            self.make_world(self.agents_init, self.agents_targ)
        else:
            self.make_world()
        self.block_flag = False
        self.agents_path = np.expand_dims(self.agents_pos, axis=0)

    def make_world(self, agents_init=None, agents_targ = None):
        # set any world properties first
        # add agents
        self.agents = [Agent(self.space_boundary, self.walk_dirs) for i in range(self.agents_num)]
        for i, agent in enumerate(self.agents):
            agent.name = 'agent %d' % i
            agent.id = i
        # make initial conditions
        if agents_init is None:
            self.gen_inits()
        else:
            self.agents_pos =  agents_init

        if agents_targ is None:
            self.agents_targ = self.gen_targets()
        else:
            self.agents_targ = agents_targ

    def gen_inits(self):
        # set random initial states
        for agent in self.agents:
            flag = True
            while(flag):
                if self.is_seperate:
                    x = np.random.randint(0, self.space_boundary[0]/2)
                    y = np.random.randint(0, self.space_boundary[1]/2)
                    z = np.random.randint(0, self.space_boundary[2]/2)
                else:
                    x = np.random.randint(0, self.space_boundary[0])
                    y = np.random.randint(0, self.space_boundary[1])
                    z = np.random.randint(0, self.space_boundary[2])
                loc = np.array([x, y, z])
                if self.find_agents_with_R_(loc, self.safe_R, self.agents_pos).size == 0:
                    flag = False
            agent.loc = loc
            self.agents_pos[:, agent.id] =loc

    def gen_targets(self):
        agents_targ = np.zeros((self.dim_p, self.agents_num), dtype=np.int)
        for agent in self.agents:
            flag = True
            while(flag):
                if self.is_seperate:
                    x = np.random.randint(self.space_boundary[0]/2, self.space_boundary[0])
                    y = np.random.randint(self.space_boundary[0]/2, self.space_boundary[1])
                    z = np.random.randint(self.space_boundary[0]/2, self.space_boundary[2])
                else:
                    x = np.random.randint(0, self.space_boundary[0])
                    y = np.random.randint(0, self.space_boundary[1])
                    z = np.random.randint(0, self.space_boundary[2])
                loc = [x, y, z]
                if (self.find_agents_with_R_(loc, self.safe_R, agents_targ).size == 0) and\
                    (self.find_agents_with_R_(loc, self.safe_R, self.agents_pos).size == 0):
                    flag = False
            agents_targ[:, agent.id] = loc

        return agents_targ

    def find_agents_with_R_(self, loc, r, agents_pos):
        x, y, z = loc
        x_limit = np.clip([x-r, x+r], 0, self.space_boundary[0]-1)
        y_limit = np.clip([y-r, y+r], 0, self.space_boundary[1]-1)
        z_limit = np.clip([z-r, z+r], 0, self.space_boundary[2]-1)

        idxes_match = (agents_pos[0, :] >= x_limit[0])&(agents_pos[0, :] <= x_limit[1])&\
                        (agents_pos[1, :] >= y_limit[0])&(agents_pos[1, :] <= y_limit[1])&\
                        (agents_pos[2, :] >= z_limit[0])&(agents_pos[2, :] <= z_limit[1])

        pos_in_area = np.where(idxes_match == True)[0]

        return pos_in_area
