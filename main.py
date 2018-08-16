# -*- coding: utf-8 -*-
"""
Created on Mon Aug 13 11:30:07 2018

@author: Morgan.Li
"""

from env import Env
from astar import Astar
from dstar import Dstar, Map
from plot import draw_path
import logging, sys, random, getopt

VERSION = 0.1
use_dstar = True
if __name__ == "__main__":
    logging.basicConfig(level = logging.INFO)
    logger = logging.getLogger("pathPlan")
    logger.setLevel(logging.INFO)
    space_boundary = (10, 10, 10)
    agents_num = 50
    safe_R = 0
    seed = random.randint(0, 99999)
    use_astar = None
    use_dstar = None
    try:
        opts, args = getopt.getopt(sys.argv[1:], "vhlads:e:n:r:", ["version", "help", \
                                   "load", "astar", "dstar", "space=", "seed=", "num=", "radius="])

    except getopt.GetoptError as err:
        logger.warn(err)
        sys.exit()
    for op, value in opts:
        if op in ("-h", "--help"):
            print( "Usage main.py to calculate the flight path.\r\n \r\n", \
                  "  -h --help: help information. \r\n \r\n",\
                  "  -v --version: Get the Version. \r\n \r\n", \
                  "  -s, --space [X, Y, Z]: \r\n", \
                  "    X, Y, Z: space size along x, y and z axis, default is 10,10,10\r\n", \
                  "    Example: '-s 10,10,10' \r\n",\
                  "  -n, --num [NUMBER] \r\n", \
                  "    NUMBER: define the number of agents, default is 50 \r\n", \
                  "    Example: '-s 10,10,10 -n 100' \r\n",\
                  "  -r, --radius [SAFE_RADIUS]: \r\n", \
                  "    SAFE_RADIUS: sate radius, default is 1. \r\n", \
                  "    Example: '-s 10,10,10 -n 100 -r 2' \r\n",\
                  "  -e, --seed [SEED]: \r\n", \
                  "    SEED: set the seed of Random generator(0-99999). If not specified, it's an random integer.\r\n", \
                  "    Example: '-s 10,10,10 -n 100 -e 1527' \r\n",\
                  "  -f, --file [PATH_FILE_PATH]: \r\n", \
                  "    PATH_FILE_PATH: set file to store data, default is 'position.npz'\r\n", \
                  "    Example: '-s 10,10,10 -n 100 -f position' \r\n",\
                  "  -a, --astar: Using Astar algorithm to plan path\r\n",\
                  "    Example: '-s 10,10,10 -n 100 -a' \r\n",\
                  "  -d, --dstar: Using Dstar algorithm to plan path\r\n",\
                  "    Example: '-s 10,10,10 -n 100 -a' \r\n",\
                )
            sys.exit()
        elif op in ("-v", "--version"):
            print("Version: ", VERSION)
            sys.exit()
        elif op in ("-f", "--file"):
            filename = value
        elif op in ("-s", "--space"):
            v_l = value.split(",")
            space_boundary = [int(x) for x in v_l]
        elif op in ("-n", "--num"):
            agents_num = int(value)
        elif op in ("-r", "--radius"):
            safe_R = int(value)
        elif op in ("-e", "--seed"):
            seed = int(value)
        elif op in ("-a", "--astar"):
            use_astar = True
        elif op in ("-d", "--dstar"):
            use_dstar = True
    if use_astar is None and use_dstar is None:
        print("Error: You need to select one algorithm to plan path: Astar or Dstar!")
        sys.exit()
    else:
        print('Space: ', space_boundary)
        print('Agents number: ', agents_num)
        print('Safe Radius: ', safe_R)
        print('Seed: ', seed)
        env = Env(space_boundary = space_boundary,
                  agents_num=agents_num, seed = seed, safe_R = safe_R)
        if use_astar:
            print('Algorithm: Astar')
            astar = Astar(env.agents_pos[:, 0], env.agents_targ[:, 0], env.agents_pos,
                          env.space_boundary, env.walk_dirs)
            pathData = astar.search()
            pathsData = {"Plan Path": pathData}
            draw_path(env.space_boundary ,env.agents_pos, pathsData,
                      env.agents_targ, title = "Path Plan with A*")
        elif use_dstar:
            print('Algorithm: Dstar')
            space_map = Map(env.space_boundary, env.walk_dirs)
            dstar = Dstar(space_map, env.space_boundary)
            paths = dstar.search( env.agents_pos[:, 0], env.agents_targ[:, 0], env.agents_pos)
            pathsData = {"Path Without Obstacle": paths[0], "Path With Obstacle": paths[1]}
            draw_path(env.space_boundary ,env.agents_pos, pathsData,
                      env.agents_targ, title = "Path Plan with D*")
    print("Finish!")
    sys.exit()