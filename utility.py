# -*- coding: utf-8 -*-
"""
Created on Mon Aug 13 09:35:33 2018

@author: Morgan.Li
"""
import numpy as np

#def check_in_area(arr, x_limit, y_limit, z_limit):
#    if arr[0] >= x_limit[0] and arr[0] <= x_limit[1]:
#        if arr[1] >= y_limit[0] and arr[1] <= y_limit[1]:
#            if arr[2] >= z_limit[0] and arr[2] <= z_limit[1]:
#                return True
#    return False

def check_in_area(loc, space_boundary):
    if loc[0] >= 0 and loc[0] < space_boundary[0]:
        if loc[1] >= 0 and loc[1] < space_boundary[1]:
            if loc[2] >= 0 and loc[2] < space_boundary[2]:
                return True
    return False


def check_in_area_with_R(arr1, arr2, r):
    if np.max(np.abs(arr1 - arr2)) <= r:
        return True
    return False


def is_exist_same_loc(loc, agents_pos):
    if agents_pos is None:
        return False
    idxes_match = (agents_pos[0, :] == loc[0])&(agents_pos[1, :] == loc[1])&\
                    (agents_pos[2, :] == loc[2])

    pos_in_area = np.where(idxes_match == True)[0]
    if pos_in_area:
        return True
    return False


def remove_same_loc(loc, agents_pos):
    if agents_pos is not None:
        idxes_match = (agents_pos[0, :] == loc[0])&(agents_pos[1, :] == loc[1])&\
                        (agents_pos[2, :] == loc[2])

        pos_in_area = np.where(idxes_match == True)[0]
        agents_pos = np.delete(agents_pos, pos_in_area, 1)
    return agents_pos

def cal_Manhattan_dist(source, dest, unit_c = 1):
    dist = np.sum(np.abs(dest - source))
    return unit_c*dist


def cal_Euclidean_dist(source, dest, unit_c = 1):
    dist = np.linalg.norm(dest - source)
    return unit_c*dist