# -*- coding: utf-8 -*-
"""
Created on Mon Aug 13 10:41:24 2018

@author: Morgan.Li
"""

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def draw_path(space_boundary, ptsData, pathsData, targData):
    colors = ['b', 'c', 'm', 'y', 'b', 'w']
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_xlim3d([0.0, space_boundary[0]])
    ax.set_xlabel('X')

    ax.set_ylim3d([0.0, space_boundary[1]])
    ax.set_ylabel('Y')

    ax.set_zlim3d([0.0, space_boundary[2]])
    ax.set_zlabel('Z')

    ax.set_title('3D Path Plan')
    ax.scatter(ptsData[0, :], ptsData[1, :], ptsData[2, :], c= 'g', marker = '^', label='Source')
    ax.scatter(targData[0, :], targData[1, :], targData[2, :], c= 'r', marker = 'x', label='Dest')
    i = 0
    for key in pathsData:
        pathData = pathsData[key]
        plt.plot(pathData[0, :], pathData[1, :], pathData[2, :], colors[i], label= key)
        i += 1
    ax.legend()

    plt.show()




