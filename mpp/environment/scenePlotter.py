import numpy as np
import networkx as nx
import pickle
import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")

from environment.fileparser import fileToPolytopes
from mathtools.timer import Timer
from mathtools.polytope import Polytope
from mathtools.walkable import *
from mathtools.linalg import *
from mathtools.plotter import Plotter

import sys

###############################################################################
# CONFIGURE / PARAMETERS 
###############################################################################
from robot.robotspecifications import *


def scenePlotter(plot,env_fname):
        pobjects = fileToPolytopes(env_fname)
        print "================================================================"
        print "Loaded environment",env_fname
        print " >> Objects:",len(pobjects)
        print "================================================================"
        scenePlotterObjects(plot,pobjects)

def scenePlotterObjects(plot,pobjects):

        Vxmin = 10000
        Vxmax = -10000
        Vymin = 10000
        Vymax = -10000
        Vzmin = 10000
        Vzmax = -10000
        for i in range(0,len(pobjects)):
                V = pobjects[i].getVertexRepresentation()
                Vxmin = min(min(V[:,0]),Vxmin)
                Vxmax = max(max(V[:,0]),Vxmax)
                Vymin = min(min(V[:,1]),Vymin)
                Vymax = max(max(V[:,1]),Vymax)
                Vzmin = min(min(V[:,2]),Vzmin)
                Vzmax = max(max(V[:,2]),Vzmax)
                plot.polytopeFromVertices(V, fcolor=COLOR_SCENE)
        plot.ax.set_xlim(Vxmin,Vxmax)
        plot.ax.set_ylim(Vymin,Vymax)
        plot.ax.set_zlim(Vzmin,Vzmax)

if __name__ == "__main__":
        folder = os.environ["MPP_PATH"]+"mpp-environment/urdf/"
        #env_fname = folder+"staircase_stones.urdf"
        env_fname = folder+"quatro_homotopy.urdf"

        plot=Plotter()
        scenePlotter(plot,env_fname)
        plot.set_view(31,35)
        plot.showEnvironment()
