import numpy as np
import networkx as nx
import pickle
import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")

from mathtools.timer import Timer
from mathtools.polytope import Polytope
from mathtools.walkable import *
from environment.urdfparser import URDFtoPolytopes
from mathtools.linalg import *
from mathtools.plotter import Plotter

import sys

###############################################################################
# CONFIGURE / PARAMETERS 
###############################################################################
from robot.robotspecifications import *


###############################################################################
def walkableSurfacesPlotter(env_fname):
        timer = Timer()
        pobjects = URDFtoPolytopes(env_fname)

        print "================================================================"
        print "Loaded environment",env_fname
        print " >> Objects:",len(pobjects)
        print "================================================================"

        output_folder = os.environ["MPP_PATH"]+"mpp-environment/output/"
        wsurfaces = pickle.load( open( output_folder+"/wsurfaces.dat", "rb" ) )

        plot=Plotter()

        timer.stop()
        for i in range(0,len(pobjects)):
                V = pobjects[i].getVertexRepresentation()
                plot.polytopeFromVertices(V, fcolor=COLOR_SCENE)

        for i in range(0,len(wsurfaces)):
                W = wsurfaces[i]
                V = W.getVertexRepresentation()
                plot.walkableSurface( V, fcolor=COLOR_WALKABLE_SURFACE, thickness=0.01)

        plot.set_view(59,56)
        plot.ax.set_aspect('equal', 'datalim')
        #plot.ax.set_xlim(-4, 4)
        #lot.ax.set_ylim(0, 10)
        #lot.ax.set_zlim(0, 3)
        #lot.point([0,0,2.2],color=COLOR_START_POINT)
        #lot.point([0,8,2.2],color=COLOR_START_POINT)

        plot.fig.show()


if __name__ == "__main__":
        folder = os.environ["MPP_PATH"]+"mpp-environment/urdf/"
        #env_fname = folder+"staircase_stones.urdf"
        env_fname = folder+"quatro_homotopy.urdf"
        walkableSurfacesPlotter(env_fname)
