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
def walkableSurfacesFromSurfacePlotter(wsurfaces, pobjects=None, plotscene=True):
        plot=Plotter()
        if plotscene:
                for i in range(0,len(pobjects)):
                        V = pobjects[i].getVertexRepresentation()
                        plot.polytopeFromVertices(V, fcolor=COLOR_SCENE)

        for i in range(0,len(wsurfaces)):
                W = wsurfaces[i]
                V = W.getVertexRepresentation()
                plot.walkableSurface( V, fcolor=COLOR_WALKABLE_SURFACE, thickness=0.01)

        plot.set_view(59,56)
        plot.ax.set_aspect('equal', 'datalim')

        plot.fig.show()

def walkableSurfacesPlotter2(plot, wsurfaces):
        for i in range(0,len(wsurfaces)):
                W = wsurfaces[i]
                V = W.getVertexRepresentation()
                plot.walkableSurface( V, fcolor=COLOR_WALKABLE_SURFACE, thickness=0.01)


def walkableSurfacesPlotter(env_fname, plotscene=True):
        timer = Timer()
        pobjects = URDFtoPolytopes(env_fname)

        print "================================================================"
        print "Loaded environment",env_fname
        print " >> Objects:",len(pobjects)
        print "================================================================"

        output_folder = os.environ["MPP_PATH"]+"mpp-environment/output/"
        wsurfaces = pickle.load( open( output_folder+"/wsurfaces.dat", "rb" ) )

        timer.stop()
        walkableSurfacesFromSurfacePlotter(wsurfaces, pobjects, plotscene)



if __name__ == "__main__":
        folder = os.environ["MPP_PATH"]+"mpp-environment/urdf/"
        #env_fname = folder+"staircase_stones.urdf"
        env_fname = folder+"quatro_homotopy.urdf"
        walkableSurfacesPlotter(env_fname)
