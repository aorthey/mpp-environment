import numpy as np
import networkx as nx
import pickle
import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")

from mathtools.polytope import Polytope
from mathtools.walkable import *
from environment.urdfparser import URDFtoPolytopes
from mathtools.linalg import *
from mathtools.timer import * 
from scipy.spatial import ConvexHull

import sys
###############################################################################
# CONFIGURE / PARAMETERS 
###############################################################################
from robot.robotspecifications import *


def computeWalkableSurfaceConnectivity(env_fname):
        output_folder = os.environ["MPP_PATH"]+"mpp-environment/output/"

        sum_time = 0.0
        ###############################################################################
        timer = Timer("building free space decomposition")
        pobjects = URDFtoPolytopes(env_fname)

        wsurfaces = WalkableSurfacesFromPolytopes(pobjects)
        timer.stop()
        sum_time = sum_time + timer.getTime()

        ###############################################################################
        # Get connectivity graph
        ###############################################################################
        timer = Timer("connectivity")

        N = len(wsurfaces)

        WD = np.zeros((N,N))
        WM = np.zeros((N,N))

        G_S = nx.Graph()
        for i in range(0,N):
                G_S.add_edge(i,i)
                for j in range(i+1,N):
                        WD[i,j]=WD[j,i]=distanceWalkableSurfaceWalkableSurface(\
                                        wsurfaces[i], \
                                        wsurfaces[j])
                        WM[i,j]=WM[j,i]=(0 if WD[i,j]>MIN_DISTANCE_WALKABLE_SURFACES else 1)
                        if WM[i,j]>0:
                                G_S.add_edge(i,j)

                WM[i,i]=1

        timer.stop()
        sum_time+=timer.getTime()

        ###############################################################################
        ### OUTPUT
        ### S -- walkable surfaces
        ### G_S -- the connectivity graph
        ###############################################################################

        pickle.dump( wsurfaces, open( output_folder+"/wsurfaces.dat", "wb" ) )
        pickle.dump( G_S, open( output_folder+"/graph.dat", "wb" ) )
        return sum_time

if __name__ == "__main__":
        envfolder = os.environ["MPP_PATH"]+"mpp-environment/"
        #env_fname = "urdf/wall_simplified.urdf"
        #env_fname = "urdf/wall.urdf"
        #env_fname = "urdf/staircase_stones.urdf"
        env_fname = envfolder+"urdf/quatro_homotopy.urdf"
        computeWalkableSurfaceConnectivity(env_fname)
