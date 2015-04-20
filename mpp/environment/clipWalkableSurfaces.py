import numpy as np
import networkx as nx
import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")

from mathtools.polytope import Polytope
from mathtools.walkable import *
from environment.urdfparser import URDFtoPolytopes
from mathtools.linalg import *

import sys
###############################################################################
# CONFIGURE / PARAMETERS 
###############################################################################
from robot.robotspecifications import *

def clipWalkableSurfaces(wsurfaces,pobjects):
        ###############################################################################
        # create footBox, box over Sip with height of one foot
        ###############################################################################
        footBoxCandidate=[]
        for i in range(0,len(wsurfaces)):
                footBoxCandidate.append(wsurfaces[i].createBox(0.01, ROBOT_FOOT_HEIGHT))
        ###############################################################################
        # Project Objects in footBox down, create clipped surfaces
        ###############################################################################

        Wsurfaces_decomposed= []
        for i in range(0,len(wsurfaces)):
                ap = wsurfaces[i].ap
                bp = wsurfaces[i].bp
                iObject = wsurfaces[i].iObject
                p = ProjectPolytopesDownInsideBox(pobjects, wsurfaces[i],\
                                footBoxCandidate[i])
                for j in range(0,len(p)):
                        Wsplit = WalkableSurface.fromVertices(ap,bp,p[j],iObject)
                        Wsurfaces_decomposed.append(Wsplit)

        print "splitted",len(wsurfaces),"walkable surfaces into",\
                        len(Wsurfaces_decomposed),"(reasoning about foot placement)"
        print "======================================================================="
        return Wsurfaces_decomposed
