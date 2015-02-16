import numpy as np
import networkx as nx
import pickle
import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")

from timeit import default_timer as timer

from mathtools.polytope import Polytope
from mathtools.walkable import WalkableSurface, WalkableSurfacesFromPolytopes
from mathtools.walkable import ProjectPolytopesDownInsideBox
from mathtools.walkable import getStartGoalWalkableSurfaces
from environment.urdfparser import URDFtoPolytopes
from mathtools.linalg import intersection
from mathtools.linalg import distanceWalkableSurfaceWalkableSurface
from mathtools.linalg import distancePointPolytope
from mathtools.linalg import distancePolytopePolytope
from mathtools.linalg import distanceWalkableSurfacePolytope
from mathtools.linalg import projectPointOntoHyperplane
from mathtools.linalg import maxDistanceBetweenVertices
from scipy.spatial import ConvexHull

import sys

###############################################################################
# CONFIGURE / PARAMETERS 
###############################################################################
from robot.robotspecifications import *

start = timer()

#env_fname = "urdf/wall_simplified.urdf"
#env_fname = "urdf/wall.urdf"
env_fname = "urdf/staircase_stones.urdf"

###############################################################################
pobjects = URDFtoPolytopes(env_fname)
print "----------------------------------------------------------------"
print "Loaded environment",env_fname
print " -- Objects:",len(pobjects)

wsurfaces = WalkableSurfacesFromPolytopes(pobjects)

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

        #######################################################################
        # Create smaller walkable surfaces from the clipped surfaces
        #######################################################################

        for j in range(0,len(p)):
                Wsplit = WalkableSurface.fromVertices(ap,bp,p[j],iObject)
                Wsurfaces_decomposed.append(Wsplit)

print "splitted",len(wsurfaces),"walkable surfaces into",\
                len(Wsurfaces_decomposed),"(reasoning about foot placement)"
print "----------------------------------------------------------------"


###############################################################################
# Get connectivity graph
###############################################################################

N = len(Wsurfaces_decomposed)

WD = np.zeros((N,N))
WM = np.zeros((N,N))

G_S = nx.Graph()
for i in range(0,N):
        G_S.add_edge(i,i)
        for j in range(i+1,N):
                WD[i,j]=WD[j,i]=distanceWalkableSurfaceWalkableSurface(\
                                Wsurfaces_decomposed[i], \
                                Wsurfaces_decomposed[j])
                WM[i,j]=WM[j,i]=(0 if WD[i,j]>MIN_DISTANCE_WALKABLE_SURFACES else 1)
                if WM[i,j]>0:
                        G_S.add_edge(i,j)

        WM[i,i]=1


for i in range(0,N):
        print "WS",i,"has neighbors:",G_S.neighbors(i)

###############################################################################
# create complete box over walkable surface, in which swept volume has to lie
###############################################################################

Wsurface_box = []
Wsurface_box_extension = []
for i in range(0,N):
        box = Wsurfaces_decomposed[i].createBox(0.01, \
                        ROBOT_VOLUME_MAX_HEIGHT)
        box_ext = Wsurfaces_decomposed[i].createBox(0.01, \
                        ROBOT_VOLUME_MAX_HEIGHT,\
                        DeltaSide=ROBOT_MAX_UPPER_BODY_DISTANCE_FROM_FOOT)
        Wsurface_box.append(box)
        Wsurface_box_extension.append(box_ext)

Wsurface_objects = []
for i in range(0,N):
        objects_in_box_i = []
        for j in range(0,len(pobjects)):
                O = pobjects[j]
                B = Wsurface_box_extension[i]
                d = distancePolytopePolytope(O,B)
                if d < 0.0001:
                        objects_in_box_i.append(O)
        Wsurface_objects.append(objects_in_box_i)
print "----------------------------------------------------------------"
print "Objects on top of walkable surfaces:"
print "----------------------------------------------------------------"
for i in range(0,N):
        print "WS",i,"has",len(Wsurface_objects[i]),"objects associated"
print "----------------------------------------------------------------"

###############################################################################
# Compute stack of boxes on top of each walkable surface
# each stack will act as a constraint on the swept volume path optimization
###############################################################################

Wsurface_box_vstack = []

for i in range(0,N):
        bottomHeight = VSTACK_DELTA
        W = Wsurfaces_decomposed[i]
        objs = Wsurface_objects[i]
        nghbrs = G_S.neighbors(i)
        for j in range(0,len(nghbrs)):
                k = nghbrs[j]
                print "removing WS",k,"from WS",i
                box = Wsurface_box[k]
                objs.append(box)

        Wi_box_vstack=[]
        foot_box = W.createBox(0,bottomHeight)
        foot_stack = []
        foot_stack.append(foot_box)
        Wi_box_vstack.append(foot_stack)

        while bottomHeight<ROBOT_VOLUME_MAX_HEIGHT:
                ap = W.ap
                bp = W.bp+bottomHeight

                iObject=W.iObject

                delta_box = W.createBox(\
                                bottomHeight,\
                                bottomHeight+VSTACK_DELTA,\
                                DeltaSide=ROBOT_MAX_UPPER_BODY_DISTANCE_FROM_FOOT)

                pprime = ProjectPolytopesDownInsideBox(\
                                objs,\
                                W, \
                                delta_box)

                hstack = []
                for k in range(0,len(pprime)):

                        Ksplit = WalkableSurface.fromVertices(\
                                        ap,bp,pprime[k],iObject)

                        ##new polytope can be outside of walkable surface box
                        ## in which case there is no connection, and we have to
                        ## discard it
                        dwp = distanceWalkableSurfacePolytope(Ksplit,\
                                        Wsurface_box[i].A,\
                                        Wsurface_box[i].b-0.01)

                        if dwp <= 0.001:
                                d = maxDistanceBetweenVertices(Ksplit.getVertexRepresentation())
                                ### TODO: check if object is changing topology of the box
                                box = Ksplit.createBox(0,VSTACK_DELTA)
                                hstack.append(box)

                if not len(hstack)>0:
                        ## layer is not existent, i.e. there is an object which
                        ## blocks the layer completely, for example a roof
                        ## => break and goto next walkable surface
                        break

                Wi_box_vstack.append(hstack)
                bottomHeight+=VSTACK_DELTA

        Wsurface_box_vstack.append(Wi_box_vstack)
        print "WS",i,"got Vstack of size",len(Wi_box_vstack)

###############################################################################
## compute connectors
###############################################################################
connector = []
upperBodyConnector = []

print "----------------------------------------------------------------"
print "Computing Connector Elements"
print "----------------------------------------------------------------"
for i in range(0,N):
        nghbrs = G_S.neighbors(i)
        for k in range(0,len(nghbrs)):
                j = nghbrs[k]
                if j>i:
                        W=Wsurfaces_decomposed[i]
                        Wnext=Wsurfaces_decomposed[j]

                        Wstack = Wsurface_box_vstack[i]
                        WstackNext = Wsurface_box_vstack[j]

                        #### create foot connector
                        boxW = W.createBox(0,ROBOT_FOOT_HEIGHT,DeltaSide=0.02)
                        boxWnext = Wnext.createBox(0,ROBOT_FOOT_HEIGHT)
                        binter = boxW.intersectWithPolytope(Wnext)
                        ap = Wnext.ap
                        bp = Wnext.bp
                        connectorWtoWnext = WalkableSurface(ap,bp,binter.A, binter.b, Wnext.iObject)
                        connector.append([connectorWtoWnext,i,j])

                        ### create upper body connectors
                        upperBodyConnectorStack = []

                        Nk = min(len(Wstack),len(WstackNext))
                        for p in range(0,XSPACE_DIMENSION):
                                if p < Nk:
                                        Winter = Wstack[p][0].intersectWithPolytope(WstackNext[p][0])
                                        d = maxDistanceBetweenVertices(Winter.getVertexRepresentation())
                                        if d>0.1:
                                                upperBodyConnectorStack.append(Winter)
                                        else:
                                                Whelper = connectorWtoWnext.createTinyHelperBox(p*VSTACK_DELTA,(p+1)*VSTACK_DELTA)
                                                upperBodyConnectorStack.append(Whelper)
                                                Nk=p
                                else:
                                        ## create helper box for higher dimensions (the small
                                        ## boxes which contain only 1 point
                                        Whelper = connectorWtoWnext.createTinyHelperBox(p*VSTACK_DELTA,(p+1)*VSTACK_DELTA)
                                        upperBodyConnectorStack.append(Whelper)

                        print "formed connector between WS",i,"and",j,"(has",Nk,"layers)"
                        upperBodyConnector.append([upperBodyConnectorStack,Nk,i,j])


###############################################################################
### OUTPUT
### S -- walkable surfaces
### E -- stack on top of S
### I -- stack of intersections, i.e. E_i \cup E_j
### G_S -- the connectivity graph
###############################################################################

output_folder = "output"
pickle.dump( Wsurfaces_decomposed, open( output_folder+"/wsurfaces.dat", "wb" ) )
pickle.dump( Wsurface_box_vstack, open( output_folder+"/wsurfaces_vstack.dat", "wb" ) )
pickle.dump( upperBodyConnector, open( output_folder+"/connector_vstack.dat", "wb" ) )
pickle.dump( connector, open( output_folder+"/connector.dat", "wb" ) )
pickle.dump( G_S, open( output_folder+"/graph.dat", "wb" ) )

end = timer()
ts= np.around(end - start,2)
tm= int(ts/60)
tms = np.around(ts - tm*60,2)

print "================================================================"
print "Time elapsed for building free space decomposition:"
print "================="
print ts,"s"
print "================="
print tm,"m",tms,"s"
print "================================================================"
