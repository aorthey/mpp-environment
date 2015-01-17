import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")
from environment.urdfparser import URDFtoPolytopes
from mathtools.plotter import Plotter
import pickle
import numpy as np
from math import sqrt

## COLORS
colorScene=(0.1,0.1,0.1,0.5)
colorBodyBox=(1,0,0,0.5)
colorWalkableSurface=(0.4,0.4,0.4,0.3)

### LOAD ENVIRONMENT SURFACES AND STACK
folder = os.environ["MPP_PATH"]+"mpp-environment/output"
Wsurfaces_decomposed = pickle.load( open( folder+"/wsurfaces.dat", "rb" ) )
connectorUpperBody = pickle.load( open( folder+"/connector_vstack.dat", "rb" ) )
connector = pickle.load( open( folder+"/connector.dat", "rb" ) )

### LOAD ROBOT START/GOAL SPECIFICATIONS
#robot_folder = os.environ["MPP_PATH"]+"mpp-robot/output"
#qgoal = pickle.load( open( robot_folder+"/qgoal.dat", "rb" ) )
#qstart = pickle.load( open( robot_folder+"/qstart.dat", "rb" ) )

plot=Plotter()

N_w = len(Wsurfaces_decomposed)
for i in range(0,N_w):
        V = Wsurfaces_decomposed[i].getVertexRepresentation() - 0.025*np.array((0,0,1))
        plot.walkableSurface( V,\
                        fcolor=colorWalkableSurface, thickness=0.05)

###############################################################################
### plot connectors
###############################################################################
for i in range(0,len(connectorUpperBody)):
        cstack = connectorUpperBody[i][0]
        Nk = connectorUpperBody[i][1]
        print "WS",i,"has",Nk,"layers"
        for j in range(0,Nk):
                V = cstack[j].getVertexRepresentation()
                plot.polytopeFromVertices(V,fcolor=colorBodyBox)

plot.showEnvironment()
