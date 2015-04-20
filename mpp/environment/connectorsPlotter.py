import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")
from environment.urdfparser import URDFtoPolytopes
from mathtools.plotter import Plotter
from robot.robotspecifications import *
import pickle
import numpy as np
from math import sqrt

### LOAD ENVIRONMENT SURFACES AND STACK

### LOAD ROBOT START/GOAL SPECIFICATIONS
#robot_folder = os.environ["MPP_PATH"]+"mpp-robot/output"
#qgoal = pickle.load( open( robot_folder+"/qgoal.dat", "rb" ) )
#qstart = pickle.load( open( robot_folder+"/qstart.dat", "rb" ) )

plot=Plotter()

def connectorsPlotter(plot, Connector_Vstack):
        for i in range(0,len(Connector_Vstack)):
                cstack = Connector_Vstack[i][0]
                Nk = Connector_Vstack[i][1]
                print "WS",i,"has",Nk,"layers"
                for j in range(0,Nk):
                        V = cstack[j].getVertexRepresentation()
                        plot.polytopeFromVertices(V,fcolor=COLOR_CONNECTOR,zorder=ZORDER_CONNECTOR)

if __name__ == '__main__':
        folder = os.environ["MPP_PATH"]+"mpp-environment/output"
        connectorUpperBody = pickle.load( open( folder+"/connector_vstack.dat", "rb" ) )
        plot=Plotter()
        connectorsPlotter(plot,connectorUpperBody)
        plot.showEnvironment()

