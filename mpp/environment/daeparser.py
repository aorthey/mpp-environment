import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
from BeautifulSoup import BeautifulSoup
from scipy.spatial import ConvexHull
import re
import numpy as np
from mathtools.polytope import Polytope
import collada
from collada import *
#from environment.scenePlotter import *
from mathtools.plotter import Plotter

from pylab import *
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
from mathtools.linalg import R_RPY

def ptsTransform(pts, pose):
        [roll,pitch,yaw]= pose[3:]
        R = R_RPY(roll,pitch,yaw)
        for i in range(0,pts.shape[0]):
                pts[i,:] = np.dot(R,pts[i,:])

        pts[:,0] = np.array(pose[0])+pts[:,0]
        pts[:,1] = np.array(pose[1])+pts[:,1]
        pts[:,2] = np.array(pose[2])+pts[:,2]
        return pts


def DAEtoPolytopes(env_fname, pose=[0,0,0,0,0,0]):
        mesh = Collada(env_fname)
        G = mesh.geometries[0]
        P = G.primitives[0]

        try:
                T = P.triangleset()
        except AttributeError:
                T = P

        T.generateNormals()
        V=T.vertex[T.vertex_index]

        pts = V.reshape(-1, 3)
        
        pts = ptsTransform(pts, pose)

        Vm = np.mean(pts,axis=0)

        M = V.shape[0]

        hull = ConvexHull(pts)
        E=hull.equations[0::2]
        Ah = np.array(E[0:,0:3])
        bh = np.zeros((len(Ah),1))

        for k in range(0,len(Ah)):
                bh[k] = -E[k,3]
        ###normalize
        for at in range(0,len(Ah)):
                normA = np.linalg.norm(Ah[at])
                Ah[at] = Ah[at]/normA
                bh[at] = bh[at]/normA
        p = Polytope(Ah,bh,Vm)
        plot=Plotter()
        objects = []
        objects.append(p)
        return objects


if __name__=='__main__':
        envfolder = os.environ["MPP_PATH"]+"mpp-environment/"
        #env_fname = envfolder+"urdf/quatro_homotopy.urdf"
        #env_fname = envfolder+"urdf/steppingstoneplatform.dae"
        #env_fname = envfolder+"urdf/steppingstoneplatform_v2.dae"
        env_fname = envfolder+"urdf/koroibot_stair_cases/meshes/stairplatform_v2.dae"
        env_fname = envfolder+"urdf/koroibot_stair_cases/meshes/stairmesh_v2.dae"
        env_fname = envfolder+"urdf/koroibot_stair_cases/meshes/stairplatform.dae"
        env_fname = envfolder+"urdf/koroibot_stair_cases/meshes/stairplatform_v2.dae"

        p = DAEtoPolytopes(env_fname)
