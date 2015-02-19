import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
from BeautifulSoup import BeautifulSoup
from scipy.spatial import ConvexHull
import re
import numpy as np
from mathtools.polytope import Polytope
import collada
from collada import *

from pylab import *
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt

class DaeParser():

        def __init__(self,env_fname):
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
                Vm = np.mean(pts,axis=0)
                print V.shape,Vm

                M = V.shape[0]
                faces = []

                A = []
                B = []
                Vc = []
                for i in range(0,M):
                        v = np.array(V[i,:])
                        faces.append(V[i,:])
                        ###compute normals of plane
                        v0 = V[i,:][0]
                        v1 = V[i,:][1]
                        v2 = V[i,:][2]
                        a = np.cross(v2-v0,v1-v0)
                        a = a/np.linalg.norm(a)
                        b = np.dot(a,v0)

                        if np.dot(a,Vm)>b:
                                a = -a
                                b = -b #np.dot(a,v0)

                        A.append(a)
                        B.append(b)
                        vc = np.mean(V[i,:],axis=0)
                        Vc.append(vc)

                ### reduce A,b
                Ar = []
                Br = []
                Vr = []
                Ar.append(A[0])
                Br.append(B[0])
                Vr.append(Vc[0])

                for i in range(1,len(A)):

                        a = A[i]
                        b = B[i]
                        doubleEntry = False
                        for j in range(0,len(Ar)):
                                at = Ar[j]
                                bt = Br[j]
                                if np.linalg.norm(a-at)<0.001 and np.linalg.norm(b-bt)<0.001:
                                        doubleEntry=True
                                        break

                        if not doubleEntry:
                                Ar.append(A[i])
                                Br.append(B[i])
                                Vr.append(Vc[i])

                fig=figure(1)
                ax = fig.gca(projection='3d')
                for i in range(0,len(Ar)):
                        a = Ar[i]
                        b = Br[i]
                        vc = Vr[i]
                        ac = vc+a
                        ax.plot([vc[0],ac[0]],[vc[1],ac[1]],[vc[2],ac[2]],'-r')

                items = Poly3DCollection(faces)
                ax.add_collection(items)
                plt.show()

if __name__=='__main__':
        envfolder = os.environ["MPP_PATH"]+"mpp-environment/"
        #env_fname = envfolder+"urdf/quatro_homotopy.urdf"
        #env_fname = envfolder+"urdf/steppingstoneplatform.dae"
        env_fname = envfolder+"urdf/steppingstone5cm_v2.dae"
        env_fname = envfolder+"urdf/steppingstoneplatform_v2.dae"
        DaeParser(env_fname)
