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
from mathtools.linalg import *
from environment.daeparser import DAEtoPolytopes

from pylab import *
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
from environment.parserutil import uriToPolytope


def SDFtoPolytopes(env_fname):
        envfolder = os.environ["MPP_PATH"]+"mpp-environment/"
        soup = BeautifulSoup(open(env_fname))
        if soup.sdf is None:
                print "[ERROR]"
                print env_fname,"seems empty or without structure"
                sys.exit(0)
        model = soup.sdf.model
        print "parsing",model["name"]
        links = model.findAll("collision")

        obj = []
        for i in range(0,len(links)):
                L=links[i]
                dae=L.uri.getText()
                if re.search('dae$', dae):
                        m= re.search('(model://)(.*)$',dae)
                        #daefile = m.group(0,2)[1]
                        daefile = envfolder+"urdf/"+m.group(0,2)[1]
                        print "parsing",daefile
                        pose = L.pose.getText()
                        ## according to SDF definition 
                        ## (https://bitbucket.org/osrf/gazebo/pull-request/235/fixes-for-issues-329-142-related-to-sdf/diff#chg-gazebo/sdf/interface/SDF.cc)

                        X = pose.split(' ')
                        pose = np.array(X).astype(float)

                        obj.append(DAEtoPolytopes(daefile,pose)[0])

                else:
                        print "uri geometry does not contain dae file"
        return obj

if __name__=='__main__':
        envfolder = os.environ["MPP_PATH"]+"mpp-environment/"
        #env_fname = envfolder+"urdf/quatro_homotopy.urdf"
        #env_fname = envfolder+"urdf/steppingstoneplatform.dae"
        #env_fname = envfolder+"urdf/steppingstoneplatform_v2.dae"
        env_fname = envfolder+"urdf/koroibot_staircases/stairplatform_v2.dae"
        env_fname = envfolder+"urdf/koroibot_staircases/stairmesh_v2.dae"
        env_fname = envfolder+"urdf/koroibot_staircases/stairplatform.dae"
        env_fname = envfolder+"urdf/koroibot_staircases/stairplatform_v2.dae"

        #env_fname = envfolder+"urdf/koroibot_stair_cases.sdf"
        #env_fname = envfolder+"urdf/koroibot_stepping_stones/koroibot_stepping_stones.sdf"
        #env_fname = envfolder+"urdf/drc/gate.sdf"
        #env_fname = envfolder+"urdf/drc/block_level_steps.sdf"
        env_fname = envfolder+"urdf/drc/block_angle_steps.sdf"

        #obj = SDFtoPolytopes(env_fname)

        #plot=Plotter()
        #scenePlotterObjects(plot,obj)
        #plot.axis.equal()
        #plot.ax.set_aspect('equal', 'datalim')

        #plot.show()
        envfolder = os.environ["MPP_PATH"]+"mpp-environment/"
        soup = BeautifulSoup(open(env_fname))
        if soup.sdf is None:
                print "[ERROR]"
                print env_fname,"seems empty or without structure"
                sys.exit(0)
        model = soup.sdf.model
        print "parsing",model["name"]
        links = model.findAll("collision")

        obj = []
        for i in range(0,len(links)):
                L=links[i]
                if L.geometry.uri:
                        uri=L.uri.getText()
                        pose=L.pose.getText()
                        obj.append(uriToPolytope(uri, pose))

                elif L.geometry.box:

                        st=L.geometry.box.size.getText()
                        size = re.split(' ',st)
                        sx = float(size[0])
                        sy = float(size[1])
                        sz = float(size[2])

                        pos=L.pose.getText()
                        pos = re.split(' ',pos)

                        x = float(pos[0])
                        y = float(pos[1])
                        z = float(pos[2])
                        ro = float(pos[3])
                        po = float(pos[4])
                        yo = float(pos[5])

                        pts = boxToPts(x,y,z,ro,po,yo,sx,sy,sz)

                        plytp = ptsToPolytope(pts)
                        obj.append(plytp)

                else:
                        print "uri geometry does not contain dae file"
                        sys.exit(1)
        if len(links) == 0:
                print "no collision links found"
                links = model.findAll("include")
                if len(links)>0:
                        for i in range(0,len(links)):
                                L=links[i]
                                if L.uri:
                                        uri=L.uri.getText()
                                        pose=L.pose.getText()
                                        p = uriToPolytope(uri,pose)
                                        obj.append(p)
                                        print L.find("name").getText()
                                else:
                                        print "no uri file found!"
                                        sys.exit(1)

                else:
                        print "no include links found"
                        print "=> no known format"
                        sys.exit(1)

        from environment.scenePlotter import *
        plot=Plotter()
        scenePlotterObjects(plot,obj)
        plot.show()
