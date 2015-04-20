import numpy as np
import pickle
import sys,os,re
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")

from environment.sdfparser import SDFtoPolytopes
from environment.daeparser import DAEtoPolytopes
from environment.urdfparser import URDFtoPolytopes
from environment.scenePlotter import *

def fileToPolytopes(env_fname):
        ##analyse file extension

        urdf = re.search('urdf$', env_fname)
        if urdf:
                return URDFtoPolytopes(env_fname)

        dae = re.search('dae$', env_fname)
        if dae:
                return DAEtoPolytopes(env_fname)

        sdf = re.search('sdf$', env_fname)
        if sdf:
                return SDFtoPolytopes(env_fname)

        print "environment filename ",env_fname,"does not match any known file extension"
        print "currently supported: URDF|DAE|SDF"

        return None

if __name__ == "__main__":
        folder = os.environ["MPP_PATH"]+"mpp-environment/urdf/"
        #env_fname = folder+"staircase_stones.urdf"
        env_fname = folder+"quatro_homotopy.urdf"
        #env_fname = folder+"koroibot_staircases/stairplatform_v2.dae"

        env_fname = folder+"drc/meshes/cinder_block_wide.dae"
        obj = fileToPolytopes(env_fname)

        plot=Plotter()
        scenePlotterObjects(plot,obj)
        plot.show()
