import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")

from environment.computeWalkableSurfaceVstackConnections import *

import sys

if __name__ == '__main__':
        env_folder = os.environ["MPP_PATH"]+"mpp-environment/urdf"
        env_fname = env_folder+"/wall-extension.urdf"
        computeWalkableSurfaceVstackConnections(env_fname)

