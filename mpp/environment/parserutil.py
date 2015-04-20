import sys,os
import numpy as np
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")
import re
from environment.daeparser import DAEtoPolytopes

def uriToPolytope(uri,pose):
        envfolder = os.environ["MPP_PATH"]+"mpp-environment/"
        if re.search('dae$', uri):
                m= re.search('(model://)(.*)$',uri)
                #daefile = m.group(0,2)[1]
                daefile = envfolder+"urdf/"+m.group(0,2)[1]
                #print "parsing",daefile
                ## according to SDF definition 
                ## (https://bitbucket.org/osrf/gazebo/pull-request/235/fixes-for-issues-329-142-related-to-sdf/diff#chg-gazebo/sdf/interface/SDF.cc)

                X = pose.split(' ')
                pose = np.array(X).astype(float)

                d = DAEtoPolytopes(daefile,pose)
                polytope = d[0]
                return polytope
        else:
                print "found uri",uri,"but no known parser"
                sys.exit(1)

