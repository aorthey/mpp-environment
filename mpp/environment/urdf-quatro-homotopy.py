import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")
from environment.boxUrdfFactory import *

folder = os.environ["MPP_PATH"]+"mpp-environment/urdf/"
fname = folder+"quatro_homotopy.urdf"
f = open(fname,"wb")

b=boxUrdfFactory()

f.write(b.header())
f.write(b.box2d(0,0,1,2.5,0.9))

sy = 0.95
f.write(b.box2d(-1.5, 1.0, 1.03, 1.5, sy))
f.write(b.box2d(-2.0, 2.0, 1.05, 1.2, sy))
f.write(b.box2d(-3.0, 3.0, 1.02, 0.9, sy))

f.write(b.box2d(-2.0, 5.0, 1.02, 1.2, sy))
f.write(b.box2d(-1.0, 6.0, 1.05, 1.1, sy))
f.write(b.box2d(-1.5, 7.0, 1.02, 1.0, sy))

f.write(b.box2d(+1.5, 1.0, 0.98, 1.5, sy))
f.write(b.box2d(+1.0, 2.0, 0.96, 1.2, sy))
f.write(b.box2d(+1.0, 3.0, 0.99, 0.9, sy))

f.write(b.box2d(+1.0, 5.0, 0.98, 1.2, sy))
f.write(b.box2d(+1.0, 6.0, 0.95, 1.1, sy))
f.write(b.box2d(+1.5, 7.0, 0.98, 1.0, sy))

f.write(b.box2d(+0.0, 4.0, 1.0, 5.6, sy))
###goal box
f.write(b.box2d(0,8,1,3.0,0.9))
f.write(b.footer())
f.close()

print "================================================================"
print "build URDF ",fname
print " >> contains",b.getCtr(),"boxes"
print "================================================================"
