import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")
from environment.boxUrdfFactory import *

folder = os.environ["MPP_PATH"]+"mpp-environment/urdf/"
fname = folder+"staircase_stones.urdf"
f = open(fname,"wb")

b=boxUrdfFactory()

f.write(b.header())
f.write(b.box2d(0,0,1,2.5,0.9))

### homotopy 1
sy = 0.95
f.write(b.box2d(-1.5, 1.0, 1.03, 1.5, sy))
f.write(b.box2d(-2.0, 2.0, 1.08, 1.2, sy))
f.write(b.box2d(-3.0, 3.0, 1.11, 0.9, sy))
f.write(b.box2d(-2.5, 4.0, 1.11, 1.6, sy))
f.write(b.box2d(-2.0, 5.0, 1.09, 1.2, sy))
f.write(b.box2d(-1.0, 6.0, 1.05, 1.1, sy))
f.write(b.box2d(-1.5, 7.0, 1.02, 1.0, sy))

### homotopy 2
f.write(b.box2d(+1.5, 1.0, 0.97, 1.5, sy))
f.write(b.box2d(+1.0, 2.0, 0.93, 1.2, sy))
f.write(b.box2d(+1.0, 3.0, 0.89, 0.9, sy))
f.write(b.box2d(+1.5, 4.0, 0.87, 1.6, sy))
f.write(b.box2d(+1.0, 5.0, 0.91, 1.2, sy))
f.write(b.box2d(+1.0, 6.0, 0.94, 1.1, sy))
f.write(b.box2d(+1.5, 7.0, 0.98, 1.0, sy))

###goal box
f.write(b.box2d(0,8,1,3.0,0.9))
f.write(b.footer())
f.close()

print "================================================================"
print "build URDF ",fname
print " >> contains",b.getCtr(),"boxes"
print "================================================================"
