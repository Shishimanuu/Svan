from robot_descriptions.loaders.pinocchio import load_robot_description
import pinocchio as pin
import numpy as np
from numpy.linalg import norm
from time import sleep
import reach

robot = load_robot_description("ur5_description")
model = robot.model
data = model.createData()

robot.initViewer()

pick = (0.5,0.1,0.5)
rgbt = [1.0, 0.2, 0.2, 1.0]  # red, green, blue, transparency
robot.viewer.gui.addSphere("world/sphere", 0.1, rgbt)  # .1 is the radius
robot.viewer.gui.refresh()
robot.viewer.gui.applyConfiguration("world/sphere", (pick[0],pick[1],pick[2],1,0.,0.,0. ))
robot.viewer.gui.refresh()  # Refresh the window.

JOINT_ID = 6
oMdes = pin.SE3(np.eye(3), np.array(pick))
  
q      = pin.neutral(model)
eps    = 0.01
IT_MAX = 1000
DT     = 1e-1
damp   = 1e-12

i=0
while True:
    q,f,err = reach.step(model,data,q,JOINT_ID,oMdes)

    if norm(err) < eps:
        success = True
        break
    if i >= IT_MAX:
        success = False
        break

    robot.loadViewerModel()
    robot.display(q)
    robot.displayVisuals(True)
    sleep(0.1)

    if not i % 10:
        print('%d: error = %s' % (i, err.T))
    i += 1
 
if success:
    print("Convergence achieved!")
else:
    print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")
 
# print('\nresult: %s' % q.flatten().tolist())
# print('\nfinal error: %s' % err.T)

place = (0.5,-0.6,0.1)
oMdes = pin.SE3(np.eye(3), np.array(place))
rgbt2 = [0, 1, 0.2, 1.0]  # red, green, blue, transparency
robot.viewer.gui.addSphere("world/sphere1", 0.1, rgbt2)  # .1 is the radius
robot.viewer.gui.refresh()
robot.viewer.gui.applyConfiguration("world/sphere1", (pin.SE3ToXYZQUATtuple(oMdes)))
robot.viewer.gui.refresh()  # Refresh the window.


while True:
    q,f,err = reach.step(model,data,q,JOINT_ID,oMdes)

    if norm(err) < eps:
        success = True
        break
    if i >= IT_MAX:
        success = False
        break
    
    robot.viewer.gui.applyConfiguration("world/sphere", (f[0],f[1],f[2],1,0.,0.,0. ))

    robot.loadViewerModel()
    robot.display(q)
    robot.displayVisuals(True)
    sleep(0.1)

    if not i % 10:
        print('%d: error = %s' % (i, err.T))
    i += 1
 
if success:
    print("Convergence achieved!")
else:
    print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")




