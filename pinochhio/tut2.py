import pinocchio as pin
from robot_descriptions.loaders.pinocchio import load_robot_description
import reach
import numpy as np
from numpy.linalg import norm
from time import sleep

robot = load_robot_description("romeo_description")
model = robot.model
data = model.createData()

q = pin.neutral(model)

robot.initViewer()
robot.loadViewerModel()
robot.display(q)
robot.displayVisuals(True)

pick = (0.3,0.2,0.3)
rgbt = [1.0, 0.2, 0.2, 1.0]  # red, green, blue, transparency
robot.viewer.gui.addSphere("world/sphere", 0.1, rgbt)  # .1 is the radius
robot.viewer.gui.refresh()
robot.viewer.gui.applyConfiguration("world/sphere", (pick[0],pick[1],pick[2],1,0.,0.,0. ))
robot.viewer.gui.refresh()  # Refresh the window.

JOINT_ID = 8
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
 
