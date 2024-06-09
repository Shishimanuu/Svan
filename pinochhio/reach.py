import pinocchio as pin
import numpy as np
from numpy.linalg import solve

def step(model,data,q,JOINT_ID,oMdes):
    DT     = 1e-1
    damp   = 1e-12

    pin.forwardKinematics(model,data,q)
    endPos = data.oMi[JOINT_ID]
    iMd = endPos.actInv(oMdes)
    err = pin.log(iMd).vector  # in joint frame

    J = pin.computeJointJacobian(model,data,q,JOINT_ID)  # in joint frame
    J = -np.dot(pin.Jlog6(iMd.inverse()), J)
    v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
    q = pin.integrate(model,q,v*DT)
    f = pin.log(endPos).vector
    
    return q,f,err