from os.path import join
from robot_descriptions.loaders.pinocchio import load_robot_description
from pinocchio import pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import rotate, zero, eye
 
import numpy as np
 
PKG = '/opt/openrobots/share'
urdf = join(PKG, 'ur5_description/urdf/ur5_gripper.urdf')
 
 
def loadRobot(M0, name):
    '''
    This function load a UR5 robot n a new model, move the basis to placement <M0>
    and add the corresponding visuals in gepetto viewer with name prefix given by string <name>.
    It returns the robot wrapper (model,data).
    '''
    robot = load_robot_description("ur5_description")
    robot.model.jointPlacements[1] = M0 * robot.model.jointPlacements[1]
    robot.visual_model.geometryObjects[0].placement = M0 * robot.visual_model.geometryObjects[0].placement
    robot.visual_data.oMg[0] = M0 * robot.visual_data.oMg[0]
    robot.initViewer(loadModel=True, sceneName="world/" + name)
    return robot
 
 
robots = []
# Load 4 Ur5 robots, placed at 0.3m from origin in the 4 directions x,y,-x,-y.
Mt = pin.SE3(eye(3), np.array([.3, 0, 0]))  # First robot is simply translated
for i in range(4):
    robots.append(loadRobot(pin.SE3(rotate('z', np.pi / 2 * i), zero(3)) * Mt, "robot%d" % i))
 
# Set up the robots configuration with end effector pointed upward.
q0 = np.array([np.pi / 4, -np.pi / 4, -np.pi / 2, np.pi / 4, np.pi / 2, 0])
for i in range(4):
    robots[i].display(q0)
 
# Add a new object featuring the parallel robot tool plate.
robots[0].initViewer()
gepettoViewer = robots[0].viewer.gui
w, h, d = 1, 0.25, 0.005
color = [red, green, blue, transparency] = [1, 0, 0.78, 1.0]
gepettoViewer.addBox('world/toolplate', w, h, d, color)
gepettoViewer.refresh()
Mtool = pin.SE3(rotate('z', 1.268), np.array([0, 0, .77]))
gepettoViewer.applyConfiguration('world/toolplate', pin.SE3ToXYZQUATtuple(Mtool))
gepettoViewer.refresh()