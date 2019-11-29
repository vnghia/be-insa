from pinocchio.utils import *
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
from pinocchio import SE3, Model, Inertia, JointModelFreeFlyer, JointModelRX, \
    JointModelRY, JointModelRZ, forwardKinematics, neutral
import gepetto.corbaserver
from display import Display

class Visual:
    '''
    Class representing one 3D mesh of the robot, to be attached to a joint. The class contains:
    * the name of the 3D objects inside Gepetto viewer.
    * the ID of the joint in the kinematic tree to which the body is attached.
    * the placement of the body with respect to the joint frame.
    This class is only used in the list Robot.visuals (see below).
    '''
    def __init__(self,name,jointParent,placement):
        self.name = name                  # Name in gepetto viewer
        self.jointParent = jointParent    # ID (int) of the joint
        self.placement = placement        # placement of the body wrt joint, i.e. bodyMjoint
    def place(self,display,oMjoint):
        oMbody = oMjoint*self.placement
        display.place(self.name,oMbody,False)

class Robot:
    '''
    Define a class Robot representing a biped robot
    The members of the class are:
    * viewer: a display encapsulating a gepetto viewer client to create 3D
      objects and place them.
    * model: the kinematic tree of the robot.
    * data: the temporary variables to be used by the kinematic algorithms.
    * visuals: the list of all the 'visual' 3D objects to render the robot,
      each element of the list being an object Visual (see above).
    '''

    def __init__(self):
        self.viewer = Display()
        self.visuals = []
        self.model = Model ()
        self.createLeggedRobot ()
        self.data = self.model.createData()
        self.q0 = neutral (self.model)

    def createLeggedRobot (self,rootId=0, prefix=''):
        # Write your code here
        pass

    def display(self,q):
        forwardKinematics(self.model,self.data,q)
        for visual in self.visuals:
            visual.place( self.viewer,self.data.oMi[visual.jointParent] )
        self.viewer.viewer.gui.refresh()
