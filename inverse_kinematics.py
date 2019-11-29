import numpy as np
import numpy.linalg
from scipy.optimize import fmin_bfgs
from pinocchio import forwardKinematics, log, neutral
import eigenpy
eigenpy.switchToNumpyMatrix()

class CallbackLogger:
     def __init__(self, ik):
          self.nfeval = 1
          self.ik = ik
     def __call__(self,x):
          print '===CBK=== {0:4d}   {1}'.format(self.nfeval,
                                                self.ik.latestCost)
          self.nfeval += 1

class InverseKinematics (object):
    leftFootJoint = 'left_leg_6_joint'
    rightFootJoint = 'right_leg_6_joint'
    waistJoint = 'waist_joint'

    def __init__ (self, robot):
        self.q = neutral (robot.model)
        forwardKinematics (robot.model, robot.data, self.q)
        self.robot = robot
        # Initialize references of feet and center of mass with initial values
        self.leftFootRefPose = robot.data.oMi [robot.leftFootJointId].copy ()
        self.rightFootRefPose = robot.data.oMi [robot.rightFootJointId].copy ()
        self.waistRefPose = robot.data.oMi [robot.waistJointId].copy ()

    def cost (self, q):
        # Write your code here
        pass

    def solve (self, q):
        # Write your code here
        pass
