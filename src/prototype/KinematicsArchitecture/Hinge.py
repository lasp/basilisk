from prototype.KinematicsArchitecture.KinematicsEngine import *
from prototype.KinematicsArchitecture.Frame import Frame
from prototype.KinematicsArchitecture.AttitudeParameterization import PRV
import numpy as np


class Hinge:
    # Initialize the class variables
    def __init__(self, kinematicsEngine, equilibriumFrame, spinAxis, thetaInit, thetaDotInit, k, c):
        # Initialize variables from constructor
        self.equilibriumFrame = equilibriumFrame
        self.spinAxis = spinAxis
        self.theta = thetaInit
        self.thetaDot = thetaDotInit
        self.k = k
        self.c = c
        self.actuator = None

        # Set the current frame
        # WE NEED TO ADD THIS TO THE LIST OF FRAMES
        self.currentFrame = Frame(PRV(self.theta * self.spinAxis), self.thetaDot * self.spinAxis, np.array([0, 0, 0]),
                                  np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0]), self.equilibriumFrame)
        kinematicsEngine.frameList.append(self.currentFrame)

    def updateKinematicsStates(self, theta, thetaDot):
        self.theta = theta
        self.thetaDot = thetaDot

    def updateFrameStates(self):
        self.currentFrame.sigma_CP = PRV(self.theta * self.spinAxis).toMRP()
        self.currentFrame.omega_CP_C = self.thetaDot * self.spinAxis

    def connectActuator(self, actuator):
        self.actuator = actuator
