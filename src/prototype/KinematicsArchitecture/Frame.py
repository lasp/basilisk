from prototype.KinematicsArchitecture.KinematicsEngine import *
from prototype.KinematicsArchitecture.Vector import Vector
from prototype.KinematicsArchitecture.AttitudeParameterization import MRP


class Frame:
    def __init__(self, attitude_CP, omega_CP_C, omegaDot_CP_C, r_CP_P, rDot_CP_P, rDDot_CP_P, parentFrame):
        # Create non-initialized fields
        self.parentFrame = parentFrame
        # self.inertial = self.parentFrame.parentFrame is None

        # Initialize variables from constructor
        self.r_CP_P = Vector(r_CP_P, self.parentFrame)
        self.rDot_CP_P = Vector(rDot_CP_P, self.parentFrame)
        self.rDDot_CP_P = Vector(rDDot_CP_P, self.parentFrame)
        self.sigma_CP = attitude_CP.toMRP()
        self.omega_CP_C = Vector(omega_CP_C, self)
        self.omegaDot_CP_C = Vector(omegaDot_CP_C, self)

    def addParentFrame(self, parentFrame):
        self.parentFrame = parentFrame
