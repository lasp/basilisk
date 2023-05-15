from prototype.KinematicsArchitecture.KinematicsEngine import *
from prototype.KinematicsArchitecture.Vector import Vector
from prototype.KinematicsArchitecture.AttitudeParameterization import MRP


class Frame:
    def __init__(self, attitude_SP, omega_SP_S, omegaDot_SP_S, r_SP_P, rDot_SP_P, rDDot_SP_P, parentFrame):
        # Create non-initialized fields
        self.parentFrame = parentFrame
        # self.inertial = self.parentFrame.parentFrame is None

        # Initialize variables from constructor
        self.r_SP_P = Vector(r_SP_P, self.parentFrame)
        self.rDot_SP_P = Vector(rDot_SP_P, self.parentFrame)
        self.rDDot_SP_P = Vector(rDDot_SP_P, self.parentFrame)
        self.sigma_SP = attitude_SP.toMRP()
        self.omega_SP_S = Vector(omega_SP_S, self)
        self.omegaDot_SP_S = Vector(omegaDot_SP_S, self)

    def addParentFrame(self, parentFrame):
        self.parentFrame = parentFrame
