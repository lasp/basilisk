from prototype.KinematicsArchitecture.KinematicsEngine import *
from prototype.KinematicsArchitecture.Frame import Frame
from prototype.KinematicsArchitecture.Vector import Vector, Tensor


class Part:
    def __init__(self, r_CsC_P, frame, m, I):
        # Update the frame
        self.frame = frame

        self.m = m
        self.I = Tensor(I, self.frame)
        self.r_CsC_P = Vector(r_CsC_P, self.frame.parentFrame)
