from prototype.KinematicsArchitecture.KinematicsEngine import *


class Vector:
    def __init__(self, vectorArray, frame):
        self.vectorArray = vectorArray
        self.frame = frame


class Tensor:
    def __init__(self, tensorArray, frame):
        self.tensorArray = tensorArray
        self.frame = frame
