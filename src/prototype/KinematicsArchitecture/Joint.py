from prototype.KinematicsArchitecture.Hinge import Hinge


class Joint:
    def __init__(self):
        self.lowerFrame = None
        self.upperFrame = None

    def connectPartToLowerConnectionPoint(self, part):
        self.lowerFrame.addParentFrame(part.frame)

    def connectPartToUpperConnectionPoint(self, part):
        part.frame.addParentFrame(self.upperFrame)


class Joint1D(Joint):
    def __init__(self, kinematicsEngine, equilibriumFrame, spinAxis, thetaInit, thetaDotInit, k, c):
        super().__init__()

        # Create the frame
        self.hinge = Hinge(kinematicsEngine, equilibriumFrame, spinAxis, thetaInit, thetaDotInit, k, c)

        # Populate lower and upper frames
        self.lowerFrame = self.hinge.equilibriumFrame
        self.upperFrame = self.hinge.currentFrame
