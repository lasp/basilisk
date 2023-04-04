from prototype.KinematicsArchitecture.Frame import Frame
from prototype.KinematicsArchitecture.Part import Part
from prototype.KinematicsArchitecture.AttitudeParameterization import MRP
from prototype.KinematicsArchitecture.Joint import Joint1D
from prototype.KinematicsArchitecture.Hinge import Hinge
from prototype.KinematicsArchitecture.Graph import findPathAlternate, findLCA


class KinematicsEngine:
    def __init__(self):
        self.frameList = []
        self.partList = []
        self.jointList = []
        self.assemblyList = []

        self.rootFrame = Frame(MRP([0, 0, 0]), [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], None)
        self.frameList.append(self.rootFrame)

    def createFrame(self, attitude_CP=MRP([0, 0, 0]), omega_CP_C=None, omegaDot_CP_C=None,
                    r_CP_P=None, rDot_CP_P=None, rDDot_CP_P=None, parentFrame=None):
        # Default parameter handling
        if r_CP_P is None:
            r_CP_P = [0, 0, 0]
        if rDot_CP_P is None:
            rDot_CP_P = [0, 0, 0]
        if rDDot_CP_P is None:
            rDDot_CP_P = [0, 0, 0]
        if omega_CP_C is None:
            omega_CP_C = [0, 0, 0]
        if omegaDot_CP_C is None:
            omegaDot_CP_C = [0, 0, 0]
        if parentFrame is None:
            parentFrame = self.rootFrame

        # Create the frame
        frame = Frame(attitude_CP, omega_CP_C, omegaDot_CP_C, r_CP_P, rDot_CP_P, rDDot_CP_P, parentFrame)
        self.frameList.append(frame)

        return frame

    def createPart(self, attitude_CP=None, omega_CP_C=None, omegaDot_CP_C=None,
                   r_CP_P=None, rDot_CP_P=None, rDDot_CP_P=None, parentFrame=None,
                   r_CsC_P=None, m=0, I=None):
        # Default parameter handling
        if attitude_CP is None:
            attitude_CP = MRP([0, 0, 0])
        if omega_CP_C is None:
            omega_CP_C = [0, 0, 0]
        if omegaDot_CP_C is None:
            omegaDot_CP_C = [0, 0, 0]
        if r_CP_P is None:
            r_CP_P = [0, 0, 0]
        if rDot_CP_P is None:
            rDot_CP_P = [0, 0, 0]
        if rDDot_CP_P is None:
            rDDot_CP_P = [0, 0, 0]
        if parentFrame is None:
            parentFrame = self.createFrame()
        if I is None:
            I = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

        # Create the frame and part
        frame = self.createFrame(attitude_CP, omega_CP_C, omegaDot_CP_C, r_CP_P, rDot_CP_P, rDDot_CP_P, parentFrame)
        part = Part(r_CsC_P, frame, m, I)
        self.partList.append(part)

        return part

    def createJoint1D(self, spinAxis, equilibriumFrame=None, thetaInit=0, thetaDotInit=0, k=0, c=0):
        if equilibriumFrame is None:
            equilibriumFrame = self.createFrame()

        # Create the joint
        joint = Joint1D(self, equilibriumFrame, spinAxis, thetaInit, thetaDotInit, k, c)
        self.jointList.append(joint)

        return joint

    def connect2Upper(self, part, joint):
        part.frame.addParentFrame(joint.upperFrame)

    def connect2Lower(self, part, joint):
        joint.lowerFrame.addParentFrame(part.frame)

    def relativeKinematics(self, frame1, frame2):
        # Find the absolute path for the frames of the two parts
        path1 = []
        path2 = []
        findPathAlternate(frame1, path1)
        findPathAlternate(frame2, path2)

        # Find the path to the least common ancestor for both part frames
        pathToLCA1, pathToLCA2 = findLCA(path1, path2)

        # Loop through each path and calculate the relative kinematics

        # Return the result
        return Frame(MRP([0, 0, 0]), [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], None)

    def vectorTransportTheorem(self):
        return

    def inertiaTransportTheorem(self):
        return

    def attitudeKinematics(self):
        return

    def updateKinematics(self):
        return
