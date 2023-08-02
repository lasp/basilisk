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

    def createFrame(self, attitude_SP=MRP([0, 0, 0]), omega_SP_S=None, omegaDot_SP_S=None,
                    r_SP_P=None, rDot_SP_P=None, rDDot_SP_P=None, parentFrame=None):
        # Default parameter handling
        if r_SP_P is None:
            r_SP_P = [0, 0, 0]
        if rDot_SP_P is None:
            rDot_SP_P = [0, 0, 0]
        if rDDot_SP_P is None:
            rDDot_SP_P = [0, 0, 0]
        if omega_SP_S is None:
            omega_SP_S = [0, 0, 0]
        if omegaDot_SP_S is None:
            omegaDot_SP_S = [0, 0, 0]
        if parentFrame is None:
            parentFrame = self.rootFrame

        # Create the frame
        frame = Frame(attitude_SP, omega_SP_S, omegaDot_SP_S, r_SP_P, rDot_SP_P, rDDot_SP_P, parentFrame)
        self.frameList.append(frame)

        return frame

    def createPart(self, attitude_SP=None, omega_SP_S=None, omegaDot_SP_S=None,
                   r_SP_P=None, rDot_SP_P=None, rDDot_SP_P=None, parentFrame=None,
                   r_ScP_P=None, m=0, I=None):
        # Default parameter handling
        if attitude_SP is None:
            attitude_SP = MRP([0, 0, 0])
        if omega_SP_S is None:
            omega_SP_S = [0, 0, 0]
        if omegaDot_SP_S is None:
            omegaDot_SP_S = [0, 0, 0]
        if r_SP_P is None:
            r_SP_P = [0, 0, 0]
        if rDot_SP_P is None:
            rDot_SP_P = [0, 0, 0]
        if rDDot_SP_P is None:
            rDDot_SP_P = [0, 0, 0]
        if parentFrame is None:
            parentFrame = self.createFrame()
        if I is None:
            I = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

        # Create the frame and part
        frame = self.createFrame(attitude_SP, omega_SP_S, omegaDot_SP_S, r_SP_P, rDot_SP_P, rDDot_SP_P, parentFrame)
        part = Part(r_ScP_P, frame, m, I)
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
