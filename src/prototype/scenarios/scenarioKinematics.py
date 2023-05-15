import numpy as np

from prototype.KinematicsArchitecture.KinematicsEngine import KinematicsEngine
from prototype.KinematicsArchitecture.AttitudeParameterization import Quaternion


def run():
    # Create the kinematics engine
    myKinematicsEngine = KinematicsEngine()

    # Define the inertial frame
    inertialFrame = myKinematicsEngine.createFrame()

    # Define parts
    beta = Quaternion([0, 0, 1, 0])
    myPart1 = myKinematicsEngine.createPart(attitude_SP=beta, parentFrame=inertialFrame)
    myPart2 = myKinematicsEngine.createPart()
    myPart3 = myKinematicsEngine.createPart(parentFrame=inertialFrame)

    # Define a 1D joint
    spinAxis = np.array([1, 0, 0])
    myJoint = myKinematicsEngine.createJoint1D(spinAxis)

    # Connect the parts with the joint
    myKinematicsEngine.connect2Lower(myPart1, myJoint)
    myKinematicsEngine.connect2Upper(myPart2, myJoint)

    relativeFrame = myKinematicsEngine.relativeKinematics(myPart2.frame, myPart3.frame)

    breakpoint()


if __name__ == "__main__":
    run()
