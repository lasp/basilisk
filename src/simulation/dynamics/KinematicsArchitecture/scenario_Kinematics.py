import os
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from Basilisk.simulation import KinematicsEngine, Frame, Vector, AttitudeParameterization


def run():
    myKinematicsEngine = KinematicsEngine.KinematicsEngine()
    myInertialFrame = myKinematicsEngine.createFrame()
    myBodyFrame = myKinematicsEngine.createFrame(myInertialFrame)

    myKinematicsEngine.rootFrame.tag = "root"
    myInertialFrame.tag = "inertial"
    myBodyFrame.tag = "body"

    breakpoint()


if __name__ == "__main__":
    run()
