import os
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from Basilisk.simulation import KinematicsEngine
from Basilisk.simulation import Frame
from Basilisk.simulation import Vector
from Basilisk.simulation import Tensor
from Basilisk.simulation import AttitudeParameterization
from Basilisk.simulation import Part
from Basilisk.simulation import Joint
from Basilisk.simulation import Hinge

def run():
    myKinematicsEngine = KinematicsEngine.KinematicsEngine()

    myInertialFrame = myKinematicsEngine.createFrame()
    myInertialFrame.tag = "inertial"
    myBodyFrame = myKinematicsEngine.createFrame(myInertialFrame)
    myBodyFrame.tag = "body"

    myPart1 = myKinematicsEngine.createPart(myBodyFrame)
    myPart1.frame.tag = "part1"
    myPart2 = myKinematicsEngine.createPart(myPart1.frame)
    myPart2.frame.tag = "part2"

    myJoint = myKinematicsEngine.createRotaryOneDOFJoint()
    myJoint.lowerFrame.tag = "lower"
    myJoint.upperFrame.tag = "upper"

    myKinematicsEngine.connect(myPart1, myJoint, myPart2)

    # ####
    #
    # myAssembly = Assembly.createAssembly()
    # myPart1 = myAssembly.createPart(myAssembly.rootFrame)
    # myJoint = myAssembly.createJoint(myPart1.frame)
    # myPart2 = myAssembly.createPart(myJoint.frame)
    #
    # myAssembly2 = Assembly.createAssembly()
    # myPart3 = myAssembly2.createPart()
    #
    # myAssembly3 = Assembly.createAssembly()
    # myAssembly3.addAssembly(myAssembly, myAssembly2)
    # myKinematicsEngine.placeAssembly(myAssembly3)
    #
    # ####
    #
    # myPart1 = myKinematicsEngine.createPart(myAssembly.rootFrame)
    # myJoint = myKinematicsEngine.createJoint(myPart1.frame)
    # myPart2 = myKinematicsEngine.createPart(myJoint.frame)
    #
    # myAssembly = myKinematicsEngine.createAssembly()
    # myAssembly.addComponent(myPart1)
    # myAssembly.addComponent(myPart2)
    # myAssembly.addComponent(myJoint)
    #
    # ####
    #
    # myAssembly = myKinematicsEngine.createAssembly()
    # myPart1 = myAssembly.createPart()
    # myJoint = myAssembly.createJoint(myPart1.frame)
    # myPart2 = myAssembly.createPart(myJoint.frame)

    breakpoint()


if __name__ == "__main__":
    run()
