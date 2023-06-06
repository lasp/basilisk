import os
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

import numpy as np
from Basilisk.simulation import KinematicsEngine
from Basilisk.simulation import Frame
from Basilisk.simulation import Vector
from Basilisk.simulation import Tensor
from Basilisk.simulation import AttitudeParameterization
from Basilisk.simulation import Assembly
from Basilisk.simulation import Part
from Basilisk.simulation import Point
from Basilisk.simulation import Joint
from Basilisk.simulation import Hinge
from Basilisk.utilities import RigidBodyKinematics as rbk

def runLeah():
    myKinematicsEngine = KinematicsEngine.KinematicsEngine()

    # Create frames
    frameN = myKinematicsEngine.createFrame()
    frameB = myKinematicsEngine.createFrame(frameN)
    frameM = myKinematicsEngine.createFrame(frameB)

    # Create parts
    part1Inertia = [[10, 0, 0], [0, 10, 0], [0, 0, 10]]
    part2Inertia = [[10, 0, 0], [0, 8, 0], [0, 0, 5]]
    part1 = myKinematicsEngine.createPart(frameM)
    part2 = myKinematicsEngine.createPart(frameB)

    # Create part data
    frameF = part1.frame
    frameP = part2.frame
    part1.IPntSc_S.matrix = part1Inertia
    part2.IPntSc_S.matrix = part2Inertia
    part1.IPntSc_S.writtenFrame = frameF
    part2.IPntSc_S.writtenFrame = frameP

    # Create assembly
    myAssembly = myKinematicsEngine.createAssembly()
    myAssembly.addPart(part1)
    myAssembly.addPart(part2)

    # Create position vector data
    r_PcP_P = [0, 2, 1]  # [m]
    r_FcF_F = [1, 2, 0]  # [m]
    r_BN_N = [1.0, 2.0, 3.0]  # [m]
    r_PB_B = [2.0, 3.0, 4.0]  # [m]
    r_MB_B = [2.0, 1.0, 3.0]  # [m]
    r_FM_M = [4.0, 3.0, 1.0]  # [m]

    # Create frame attitude data
    theta_BN = 30 * np.pi / 180  # [rad]
    theta_PB = 10 * np.pi / 180  # [rad]
    theta_MB = -5 * np.pi / 180  # [rad]
    theta_FM = -10 * np.pi / 180  # [rad]

    dcm_BN = np.array([[1, 0, 0],
                       [0, np.cos(theta_BN),  np.sin(theta_BN)],
                       [0, -np.sin(theta_BN),  np.cos(theta_BN)]])
    dcm_PB = np.array([[np.cos(theta_PB),  np.sin(theta_PB),  0],
                       [-np.sin(theta_PB),  np.cos(theta_PB), 0],
                       [0,  0,  1]])
    dcm_MB = np.array([[np.cos(theta_MB),  np.sin(theta_MB),  0],
                       [-np.sin(theta_MB),  np.cos(theta_MB), 0],
                       [0,  0,  1]])
    dcm_FM = np.array([[np.cos(theta_FM),  0, -np.sin(theta_FM)],
                       [0, 1, 0],
                       [np.sin(theta_FM),  0, np.cos(theta_FM)]])

    sigma_BN = rbk.C2MRP(dcm_BN)
    sigma_PB = rbk.C2MRP(dcm_PB)
    sigma_MB = rbk.C2MRP(dcm_MB)
    sigma_FM = rbk.C2MRP(dcm_FM)

    # Create angular velocity data
    omega_PB_P = [0.5, 1, 0.5]  # [rad/s]
    omega_FM_F = [-1.0, -2.0, 3.0]  # [rad/s]
    omega_MB_M = [-2.0, -3.0, -4.0]  # [rad/s]
    omega_BN_B = [0.0, 1, 0.0]  # [rad/s]

    # Create frame data
    frameB.tag = "frameB"
    frameM.tag = "frameM"
    frameF.tag = "frameF"
    frameP.tag = "frameP"
    frameN.tag = "frameN"
    frameB.r_SP.matrix = r_BN_N
    frameM.r_SP.matrix = r_MB_B
    frameF.r_SP.matrix = r_FM_M
    frameP.r_SP.matrix = r_PB_B
    frameB.sigma_SP = sigma_BN
    frameP.sigma_SP = sigma_PB
    frameM.sigma_SP = sigma_MB
    frameF.sigma_SP = sigma_FM
    frameB.omega_SP.matrix = omega_BN_B
    frameP.omega_SP.matrix = omega_PB_P
    frameM.omega_SP.matrix = omega_MB_M
    frameF.omega_SP.matrix = omega_FM_F
    frameB.originPoint.tag = "pointB"
    frameM.originPoint.tag = "pointM"
    frameF.originPoint.tag = "pointF"

    # Create part data
    m1 = 10  # [kg]
    m2 = 20  # [kg]
    part1.mass = m1
    part2.mass = m2
    part1.r_ScS.matrix = r_FcF_F
    part2.r_ScS.matrix = r_PcP_P
    # part1.r_ScS.writtenFrame = frameF
    # part2.r_ScS.writtenFrame = frameP
    part1.r_ScS.headPoint.tag = "pointFc"
    part2.r_ScS.headPoint.tag = "pointPc"

    # Call kinematic engine functions
    sigma_FPKin = myKinematicsEngine.findRelativeAttitude(frameF, frameP)
    r_FB_BKin = myKinematicsEngine.addPositionVectors(frameF.r_SP, frameM.r_SP)
    omega_FB_MKin = myKinematicsEngine.addAngularVelocityVectors(frameF.omega_SP, frameM.omega_SP)
    r_PcF_MKin = myKinematicsEngine.callFindRelativePosition(part2.r_ScS.headPoint, frameF.originPoint, frameM)
    omega_PF_MKin = myKinematicsEngine.callFindRelativeAngularVelocity(frameP, frameF, frameM)
    assemblyMassKin = myKinematicsEngine.getAssemblyMass(myAssembly)
    r_AssemCMPntB_MKin = myKinematicsEngine.getAssemblyCOM(myAssembly, frameB.originPoint, frameM)
    IPart1PntB_MKin = myKinematicsEngine.parallelAxisTheorem(part1, frameB.originPoint, frameM)
    IAssemPntB_MKin = myKinematicsEngine.getAssemblyInertia(myAssembly, frameB.originPoint, frameM)

    # Calculate the truth data
    sigma_FP = [0.0, 0.0, 0.0]
    r_FB_B = [0.0, 0.0, 0.0]
    omega_FB_M = [0.0, 0.0, 0.0]
    r_PcF_M = [0.0, 0.0, 0.0]
    IPart1PntB_M = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

    # 1. Calculate sigma_FP
    dcm_FP = np.matmul(np.matmul(dcm_FM, dcm_MB), np.transpose(dcm_PB))
    sigma_FP = rbk.C2MRP(dcm_FP)

    # 2. Calculate r_FB_B
    r_FM_B = np.matmul(np.transpose(dcm_MB), r_FM_M)
    r_FB_B = np.add(r_FM_B, r_MB_B)

    # 3. Calculate omega_FB_M
    omega_FM_M = np.matmul(np.transpose(dcm_FM), omega_FM_F)
    omega_FB_M = np.add(omega_FM_M, omega_MB_M)

    # 4. Calculate r_PcF_M
    r_PcP_M = np.matmul(np.matmul(dcm_MB, np.transpose(dcm_PB)), r_PcP_P)
    r_PB_M = np.matmul(dcm_MB, r_PB_B)
    r_PcB_M = np.add(r_PcP_M, r_PB_M)
    r_MB_M = np.matmul(dcm_MB, r_MB_B)
    r_PcM_M = np.subtract(r_PcB_M, r_MB_M)
    r_PcF_M = np.subtract(r_PcM_M, r_FM_M)

    # 5. Calculate omega_PF_M
    dcm_PM = np.matmul(dcm_PB, dcm_MB.transpose())
    omega_PF_M = np.subtract(np.subtract(np.matmul(dcm_PM.transpose(), omega_PB_P), omega_MB_M), np.matmul(dcm_FM.transpose(), omega_FM_F))

    # 6. Calculate IPart1PntB_M
    rFcB_M = np.add(np.add(np.matmul(np.transpose(dcm_FM), r_FcF_F), r_FM_M), np.matmul(dcm_MB, r_MB_B))
    rTildeFcB_M = [[0, -rFcB_M[2], rFcB_M[1]],
                   [rFcB_M[2], 0, -rFcB_M[0]],
                   [-rFcB_M[1], rFcB_M[0], 0]]
    IPart1PntB_M = np.matmul(np.matmul(np.transpose(dcm_FM), part1Inertia), dcm_FM) + m1 * np.matmul(rTildeFcB_M, np.transpose(rTildeFcB_M))

    # 7. Calculate assembly mass
    assemblyMass = m1 + m2

    # 8. Calculate assembly CoM
    r_FcB_M = np.add(np.add(np.matmul(np.transpose(dcm_FM), r_FcF_F), r_FM_M), r_MB_M)
    r_AssemCMPntB_M = np.add(m2 * r_PcB_M, m1 * r_FcB_M) / assemblyMass

    # 9. Calculate assembly inertia IAssemPntB_M
    rTildePcB_M = [[0, -r_PcB_M[2], r_PcB_M[1]],
                   [r_PcB_M[2], 0, -r_PcB_M[0]],
                   [-r_PcB_M[1], r_PcB_M[0], 0]]
    IPart2PntB_M = np.matmul(np.matmul(np.transpose(dcm_PM), part2Inertia), dcm_PM) + m2 * np.matmul(rTildePcB_M, np.transpose(rTildePcB_M))
    IAssemPntB_M = np.add(IPart1PntB_M, IPart2PntB_M)

    print("\n** ** ** ** ** TESTING 'ZEROTH' ORDER RELATIVE KINEMATICS ** ** ** ** **")

    print("\n\n1. RELATIVE ATTITUDE:")
    print("\nTRUTH: sigma_FP: ")
    print(sigma_FP)
    print("\nKINEMATICS ENGINE: sigma_FP: ")
    print(sigma_FPKin)

    print("\n\n2. ADD POSITION VECTORS:")
    print("\nTRUTH: r_FB_B: ")
    print(r_FB_B)
    print("\nKINEMATICS ENGINE: r_FB_B: ")
    print(r_FB_BKin.matrix)

    print("\n\n3. ADD ANGULAR VELOCITY VECTORS:")
    print("\nTRUTH: omega_FB_M: ")
    print(omega_FB_M)
    print("\nKINEMATICS ENGINE: omega_FB_M: ")
    print(omega_FB_MKin.matrix)

    print("\n\n4. RELATIVE POSITION:")
    print("\nTRUTH: r_PcF_M: ")
    print(r_PcF_M)
    print("\nKINEMATICS ENGINE: r_PcF_M: ")
    print(r_PcF_MKin.matrix)

    print("\n\n5. RELATIVE ANGULAR VELOCITY:")
    print("\nTRUTH: omega_PF_M: ")
    print(omega_PF_M)
    print("\nKINEMATICS ENGINE: omega_PF_M: ")
    print(omega_PF_MKin.matrix)

    print("\n\n6. PARALLEL AXIS THEOREM:")
    print("\nTRUTH: Inertia: ")
    print(IPart1PntB_M)
    print("\nKINEMATICS ENGINE: Inertia: ")
    print(IPart1PntB_MKin.matrix)

    print("\n\n7. ASSEMBLY MASS:")
    print("\nTRUTH: Assembly Mass: ")
    print(assemblyMass)
    print("\nKINEMATICS ENGINE: Assembly Mass: ")
    print(assemblyMassKin)

    print("\n\n8. ASSEMBLY COM:")
    print("\nTRUTH: r_AssemCMPntB_M: ")
    print(r_AssemCMPntB_M)
    print("\nKINEMATICS ENGINE: r_AssemCMPntB_M: ")
    print(r_AssemCMPntB_MKin.matrix)

    print("\n\n9. ASSEMBLY INERTIA:")
    print("\nTRUTH: IAssemPntB_M: ")
    print(IAssemPntB_M)
    print("\nKINEMATICS ENGINE: IAssemPntB_M: ")
    print(IAssemPntB_MKin.matrix)

def runJoao():
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
    # myJoint.hingeVector[0].spinAxis = [1, 0, 0]
    myJoint.lowerFrame.tag = "lower"
    myJoint.upperFrame.tag = "upper"

    myKinematicsEngine.connect(myPart1, myJoint, myPart2)

if __name__ == "__main__":
    runLeah()
    #runJoao()