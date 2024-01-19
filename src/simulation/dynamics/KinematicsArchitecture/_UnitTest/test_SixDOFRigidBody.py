# ISC License
#
# Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.


#
#   Unit Test Script
#   Module Name:        spinningBodies
#   Author:             João Vaz Carneiro
#   Creation Date:      July 14, 2023
#

import os

import numpy as np
import math
import pytest
import matplotlib.pyplot as plt
import random
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from Basilisk.simulation.KinematicsArchitecture import KinematicsEngine
from Basilisk.simulation.KinematicsArchitecture import Frame
from Basilisk.simulation.KinematicsArchitecture import Vector
from Basilisk.simulation.KinematicsArchitecture import ForceVector
from Basilisk.simulation.KinematicsArchitecture import TorqueVector
from Basilisk.simulation.KinematicsArchitecture import Tensor
from Basilisk.simulation.KinematicsArchitecture import Part
from Basilisk.simulation.KinematicsArchitecture import ExtForce
from Basilisk.simulation.KinematicsArchitecture import ExtTorque

from Basilisk.simulation import SixDOFRigidBody
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import RigidBodyKinematics as rbk


@pytest.mark.parametrize("function", ["energyAngularMomentumConservation"
                                      , "translation"
                                      , "rotationTorque"
                                      , "rotationTorqueFree"])
def test_SixDOFRigidBody(show_plots, function):
    eval(function + '(show_plots)')


def energyAngularMomentumConservation(show_plots):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    simulationTimeStep = macros.sec2nano(0.00001)
    dynProcess = scSim.CreateNewProcess(unitProcessName)
    dynProcess.addTask(scSim.CreateNewTask(unitTaskName, simulationTimeStep))

    #
    # Kinematics setup
    #
    myKinematicsEngine = KinematicsEngine()
    myInertialFrame = myKinematicsEngine.createFrame()
    myInertialFrame.tag = "inertial"

    myPart = myKinematicsEngine.createPart(myInertialFrame)
    myPart.frame.tag = "part"
    myPart.mass = 5.0
    inertia = [[24, 1, 6], [1, 15, 3], [6, 3, 10]]
    myPart.IPntSc_S.set(inertia, myPart.frame)
    myPart.r_ScS.setPosition([1.0, -0.5, 1.5], myPart.frame)
    myPart.r_ScS.setVelocity([0.0, 0.0, 0.0], myPart.frame, myPart.frame)
    myPart.frame.r_SP.setPosition([10.0, 50.0, -30.0], myInertialFrame)
    myPart.frame.r_SP.setVelocity([-1, 1, 0.5], myInertialFrame, myInertialFrame)
    myPart.frame.sigma_SP.setAttitude([0.1, 0.2, -0.3])
    myPart.frame.sigma_SP.setAngularVelocity([0.05, 0.03, -0.02], myPart.frame)

    #
    # Dynamics setup
    #
    myDynamicsEngine = SixDOFRigidBody.SixDOFRigidBody(myKinematicsEngine, myInertialFrame, myPart)
    myDynamicsEngine.ModelTag = "myDynamicsEngine"
    scSim.AddModelToTask(unitTaskName, myDynamicsEngine)

    #
    #   Logging
    #
    samplingTime = simulationTimeStep
    stateLog = myPart.bodyStateOutMsg.recorder(samplingTime)
    testingLog = myDynamicsEngine.logger(["transAngMomPntN_N", "rotEnergy", "rotAngMomPntC_N"], samplingTime)
    scSim.AddModelToTask(unitTaskName, stateLog)
    scSim.AddModelToTask(unitTaskName, testingLog)

    #
    # Simulation
    #
    scSim.InitializeSimulation()
    simulationTime = macros.sec2nano(0.01)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    # Plotting
    #
    timeAxis = testingLog.times() * macros.NANO2SEC
    transAngularMomentum_N = testingLog.transAngMomPntN_N
    initialTransAngMom_N = [transAngularMomentum_N[0, 0], transAngularMomentum_N[0, 1], transAngularMomentum_N[0, 2]]
    rotAngularMomentum_N = testingLog.rotAngMomPntC_N
    initialRotAngMom_N = [rotAngularMomentum_N[0, 0], rotAngularMomentum_N[0, 1], rotAngularMomentum_N[0, 2]]
    energy = testingLog.rotEnergy
    initialRotEnergy = energy[0]

    plt.figure(1)
    for idx in range(3):
        plt.plot(timeAxis,
                 (transAngularMomentum_N[:, idx] - initialTransAngMom_N[idx]) / initialTransAngMom_N[idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$H_' + str(idx) + '$')
    plt.legend(loc='best')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Translational Angular Momentum')

    plt.figure(2)
    for idx in range(3):
        plt.plot(timeAxis,
                 (rotAngularMomentum_N[:, idx] - initialRotAngMom_N[idx]) / initialRotAngMom_N[idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$H_' + str(idx) + '$')
    plt.legend(loc='best')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Rotational Angular Momentum')

    plt.figure(3)
    plt.plot(timeAxis, (energy - initialRotEnergy) / initialRotEnergy)
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Energy')

    if show_plots:
        plt.show()
    plt.close("all")

    #
    # Verification
    #

    # TODO: change to relative accuracy
    accuracy = 1e-13
    np.testing.assert_allclose(initialRotEnergy, energy, rtol=accuracy)
    for i in range(3):
        np.testing.assert_allclose(initialRotAngMom_N[i], rotAngularMomentum_N[:, i], rtol=accuracy)
        np.testing.assert_allclose(initialTransAngMom_N[i], transAngularMomentum_N[:, i], rtol=accuracy)


def translation(show_plots):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    simulationTimeStep = macros.sec2nano(0.01)
    dynProcess = scSim.CreateNewProcess(unitProcessName)
    dynProcess.addTask(scSim.CreateNewTask(unitTaskName, simulationTimeStep))

    #
    # Kinematics setup
    #
    myKinematicsEngine = KinematicsEngine()
    myInertialFrame = myKinematicsEngine.createFrame()
    myInertialFrame.tag = "inertial"

    myPart = myKinematicsEngine.createPart(myInertialFrame)
    myPart.frame.tag = "part"
    myPart.mass = 5.0
    inertia = [[24, 1, 6], [1, 15, 3], [6, 3, 10]]
    myPart.IPntSc_S.set(inertia, myPart.frame)
    myPart.r_ScS.setPosition([1.0, -0.5, 1.5], myPart.frame)
    myPart.r_ScS.setVelocity([0.0, 0.0, 0.0], myPart.frame, myPart.frame)
    myPart.frame.r_SP.setPosition([10.0, 50.0, -30.0], myInertialFrame)
    myPart.frame.r_SP.setVelocity([1.0, -3.0, 2.0], myInertialFrame, myInertialFrame)
    myPart.frame.sigma_SP.setAttitude([0.1, 0.2, -0.3])
    myPart.frame.sigma_SP.setAngularVelocity([0.05, 0.03, -0.02], myPart.frame)

    #
    # Dynamics setup
    #
    myDynamicsEngine = SixDOFRigidBody.SixDOFRigidBody(myKinematicsEngine, myInertialFrame, myPart)
    myDynamicsEngine.ModelTag = "myDynamicsEngine"
    scSim.AddModelToTask(unitTaskName, myDynamicsEngine)

    myExtForce = myDynamicsEngine.createExtForce(ForceVector([0, 0, 0], myInertialFrame, myPart.CoMPoint), myPart)

    #
    #   Logging
    #
    samplingTime = simulationTimeStep
    stateLog = myPart.bodyStateOutMsg.recorder(samplingTime)
    testingLog = myDynamicsEngine.logger(["rotEnergy", "rotAngMomPntC_N"], samplingTime)
    scSim.AddModelToTask(unitTaskName, stateLog)
    scSim.AddModelToTask(unitTaskName, testingLog)

    #
    # Simulation
    #
    scSim.InitializeSimulation()
    simulationTime = macros.sec2nano(10.)
    numSegments = 4
    forceMatrix = [[random.uniform(-1, 1) for _ in range(3)] for _ in range(numSegments)]
    for segment in range(numSegments):
        force = ForceVector(forceMatrix[segment], myInertialFrame, myPart.CoMPoint)
        myExtForce.setForce(force)
        scSim.ConfigureStopTime(simulationTime * (segment + 1))
        scSim.ExecuteSimulation()

    #
    # Plotting
    #
    timeAxis = stateLog.times() * macros.NANO2SEC
    dataPos = stateLog.r_CN_N
    dataVel = stateLog.v_CN_N

    rotAngularMomentum_N = testingLog.rotAngMomPntC_N
    initialRotAngMom_N = [rotAngularMomentum_N[0, 0], rotAngularMomentum_N[0, 1], rotAngularMomentum_N[0, 2]]
    energy = testingLog.rotEnergy
    initialRotEnergy = energy[0]

    plt.close("all")
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeAxis, dataPos[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$r_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Position')

    plt.figure(2)
    for idx in range(3):
        plt.plot(timeAxis, dataVel[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$rDot_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Velocity')

    plt.figure(3)
    for idx in range(3):
        plt.plot(timeAxis,
                 (rotAngularMomentum_N[:, idx] - initialRotAngMom_N[idx]) / initialRotAngMom_N[idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$H_' + str(idx) + '$')
    plt.legend(loc='best')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Angular Momentum')

    plt.figure(4)
    plt.plot(timeAxis, (energy - initialRotEnergy) / initialRotEnergy)
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Energy')

    if show_plots:
        plt.show()
    plt.close("all")

    #
    # Verification
    #

    # Truth values
    deltaT = samplingTime * macros.NANO2SEC
    truePos = dataPos[0, None]
    trueVel = dataVel[0, None]
    for idx in range(1, len(timeAxis)):
        segment = math.floor((idx - 1) / (simulationTime / samplingTime))

        newPos = truePos[-1, :] + trueVel[-1, :] * deltaT + np.array(forceMatrix[segment]) / myPart.mass * deltaT ** 2 / 2
        truePos = np.row_stack((truePos, newPos[None, :]))

        newVel = trueVel[-1, :] + np.array(forceMatrix[segment]) / myPart.mass * deltaT
        trueVel = np.row_stack((trueVel, newVel[None, :]))

    # Testing setup
    accuracy = 1e-13
    np.testing.assert_allclose(truePos, dataPos, atol=accuracy)
    np.testing.assert_allclose(trueVel, dataVel, atol=accuracy)
    np.testing.assert_allclose(initialRotEnergy, energy, atol=accuracy)
    for i in range(3):
        np.testing.assert_allclose(initialRotAngMom_N[i], rotAngularMomentum_N[:, i], atol=accuracy)


def rotationTorque(show_plots):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    simulationTimeStep = macros.sec2nano(0.01)
    dynProcess = scSim.CreateNewProcess(unitProcessName)
    dynProcess.addTask(scSim.CreateNewTask(unitTaskName, simulationTimeStep))

    #
    # Kinematics setup
    #
    myKinematicsEngine = KinematicsEngine()
    myInertialFrame = myKinematicsEngine.createFrame()
    myInertialFrame.tag = "inertial"

    myPart = myKinematicsEngine.createPart(myInertialFrame)
    myPart.frame.tag = "part"
    myPart.mass = 5.0
    inertia = [[24, 0, 0], [0, 15, 0], [0, 0, 10]]
    myPart.IPntSc_S.set(inertia, myPart.frame)
    myPart.r_ScS.setPosition([1.0, -0.5, 1.5], myPart.frame)
    myPart.r_ScS.setVelocity([0.0, 0.0, 0.0], myPart.frame, myPart.frame)
    myPart.frame.r_SP.setPosition([10.0, 50.0, -30.0], myInertialFrame)
    myPart.frame.r_SP.setVelocity([1.0, -3.0, 2.0], myInertialFrame, myInertialFrame)
    myPart.frame.sigma_SP.setAttitude([0.1, 0.2, -0.3])

    torqueIdx = random.choice([0, 1, 2])
    angVel = [0.0, 0.0, 0.0]
    angVel[torqueIdx] = 0.05
    myPart.frame.sigma_SP.setAngularVelocity(angVel, myPart.frame)

    #
    # Dynamics setup
    #
    myDynamicsEngine = SixDOFRigidBody.SixDOFRigidBody(myKinematicsEngine, myInertialFrame, myPart)
    myDynamicsEngine.ModelTag = "myDynamicsEngine"
    scSim.AddModelToTask(unitTaskName, myDynamicsEngine)

    myExtTorque = myDynamicsEngine.createExtTorque(TorqueVector([0, 0, 0], myPart.frame), myPart)

    #
    #   Logging
    #
    samplingTime = simulationTimeStep
    stateLog = myPart.bodyStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(unitTaskName, stateLog)

    #
    # Simulation
    #
    scSim.InitializeSimulation()
    simulationTime = macros.sec2nano(10.)
    numSegments = 4
    torqueMatrix = [[0.0, 0.0, 0.0] for _ in range(numSegments)]
    for segment in range(numSegments):
        torqueMatrix[segment][torqueIdx] = random.uniform(-1, 1)
        torque = TorqueVector(torqueMatrix[segment], myPart.frame)
        myExtTorque.setTorque(torque)
        scSim.ConfigureStopTime(simulationTime * (segment + 1))
        scSim.ExecuteSimulation()

    #
    # Plotting
    #
    timeAxis = stateLog.times() * macros.NANO2SEC
    dataSigmaBN = stateLog.sigma_BN
    dataOmegaBN = stateLog.omega_BN_B

    plt.close("all")
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeAxis, dataSigmaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Attitude $\sigma_{B/R}$')

    plt.figure(2)
    for idx in range(3):
        plt.plot(timeAxis, dataOmegaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Angular Velocity')

    if show_plots:
        plt.show()
    plt.close("all")

    #
    # Verification
    #

    # Truth values
    deltaT = samplingTime * macros.NANO2SEC
    trueAngVel = dataOmegaBN[0, None]
    for idx in range(1, len(timeAxis)):
        segment = math.floor((idx - 1) / (simulationTime / samplingTime))

        newAngVel = trueAngVel[-1, :] + np.array(torqueMatrix[segment]) / myPart.IPntSc_S.getMatrix(myPart.frame)[torqueIdx][torqueIdx] * deltaT
        trueAngVel = np.row_stack((trueAngVel, newAngVel[None, :]))

    accuracy = 1e-13
    np.testing.assert_allclose(trueAngVel, dataOmegaBN, atol=accuracy)


def rotationTorqueFree(show_plots):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    simulationTimeStep = macros.sec2nano(0.01)
    dynProcess = scSim.CreateNewProcess(unitProcessName)
    dynProcess.addTask(scSim.CreateNewTask(unitTaskName, simulationTimeStep))

    #
    # Kinematics setup
    #
    myKinematicsEngine = KinematicsEngine()
    myInertialFrame = myKinematicsEngine.createFrame()
    myInertialFrame.tag = "inertial"

    myPart = myKinematicsEngine.createPart(myInertialFrame)
    myPart.frame.tag = "part"
    myPart.mass = 5.0
    myPart.IPntSc_S.set([[30, 0, 0], [0, 50, 0], [0, 0, 25]], myPart.frame)
    myPart.r_ScS.setPosition([1.0, -0.5, 1.5], myPart.frame)
    myPart.r_ScS.setVelocity([0.0, 0.0, 0.0], myPart.frame, myPart.frame)
    myPart.frame.r_SP.setPosition([10.0, 50.0, -30.0], myInertialFrame)
    myPart.frame.r_SP.setVelocity([1.0, -3.0, 2.0], myInertialFrame, myInertialFrame)
    myPart.frame.sigma_SP.setAttitude([0.1, 0.2, -0.3])

    # Choose angular velocity such that the angular momentum is along the -nHat3 axis
    h = random.random()
    H = Vector([0.0, 0.0, -h], myInertialFrame)
    omega_BN = myPart.IPntSc_S.inverse() * H
    myPart.frame.sigma_SP.setAngularVelocity(omega_BN)

    #
    # Dynamics setup
    #
    myDynamicsEngine = SixDOFRigidBody.SixDOFRigidBody(myKinematicsEngine, myInertialFrame, myPart)
    myDynamicsEngine.ModelTag = "myDynamicsEngine"
    scSim.AddModelToTask(unitTaskName, myDynamicsEngine)

    #
    #   Logging
    #
    samplingTime = simulationTimeStep
    stateLog = myPart.bodyStateOutMsg.recorder(samplingTime)
    testingLog = myDynamicsEngine.logger(["rotEnergy", "rotAngMomPntC_N"], samplingTime)
    scSim.AddModelToTask(unitTaskName, stateLog)
    scSim.AddModelToTask(unitTaskName, testingLog)

    #
    # Simulation
    #
    scSim.InitializeSimulation()
    simulationTime = macros.sec2nano(10.)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    # Plotting
    #
    timeAxis = stateLog.times() * macros.NANO2SEC
    dataSigmaBN = stateLog.sigma_BN
    dataOmegaBN = stateLog.omega_BN_B

    rotAngularMomentum_N = testingLog.rotAngMomPntC_N
    initialRotAngMom_N = [rotAngularMomentum_N[0, 0], rotAngularMomentum_N[0, 1], rotAngularMomentum_N[0, 2]]
    energy = testingLog.rotEnergy
    initialRotEnergy = energy[0]

    plt.close("all")
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeAxis, dataSigmaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Attitude $\sigma_{B/R}$')

    plt.figure(2)
    for idx in range(3):
        plt.plot(timeAxis, dataOmegaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Angular Velocity')

    plt.figure(3)
    plt.plot(timeAxis,
             (rotAngularMomentum_N[:, 2] - initialRotAngMom_N[2]) / initialRotAngMom_N[2],
             label=r'$H_3$')
    plt.legend(loc='best')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Angular Momentum')

    plt.figure(4)
    plt.plot(timeAxis, (energy - initialRotEnergy) / initialRotEnergy)
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Energy')

    if show_plots:
        plt.show()
    plt.close("all")

    #
    # Verification
    #

    IPntBc_B = myPart.IPntSc_S.getMatrix(myPart.frame)
    I1 = IPntBc_B[0][0]
    I2 = IPntBc_B[1][1]
    I3 = IPntBc_B[2][2]

    # Truth values
    trueAngVel = np.zeros((len(timeAxis), 3))
    for idx in range(len(timeAxis)):
        sigma1 = dataSigmaBN[idx, 0]
        sigma2 = dataSigmaBN[idx, 1]
        sigma3 = dataSigmaBN[idx, 2]
        sigmaNorm = np.linalg.norm(dataSigmaBN[idx, :])

        trueAngVel[idx, :] = np.array([- h / I1 * (8 * sigma1 * sigma3 - 4 * sigma2 * (1 - sigmaNorm ** 2)) / (1 + sigmaNorm ** 2) ** 2,
                                       - h / I2 * (8 * sigma2 * sigma3 + 4 * sigma1 * (1 - sigmaNorm ** 2)) / (1 + sigmaNorm ** 2) ** 2,
                                       - h / I3 * (4 * (- sigma1 ** 2 - sigma2 ** 2 + sigma3 ** 2) + (1 - sigmaNorm ** 2) ** 2) / (1 + sigmaNorm ** 2) ** 2
                                       ])

    accuracy = 1e-13
    np.testing.assert_allclose(trueAngVel, dataOmegaBN, atol=accuracy)
    np.testing.assert_allclose(initialRotEnergy, energy, atol=accuracy)
    for i in range(3):
        np.testing.assert_allclose(initialRotAngMom_N[i], rotAngularMomentum_N[:, i], atol=accuracy)


if __name__ == "__main__":
    energyAngularMomentumConservation(True)
    # translation(True)
    # rotationTorque(True)
    # rotationTorqueFree(True)
