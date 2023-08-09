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
import matplotlib.pyplot as plt
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from Basilisk.simulation.KinematicsArchitecture import KinematicsEngine
from Basilisk.simulation.KinematicsArchitecture import Frame
from Basilisk.simulation.KinematicsArchitecture import Vector
from Basilisk.simulation.KinematicsArchitecture import Tensor
from Basilisk.simulation.KinematicsArchitecture import Part
from Basilisk.simulation.KinematicsArchitecture import Joint
from Basilisk.simulation.KinematicsArchitecture import Hinge

from Basilisk.simulation import SixDOFRigidBody
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport

def test_SixDOFRigidBody(show_plots):
    r"""
    **Validation Test Description**

    **Description of Variables Being Tested**

    - ``finalRotAngMom``
    - ``finalRotEnergy``

    against their initial values.
    """
    [testResults, testMessage] = dynamicsEngine_SixDOFRigidBody(show_plots)
    assert testResults < 1, testMessage


def dynamicsEngine_SixDOFRigidBody(show_plots):
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
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
    inertia = [[34, 1, 6], [1, 15, 3], [6, 3, 10]]
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

    #
    #   Logging
    #
    samplingTime = simulationTimeStep
    stateLog = myPart.bodyStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(unitTaskName, stateLog)
    scSim.AddVariableForLogging(myDynamicsEngine.ModelTag + ".rotEnergy", samplingTime, 0, 0, 'double')
    scSim.AddVariableForLogging(myDynamicsEngine.ModelTag + ".rotAngMomPntC_N", samplingTime, 0, 2, 'double')

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
    dataPos = stateLog.r_CN_N
    dataVel = stateLog.v_CN_N

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
    for idx in range(3):
        plt.plot(timeAxis, dataPos[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$r_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Position')

    plt.figure(4)
    for idx in range(3):
        plt.plot(timeAxis, dataVel[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$rDot_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Velocity')

    #
    # Verification
    #
    energy = scSim.GetLogVariableData(myDynamicsEngine.ModelTag + ".rotEnergy")
    angularMomentum_N = scSim.GetLogVariableData(myDynamicsEngine.ModelTag + ".rotAngMomPntC_N")

    # Setup the conservation quantities
    initialRotAngMom_N = [[angularMomentum_N[0, 1], angularMomentum_N[0, 2], angularMomentum_N[0, 3]]]
    initialRotEnergy = [[energy[0, 1]]]

    plt.figure(5)
    for idx in range(3):
        plt.plot(timeAxis, (angularMomentum_N[:, idx+1] - angularMomentum_N[0, idx+1]) / angularMomentum_N[0, idx+1],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$H_' + str(idx) + '$')
    plt.legend(loc='best')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Angular Momentum')

    plt.figure(6)
    plt.plot(timeAxis, (energy[:, 1] - energy[0, 1]) / energy[0, 1])
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Energy')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12
    angularMomentum_N = np.delete(angularMomentum_N, 0, axis=1)  # remove time column
    energy = np.delete(energy, 0, axis=1)  # remove time column

    for i in range(0, len(initialRotAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(angularMomentum_N[i], initialRotAngMom_N[i], 3, accuracy):
            testFailCount += 1
            testMessages.append(
                "FAILED: sixDOFRigidBody integrated test failed rotational angular momentum unit test")

    for i in range(0, len(initialRotEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(energy[i], initialRotEnergy[i], 1, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: sixDOFRigidBody integrated test failed rotational energy unit test")

    if testFailCount == 0:
        print("PASSED: " + " sixDOFRigidBody gravity integrated test")

    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    dynamicsEngine_SixDOFRigidBody(True)
