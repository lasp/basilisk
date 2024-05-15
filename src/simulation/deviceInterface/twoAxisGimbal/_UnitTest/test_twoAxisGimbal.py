#
#  ISC License
#
#  Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

#
#   Unit Test Script
#   Module Name:        twoAxisGimbal
#   Author:             Leah Kiner
#   Created:            May 25, 2024
#

import inspect
import matplotlib.pyplot as plt
import numpy as np
import os
import pytest
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.simulation import stepperMotor
from Basilisk.simulation import twoAxisGimbal
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

def test_twoAxisGimbal(show_plots):
    r"""
    **Validation Test Description**

    The unit test for this module ensures that

    **Test Parameters**

    Args:
        show_plots (bool): Variable for choosing whether plots should be displayed

    **Description of Variables Being Tested**

    This unit test checks that the
    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    testTimeStepSec = 0.1  # [s]
    testProcessRate = macros.sec2nano(testTimeStepSec)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create initial motor angle message
    motorThetaInit = 0.0  # [deg]
    MotorThetaInitMessageData = messaging.HingedRigidBodyMsgPayload()
    MotorThetaInitMessageData.theta = motorThetaInit
    MotorThetaInitMessageData.thetaDot = 0.0
    MotorThetaInitMessage = messaging.HingedRigidBodyMsg().write(MotorThetaInitMessageData)

    # Create two instances of the stepperMotor module
    motorStepAngle = 0.008  # [deg]
    motorStepTime = 0.008  # [s]
    StepperMotor1 = stepperMotor.StepperMotor()
    StepperMotor1.ModelTag = "StepperMotor1"
    StepperMotor1.setThetaInit(motorThetaInit)
    StepperMotor1.setStepAngle(motorStepAngle)
    StepperMotor1.setStepTime(motorStepTime)
    StepperMotor1.setThetaDDotMax(motorStepAngle / (0.25 * motorStepTime * motorStepTime))
    unitTestSim.AddModelToTask(unitTaskName, StepperMotor1)

    StepperMotor2 = stepperMotor.StepperMotor()
    StepperMotor2.ModelTag = "StepperMotor2"
    StepperMotor2.setThetaInit(motorThetaInit)
    StepperMotor2.setStepAngle(motorStepAngle)
    StepperMotor2.setStepTime(motorStepTime)
    StepperMotor2.setThetaDDotMax(motorStepAngle / (0.25 * motorStepTime * motorStepTime))
    unitTestSim.AddModelToTask(unitTaskName, StepperMotor2)

    # Create the StepperMotor input messages
    motor1StepsCommanded = 10
    motor2StepsCommanded = 20
    Motor1StepCommandMessageData = messaging.MotorStepCommandMsgPayload()
    Motor1StepCommandMessageData.stepsCommanded = motor1StepsCommanded
    Motor1StepCommandMessage = messaging.MotorStepCommandMsg().write(Motor1StepCommandMessageData)
    StepperMotor1.motorStepCommandInMsg.subscribeTo(Motor1StepCommandMessage)

    Motor2StepCommandMessageData = messaging.MotorStepCommandMsgPayload()
    Motor2StepCommandMessageData.stepsCommanded = motor2StepsCommanded
    Motor2StepCommandMessage = messaging.MotorStepCommandMsg().write(Motor2StepCommandMessageData)
    StepperMotor2.motorStepCommandInMsg.subscribeTo(Motor2StepCommandMessage)

    # Create an instance of the twoAxisGimbal module to be tested
    rotHat1_M = np.array([1.0, 0.0, 0.0])
    rotHat2_F = np.array([0.0, 1.0, 0.0])
    gimbal = twoAxisGimbal.TwoAxisGimbal()
    gimbal.ModelTag = "twoAxisGimbal"
    gimbal.setGimbalRotHat1_M(rotHat1_M)
    gimbal.setGimbalRotHat2_F(rotHat2_F)
    gimbal.setMotorStepAngle(motorStepAngle)
    gimbal.setMotorStepTime(motorStepTime)
    gimbal.motor1InitStateInMsg.subscribeTo(MotorThetaInitMessage)
    gimbal.motor2InitStateInMsg.subscribeTo(MotorThetaInitMessage)
    gimbal.motor1StepCmdInMsg.subscribeTo(Motor1StepCommandMessage)
    gimbal.motor2StepCmdInMsg.subscribeTo(Motor2StepCommandMessage)
    gimbal.motor1StateInMsg.subscribeTo(StepperMotor1.stepperMotorOutMsg)
    gimbal.motor2StateInMsg.subscribeTo(StepperMotor2.stepperMotorOutMsg)
    unitTestSim.AddModelToTask(unitTaskName, gimbal)

    # Log module data for module unit test validation
    gimbalPrescribedRotStateData = gimbal.prescribedRotationOutMsg.recorder()
    gimbalTipTiltAngleData = gimbal.twoAxisGimbalOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, gimbalPrescribedRotStateData)
    unitTestSim.AddModelToTask(unitTaskName, gimbalTipTiltAngleData)

    # Run the simulation
    simTime = 5.0  # [s]
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))
    unitTestSim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    timespan = macros.NANO2SEC * gimbalPrescribedRotStateData.times()  # [s]
    gimbalTipAngle = macros.R2D * gimbalTipTiltAngleData.theta1  # [deg]
    gimbalTiltAngle = macros.R2D * gimbalTipTiltAngleData.theta2  # [deg]

    # Unit test validation:

    # Plot gimbal data
    plt.figure()
    plt.clf()
    plt.plot(timespan, gimbalTipAngle, label=r"$\psi$")
    plt.plot(timespan, gimbalTiltAngle, label=r"$\phi$")
    plt.title(r'Two-Axis Gimbal Tip $\psi$ and Tilt $\phi$ Angles', fontsize=14)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")

if __name__ == "__main__":
    test_twoAxisGimbal(
        True,  # show_plots
    )
