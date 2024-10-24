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
#   Module Name:        stepperMotor
#   Author:             Leah Kiner
#   Creation Date:      March 25, 2024#

import inspect
import matplotlib.pyplot as plt
import numpy as np
import os
import pytest
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import stepperMotorController
from Basilisk.simulation import stepperMotor
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

@pytest.mark.parametrize("stepAngle", [1.0 * (np.pi / 180)])
@pytest.mark.parametrize("stepTime", [1.0])
@pytest.mark.parametrize("initialMotorAngle", [0.0])
@pytest.mark.parametrize("desiredMotorAngle1", [10.0 * (np.pi / 180), -10.0 * (np.pi / 180)])
@pytest.mark.parametrize("desiredMotorAngle2", [0.0, 10.0 * (np.pi / 180), 5.0 * (np.pi / 180)])
@pytest.mark.parametrize("interruptFraction", [0.0, 0.25, 0.5, 0.75])
@pytest.mark.parametrize("accuracy", [1e-12])
def test_stepperMotorInterrupt(show_plots, stepAngle, stepTime, initialMotorAngle, desiredMotorAngle1, desiredMotorAngle2, interruptFraction, accuracy):
    r"""
    **Validation Test Description**

    This unit test ensures that the stepper motor profiler module correctly actuates the stepper motor from an initial
    angle to a final reference angle, given an input number of steps commanded. An interrupted message is introduced
    during the motor actuation that specifies a new final reference angle. The initial and desired motor angles are
    varied so that combinations of both positive and negative steps are taken. The time of step interruption is varied
    to ensure that once a step begins, it is completed regardless of when the interrupted message is written. Because
    the other unit test script for this module checked the basic module functionality for a single positive or negative
    step command, the step angle, step time, initial and desired angles chosen in this script are set to simple values.

    **Test Parameters**

    Args:
        stepAngle (float): [rad] Angle the stepper motor moves through for a single step (constant)
        stepTime (float): [sec] Time required for a single motor step (constant)
        initialMotorAngle (float): [rad] Initial stepper motor angle
        desiredMotorAngle1 (float): [rad] Desired stepper motor angle 1
        desiredMotorAngle2 (float): [rad] Desired stepper motor angle 2
        interruptFraction (float): Specifies what fraction of a step is completed when the interrupted message is written
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    This test checks that the final motor angle matches the second desired motor angle.

    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create the test thread
    testProcessRate = macros.sec2nano(0.1)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the stepper motor controller module
    StepperMotorController = stepperMotorController.StepperMotorController()
    StepperMotorController.ModelTag = "stepperMotorController"
    StepperMotorController.setStepAngle(stepAngle)
    StepperMotorController.setStepTime(stepTime)
    StepperMotorController.setThetaInit(initialMotorAngle)

    # Add the stepperMotorController test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, StepperMotorController)

    # Create the stepperMotorController input message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = desiredMotorAngle1
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData)
    StepperMotorController.motorRefAngleInMsg.subscribeTo(HingedRigidBodyMessage)

    # Create an instance of the stepperMotor module to be tested
    StepperMotor = stepperMotor.StepperMotor()
    StepperMotor.ModelTag = "StepperMotor"
    StepperMotor.setThetaInit(initialMotorAngle)
    StepperMotor.setStepAngle(stepAngle)
    StepperMotor.setStepTime(stepTime)
    StepperMotor.setThetaDDotMax(stepAngle / (0.25 * stepTime * stepTime))

    # Add the stepperMotor test module to the runtime call list
    unitTestSim.AddModelToTask(unitTaskName, StepperMotor)

    # Subscribe the step command message to the stepper motor profiler module
    StepperMotor.motorStepCommandInMsg.subscribeTo(StepperMotorController.motorStepCommandOutMsg)

    # Log the test module output message for data comparison
    stepCommandDataLog = StepperMotorController.motorStepCommandOutMsg.recorder(testProcessRate)
    stepperMotorDataLog = StepperMotor.stepperMotorOutMsg.recorder(testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, stepCommandDataLog)
    unitTestSim.AddModelToTask(unitTaskName, stepperMotorDataLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Calculate required number of steps for validation
    if (initialMotorAngle > 0):
        trueNumSteps1 = (desiredMotorAngle1 - (np.ceil(initialMotorAngle/stepAngle)*stepAngle)) / stepAngle
    else:
        trueNumSteps1 = (desiredMotorAngle1 - (np.floor(initialMotorAngle/stepAngle)*stepAngle)) / stepAngle

    # If the desired motor angle is not a multiple of the step angle, the number of steps calculated is not an integer
    # and it must be rounded to the nearest whole step
    lowerStepFraction = trueNumSteps1 - np.floor(trueNumSteps1)
    upperStepFraction = np.ceil(trueNumSteps1) - trueNumSteps1
    if (upperStepFraction > lowerStepFraction):
        trueNumSteps1 = np.floor(trueNumSteps1)
    else:
        trueNumSteps1 = np.ceil(trueNumSteps1)

    # If the desired motor angle is not a multiple of the step angle, a new desired angle is calculated
    newMotorDesiredAngle = initialMotorAngle + (trueNumSteps1 * stepAngle)

    # Set the simulation time
    actuateTime1 = stepTime * np.abs(trueNumSteps1)  # [sec] Time for the motor to actuate to the desired angle
    simTime1 = (actuateTime1 / 2) + (interruptFraction * stepTime)  # [sec] Time before the first message is interrupted
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime1))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()

    # Create the second interruption message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = desiredMotorAngle2
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData, unitTestSim.TotalSim.getCurrentNanos())
    StepperMotorController.motorRefAngleInMsg.subscribeTo(HingedRigidBodyMessage)

    # Calculate number of steps to actuate from interrupted motor position to the second desired motor angle
    if (trueNumSteps1 > 0):
        interruptedMotorAngle = initialMotorAngle + ((simTime1 / stepTime) * stepAngle)
        # Ensure the interrupted motor angle is set to the next multiple of the motor step angle
        # (If the motor is interrupted during a step)
        interruptedMotorAngle = np.ceil(interruptedMotorAngle / stepAngle) * stepAngle
    else:
        interruptedMotorAngle = initialMotorAngle - ((simTime1 / stepTime) * stepAngle)
        # Ensure the interrupted motor angle is set to the next multiple of the motor step angle
        # (If the motor is interrupted during a step)
        interruptedMotorAngle = np.floor(interruptedMotorAngle / stepAngle) * stepAngle

    trueNumSteps2 = (desiredMotorAngle2 - interruptedMotorAngle) / stepAngle

    # If the desired motor angle is not a multiple of the step angle, the number of steps calculated is not an integer
    # and it must be rounded to the nearest whole step
    lowerStepFraction = trueNumSteps2 - np.floor(trueNumSteps2)
    upperStepFraction = np.ceil(trueNumSteps2) - trueNumSteps2
    if (upperStepFraction > lowerStepFraction):
        trueNumSteps2 = np.floor(trueNumSteps2)
    else:
        trueNumSteps2 = np.ceil(trueNumSteps2)

    # If the desired motor angle is not a multiple of the step angle, a new desired angle is calculated
    newMotorDesiredAngle2 = interruptedMotorAngle + (trueNumSteps2 * stepAngle)

    # Set the simulation time for chunk 2
    actuateTime2 = stepTime * np.abs(trueNumSteps2)  # [sec] Time for the motor to actuate to the desired angle
    holdTime = 5  # [sec] Time the simulation will continue while holding the final angle
    simTime2 = actuateTime2 + holdTime
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime1 + simTime2))

    # Execute simulation chunk 2
    unitTestSim.ExecuteSimulation()

    # Pull the logged motor step data
    timespan = stepperMotorDataLog.times()
    theta = macros.R2D * stepperMotorDataLog.theta
    thetaDot = macros.R2D * stepperMotorDataLog.thetaDot
    thetaDDot = macros.R2D * stepperMotorDataLog.thetaDDot
    motorStepCount = stepperMotorDataLog.stepCount
    motorCommandedSteps = stepperMotorDataLog.stepsCommanded

    # Plot motor angle
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, theta, label=r"$\theta$")
    plt.title(r'Stepper Motor Angle $\theta_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot motor thetaDot
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, thetaDot, label=r"$\dot{\theta}$")
    plt.title(r'Stepper Motor Angle Rate $\dot{\theta}_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
    plt.ylabel('(deg/s)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot motor thetaDDot
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, thetaDDot, label=r"$\ddot{\theta}$")
    plt.title(r'Stepper Motor Angular Acceleration $\ddot{\theta}_{\mathcal{F}/\mathcal{M}}$ ', fontsize=14)
    plt.ylabel('(deg/s$^2$)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot steps commanded and motor steps taken
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, motorStepCount)
    plt.plot(timespan * macros.NANO2SEC, motorCommandedSteps, '--', label='Commanded')
    plt.title(r'Motor Step History', fontsize=14)
    plt.ylabel('Steps', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")

    # Check that the final motor angle matches the motor reference angle desiredMotorAngle2
    np.testing.assert_allclose(theta[-1],
                               macros.R2D * desiredMotorAngle2,
                               atol=accuracy,
                               verbose=True)

if __name__ == "__main__":
    test_stepperMotorInterrupt(
                 True,
                 1.0 * (np.pi / 180),     # stepAngle
                 1.0,                     # stepTime
                 0.0,                     # initialMotorAngle
                 10.0 * (np.pi / 180),    # desiredMotorAngle1,
                 5.0 * (np.pi / 180),     # desiredMotorAngle2
                 0.0,                     # interruptFraction
                 1e-12                    # accuracy
               )
