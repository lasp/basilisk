#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        stepperMotorController
#   Author:             Shamsa SabekZaei and Leah Kiner
#   Creation Date:      Aug 25, 2023
#

import inspect
import matplotlib.pyplot as plt
import numpy as np
import os
import pytest
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import stepperMotorController
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

@pytest.mark.parametrize("stepAngle", [0.008 * (np.pi / 180), 0.01 * (np.pi / 180), 0.5 * (np.pi / 180)])
@pytest.mark.parametrize("stepTime", [0.008, 0.1, 0.5])
@pytest.mark.parametrize("initialMotor1Angle", [-5 * (np.pi / 180), 0.0, 60.0 * (np.pi / 180)])
@pytest.mark.parametrize("initialMotor2Angle", [-5 * (np.pi / 180), 0.0, 60.0 * (np.pi / 180)])
@pytest.mark.parametrize("desiredMotor1Angle", [0.0, 10.6 * (np.pi / 180), 60.0051 * (np.pi / 180)])
@pytest.mark.parametrize("desiredMotor2Angle", [0.0, 10.6 * (np.pi / 180), 60.0051 * (np.pi / 180)])

def test_stepperMotorControllerTestFunction(show_plots, stepAngle, stepTime, initialMotor1Angle, initialMotor2Angle, desiredMotor1Angle,desiredMotor2Angle):
    r"""
    **Validation Test Description**

    This unit test ensures that the stepper motor controller module correctly determines the number of steps required to actuate
    from an initial angle to a final reference angle. The initial and desired motor angles are varied so that both
    positive and negative steps are taken. It should be noted that the motor angles are descretized by a constant
    ``stepAngle``; therefore the motor cannot actuate to any desired angle. The desired motor angles are chosen in this
    test so that several cases require the desired angle to be adjusted to the nearest multiple of the motor step angle.
    In other words, this test introduces cases where the computed number of required steps is not an integer. For these
    cases, the determined number of steps must be rounded to the nearest whole step.

    **Test Parameters**

    Args:
        stepAngle (float): [rad] Angle the stepper motor moves through for a single step (constant)
        stepTime (float): [sec] Time required for a single motor step (constant)
        initialMotorAngle (float): [rad] Initial stepper motor angle
        desiredMotorAngle (float): [rad] Desired stepper motor angle

    **Description of Variables Being Tested**

    The module-computed number of required stepper motor steps is checked to match the true number of motor steps
    computed in this script. The first element of the module ``motorStepCommand`` output message is checked to match
    the number of steps determined in this script.

    """
    [testResults, testMessage] = stepperMotorControllerTestFunction(show_plots, stepAngle, stepTime,initialMotor1Angle, initialMotor2Angle, desiredMotor1Angle,desiredMotor2Angle)

    assert testResults < 1, testMessage

def stepperMotorControllerTestFunction(show_plots, stepAngle, stepTime, initialMotor1Angle,initialMotor2Angle, desiredMotor1Angle,desiredMotor2Angle):
    testFailCount = 0                                        # Zero the unit test result counter
    testMessages = []                                        # Create an empty array to store the test log messages
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Print initial and desired angles for debugging
    print(f"Initial Motor 1 Angle: {initialMotor1Angle}")
    print(f"Initial Motor 2 Angle: {initialMotor2Angle}")
    print(f"Desired Motor 1 Angle: {desiredMotor1Angle}")
    print(f"Desired Motor 2 Angle: {desiredMotor2Angle}")

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create the test thread
    testProcessRate = macros.sec2nano(stepTime)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the stepperMotorController module to be tested
    StepperMotor1Controller = stepperMotorController.stepperMotorController()
    StepperMotor1Controller.ModelTag = "stepperMotorController"
    StepperMotor1Controller.stepAngle = stepAngle
    StepperMotor1Controller.stepTime = stepTime
    StepperMotor1Controller.initAngle = initialMotor1Angle
    StepperMotor1Controller.currentAngle = initialMotor1Angle

    StepperMotor2Controller = stepperMotorController.stepperMotorController()
    StepperMotor2Controller.ModelTag = "stepperMotorController"
    StepperMotor2Controller.stepAngle = stepAngle
    StepperMotor2Controller.stepTime = stepTime
    StepperMotor2Controller.initAngle = initialMotor2Angle
    StepperMotor2Controller.currentAngle = initialMotor2Angle

    # Add the stepperMotor test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, StepperMotor1Controller)
    unitTestSim.AddModelToTask(unitTaskName, StepperMotor2Controller)

    # Create the stepperMotorController standaone input message
    HingedRigidBodyMessageDataMotor1 = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageDataMotor1.theta = desiredMotor1Angle
    HingedRigidBodyMessageMotor1 = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageDataMotor1)
    StepperMotor1Controller.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessageMotor1)

    HingedRigidBodyMessageDataMotor2 = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageDataMotor2.theta = desiredMotor2Angle
    HingedRigidBodyMessageMotor2 = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageDataMotor2)
    StepperMotor2Controller.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessageMotor2)

    # Log the test module output message for data comparison
    motor1StepCommandLog = StepperMotor1Controller.motorStepCommandOutMsg.recorder(testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, motor1StepCommandLog)
    motor2StepCommandLog = StepperMotor2Controller.motorStepCommandOutMsg.recorder(testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, motor2StepCommandLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Calculate required number of steps for validation
    if (initialMotor1Angle > 0):
        trueNumSteps1 = (desiredMotor1Angle - (np.ceil(initialMotor1Angle/stepAngle)*stepAngle)) / stepAngle
    else:
        trueNumSteps1 = (desiredMotor1Angle - (np.floor(initialMotor1Angle/stepAngle)*stepAngle)) / stepAngle

    if (initialMotor2Angle > 0):
        trueNumSteps2 = (desiredMotor2Angle - (np.ceil(initialMotor2Angle/stepAngle)*stepAngle)) / stepAngle
    else:
        trueNumSteps2 = (desiredMotor2Angle - (np.floor(initialMotor2Angle/stepAngle)*stepAngle)) / stepAngle

    # If the desired motor angle is not a multiple of the step angle, the number of steps calculated is not an integer
    # and it must be rounded to the nearest whole step
    lowerStepFraction = trueNumSteps1 - np.floor(trueNumSteps1)
    upperStepFraction = np.ceil(trueNumSteps1) - trueNumSteps1
    if (upperStepFraction > lowerStepFraction):
        trueNumSteps1 = np.floor(trueNumSteps1)
    else:
        trueNumSteps1 = np.ceil(trueNumSteps1)

    lowerStepFraction = trueNumSteps2 - np.floor(trueNumSteps2)
    upperStepFraction = np.ceil(trueNumSteps2) - trueNumSteps2
    if (upperStepFraction > lowerStepFraction):
        trueNumSteps2 = np.floor(trueNumSteps2)
    else:
        trueNumSteps2 = np.ceil(trueNumSteps2)

    # Print the true number of steps for validation
    print(f"True Number of Steps 1 for {StepperMotor1Controller.ModelTag}: {trueNumSteps1}")
    print(f"True Number of Steps 2 for {StepperMotor2Controller.ModelTag}: {trueNumSteps2}")

    # If the desired motor angle is not a multiple of the step angle, a new desired angle is calculated
    newMotor1DesiredAngle = initialMotor1Angle + (trueNumSteps1 * stepAngle)
    newMotor2DesiredAngle = initialMotor2Angle + (trueNumSteps2 * stepAngle)

    # Set the simulation time
    actuateTime = stepTime * np.abs(trueNumSteps1+trueNumSteps2/3)  # [sec] Time for the motor to actuate to the desired angle
    holdTime = 5  # [sec] Time the simulation will continue while holding the final angle
    unitTestSim.ConfigureStopTime(macros.sec2nano(actuateTime + holdTime))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()

    # Pull the logged motor step data
    stepsCommanded1 = motor1StepCommandLog.stepsCommanded
    stepsCommanded2 = motor2StepCommandLog.stepsCommanded

    # Print the steps commanded
    print(f"Steps Commanded for {StepperMotor1Controller.ModelTag}: {stepsCommanded1[0]}")
    print(f"Steps Commanded for {StepperMotor2Controller.ModelTag}: {stepsCommanded2[0]}")

    # Check that the correct number of steps was calculated
    if (stepsCommanded1[0] != trueNumSteps1):
        testFailCount += 1
        testMessages.append("\nFAILED: " + StepperMotor1Controller.ModelTag + " Number of required motor steps do not match")
    if (stepsCommanded2[0] != trueNumSteps2):
        testFailCount += 1
        testMessages.append("\nFAILED: " + StepperMotor2Controller.ModelTag + " Number of required motor steps do not match")
    return [testFailCount, ''.join(testMessages)]


# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
if __name__ == "__main__":
    stepperMotorControllerTestFunction(
         False,
         1.0 ,     # stepAngle
         1.0,      #stepTime
         10.0,     # initialAngle1
         8.0,      # initialAngle2
         5.0 ,     # desiredmotorAngle1
         2.0 ,     # desiredmotorAngle2

    )
    