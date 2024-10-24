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

import inspect
import os

import numpy as np
import pytest
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import stepperMotorController
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

@pytest.mark.parametrize("motorStepAngle", [1.0 * macros.D2R])
@pytest.mark.parametrize("motorStepTime", [1.0])
@pytest.mark.parametrize("motorThetaInit", [0.0])
@pytest.mark.parametrize("motorThetaRef1", [-10.0 * macros.D2R, 10.0 * macros.D2R])
@pytest.mark.parametrize("motorThetaRef2", [0.0, 5.0 * macros.D2R, 10.0 * macros.D2R])
@pytest.mark.parametrize("interruptFraction", [0.0, 0.25, 0.5, 0.75])
def test_stepperMotorController(show_plots, motorStepAngle, motorStepTime, motorThetaInit, motorThetaRef1, motorThetaRef2, interruptFraction):
    r"""
    **Validation Test Description**

    This unit test ensures that the stepper motor controller module correctly determines the number of steps required to actuate
    from an initial angle to a final reference angle. An interrupted message is introduced during the motor actuation
    that specifies a new final reference angle. The initial and desired motor angles are varied so that combinations of
    both positive and negative steps are taken. The time of step interruption is varied to ensure that once a step
    begins, it is completed regardless of when the interrupted message is written. Because the other unit test script
    for this module checked the module functionality for various motor step angles and desired angles that are not
    multiples of the motor step angle, the step angle and desired angles chosen in this script are set to simple values.

    **Test Parameters**

    Args:
        motorStepAngle (float): [rad] Angle the stepper motor moves through for a single step (constant)
        motorStepTime (float): [sec] Time required for a single motor step (constant)
        motorThetaInit (float): [rad] Initial stepper motor angle
        motorThetaRef1 (float): [rad] Desired stepper motor angle 1
        motorThetaRef2 (float): [rad] Desired stepper motor angle 2
        interruptFraction (float): Specifies what fraction of a step is completed when the interrupted message is written

    **Description of Variables Being Tested**

    The module-computed number of required stepper motor steps for both simulation chunks are checked with the true
    number of motor steps computed in this script. Because the other unit test script for this module checked that
    messages are correctly written, this validation is not repeated in this script.

    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create the test thread
    testProcessRate = macros.sec2nano(0.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the stepperMotor module to be tested
    motorController = stepperMotorController.StepperMotorController()
    motorController.ModelTag = "stepperMotorController"
    motorController.setStepAngle(motorStepAngle)
    motorController.setStepTime(motorStepTime)
    motorController.setThetaInit(motorThetaInit)
    unitTestSim.AddModelToTask(unitTaskName, motorController)

    # Create the stepperMotorController input message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = motorThetaRef1
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData)
    motorController.motorRefAngleInMsg.subscribeTo(HingedRigidBodyMessage)

    # Log the test module output message for data comparison
    motorStepCommandLog = motorController.motorStepCommandOutMsg.recorder(testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, motorStepCommandLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Calculate required number of steps for validation
    trueNumSteps1 = (motorThetaRef1 - (np.ceil(motorThetaInit/motorStepAngle)*motorStepAngle)) / motorStepAngle

    # If the desired motor angle is not a multiple of the step angle, the number of steps calculated is not an integer
    # and it must be rounded to the nearest whole step
    lowerStepFraction = trueNumSteps1 - np.floor(trueNumSteps1)
    upperStepFraction = np.ceil(trueNumSteps1) - trueNumSteps1
    if (upperStepFraction > lowerStepFraction):
        trueNumSteps1 = np.floor(trueNumSteps1)
    else:
        trueNumSteps1 = np.ceil(trueNumSteps1)

    # If the desired motor angle is not a multiple of the step angle, a new desired angle is calculated
    newMotorDesiredAngle = motorThetaInit + (trueNumSteps1 * motorStepAngle)

    # Set the simulation time
    actuateTime1 = motorStepTime * np.abs(trueNumSteps1)  # [sec] Time for the motor to actuate to the desired angle
    simTime1 = (actuateTime1 / 2) + (interruptFraction * motorStepTime)  # [sec] Time before the first message is interrupted
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime1))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()

    # Create the second interruption message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = motorThetaRef2
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData, unitTestSim.TotalSim.getCurrentNanos())
    motorController.motorRefAngleInMsg.subscribeTo(HingedRigidBodyMessage)

    # Calculate number of steps to actuate from interrupted motor position to the second desired motor angle
    if (trueNumSteps1 > 0):
        interruptedMotorAngle = motorThetaInit + ((simTime1 / motorStepTime) * motorStepAngle)
        # Ensure the interrupted motor angle is set to the next multiple of the motor step angle
        # (If the motor is interrupted during a step)
        interruptedMotorAngle = np.ceil(interruptedMotorAngle / motorStepAngle) * motorStepAngle
    else:
        interruptedMotorAngle = motorThetaInit - ((simTime1 / motorStepTime) * motorStepAngle)
        # Ensure the interrupted motor angle is set to the next multiple of the motor step angle
        # (If the motor is interrupted during a step)
        interruptedMotorAngle = np.floor(interruptedMotorAngle / motorStepAngle) * motorStepAngle

    trueNumSteps2 = (motorThetaRef2 - interruptedMotorAngle) / motorStepAngle

    # If the desired motor angle is not a multiple of the step angle, the number of steps calculated is not an integer
    # and it must be rounded to the nearest whole step
    lowerStepFraction = trueNumSteps2 - np.floor(trueNumSteps2)
    upperStepFraction = np.ceil(trueNumSteps2) - trueNumSteps2
    if (upperStepFraction > lowerStepFraction):
        trueNumSteps2 = np.floor(trueNumSteps2)
    else:
        trueNumSteps2 = np.ceil(trueNumSteps2)

    # If the desired motor angle is not a multiple of the step angle, a new desired angle is calculated
    newMotorDesiredAngle2 = interruptedMotorAngle + (trueNumSteps2 * motorStepAngle)

    # Set the simulation time for chunk 2
    actuateTime2 = motorStepTime * np.abs(trueNumSteps2)  # [sec] Time for the motor to actuate to the desired angle
    holdTime = 5  # [sec] Time the simulation will continue while holding the final angle
    simTime2 = actuateTime2 + holdTime
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime1 + simTime2))

    # Execute simulation chunk 2
    unitTestSim.ExecuteSimulation()

    # Pull the logged motor step data
    stepsCommanded = motorStepCommandLog.stepsCommanded

    # Check that the correct number of steps was calculated
    accuracy = 1e-12
    np.testing.assert_allclose(stepsCommanded[0],
                               trueNumSteps1,
                               atol=accuracy,
                               verbose=True)

    np.testing.assert_allclose(stepsCommanded[-1],
                               trueNumSteps2,
                               atol=accuracy,
                               verbose=True)

    # Manual check that module outputs match the expected true result
    print("True Steps: ")
    print(trueNumSteps1)
    print(trueNumSteps2)
    print("Module Calculation: ")
    print(stepsCommanded[0])
    print(stepsCommanded[-1])


if __name__ == "__main__":
    test_stepperMotorController(
                 False,
                 1.0 * macros.D2R,  # motorStepAngle
                 1.0,  # motorStepTime
                 0.0,  # motorThetaInit
                 10.0 * macros.D2R,  # motorThetaRef1,
                 5.0 * macros.D2R,  # motorThetaRef2
                 0.0   # interruptFraction
               )
