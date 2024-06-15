
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
#   Module Name:        twoAxisGimbalController
#   Author:             Leah Kiner
#   Created:            June 14, 2024
#

import inspect
import os

import math
import numpy as np
import pandas as pd
import pytest
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import twoAxisGimbalController
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


@pytest.mark.parametrize("gimbalTipAngleRef", [0.0 * macros.D2R, 5.0 * macros.D2R, 10.0 * macros.D2R, -5.0 * macros.D2R])
@pytest.mark.parametrize("gimbalTiltAngleRef", [0.0 * macros.D2R, 5.0 * macros.D2R, 10.0 * macros.D2R, -5.0 * macros.D2R])
@pytest.mark.parametrize("accuracy", [1e-4])
def test_twoAxisGimbalController(show_plots, gimbalTipAngleRef, gimbalTiltAngleRef, accuracy):
    r"""
    **Validation Test Description**

    This unit test ensures that the two-axis gimbal flight software module correctly determines the motor 1 and motor 2
    angles corresponding to any given gimbal tip and tilt angles. Using the provided gimbal-to-motor angle lookup
    tables, the module determines the corresponding motor angles.

    **Test Parameters**

    Args:
        show_plots (bool):                          Variable for choosing whether plots should be displayed
        gimbalTipAngleRef (float):                  Gimbal tip reference angle
        gimbalTiltAngleRef (float):                 Gimbal tilt reference angle
        accuracy:                                   Accuracy used in unit test verification

    **Description of Variables Being Tested**

    This test checks that the motor angles determined and output by the module correctly correspond to the reference
    gimbal tip and tilt angles.
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

    # Create the two-axis gimbal reference message
    twoAxisGimbalMessageData = messaging.TwoAxisGimbalMsgPayload()
    twoAxisGimbalMessageData.theta1 = gimbalTipAngleRef
    twoAxisGimbalMessageData.theta2 = gimbalTiltAngleRef
    twoAxisGimbalMessage = messaging.TwoAxisGimbalMsg().write(twoAxisGimbalMessageData)

    # Create the two-axis gimbal (Module tested in this script)
    gimbalToMotor1AnglePath = path + "/../gimbal_to_motor1_angle.csv"
    gimbalToMotor2AnglePath = path + "/../gimbal_to_motor2_angle.csv"
    gimbalController = twoAxisGimbalController.TwoAxisGimbalController(gimbalToMotor1AnglePath, gimbalToMotor2AnglePath)
    gimbalController.ModelTag = "twoAxisGimbalController"
    gimbalController.twoAxisGimbalInMsg.subscribeTo(twoAxisGimbalMessage)
    unitTestSim.AddModelToTask(unitTaskName, gimbalController)

    # Set up data logging for the gimbal module unit test
    motor1StateData = gimbalController.motor1AngleOutMsg.recorder()
    motor2StateData = gimbalController.motor2AngleOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, motor1StateData)
    unitTestSim.AddModelToTask(unitTaskName, motor2StateData)

    # Run the simulation
    simTime = 5.0  # [s]
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))
    unitTestSim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    motor1AngleSim = macros.R2D * motor1StateData.theta  # [deg]
    motor2AngleSim = macros.R2D * motor2StateData.theta  # [deg]

    #
    # Unit Test Verification
    #

    # Read in the lookup tables
    gimbal_to_motor_1_angle = pd.read_csv(gimbalToMotor1AnglePath)
    gimbal_to_motor_2_angle = pd.read_csv(gimbalToMotor2AnglePath)
    gimbal_to_motor_1_angle_table = gimbal_to_motor_1_angle.to_numpy()
    gimbal_to_motor_2_angle_table = gimbal_to_motor_2_angle.to_numpy()

    motor1AngleTruth, motor2AngleTruth = gimbalToMotorAngles(macros.R2D * gimbalTipAngleRef,
                                                             macros.R2D * gimbalTiltAngleRef,
                                                             gimbal_to_motor_1_angle_table,
                                                             gimbal_to_motor_2_angle_table)

    # Print unit test checks
    print("DESIRED GIMBAL TIP ANGLE: ")
    print(macros.R2D * gimbalTipAngleRef)
    print("DESIRED GIMBAL TILT ANGLE: ")
    print(macros.R2D * gimbalTipAngleRef)

    print("TRUE MOTOR 1 ANGLE: ")
    print(motor1AngleTruth)
    print("MODULE-DETERMINED MOTOR 1 ANGLE: ")
    print(motor1AngleSim[-1])

    print("TRUE MOTOR 2 ANGLE: ")
    print(motor2AngleTruth)
    print("MODULE-DETERMINED MOTOR 2 ANGLE: ")
    print(motor2AngleSim[-1])

    # Check that the module-obtained motor angles match the determined truth values
    np.testing.assert_allclose(motor1AngleTruth, motor1AngleSim[-1], atol=accuracy, verbose=True)
    np.testing.assert_allclose(motor2AngleTruth, motor2AngleSim[-1], atol=accuracy, verbose=True)

# This function is used to convert the given gimbal tip and tilt angles to motor angles
def gimbalToMotorAngles(gimbalTipAngle, gimbalTiltAngle, gimbal_to_motor_1_angle_table, gimbal_to_motor_2_angle_table):
    tableStepAngle = 0.5  # [deg]

    if gimbalTipAngle % tableStepAngle == 0 and gimbalTiltAngle % tableStepAngle == 0:  # Do not need to interpolate
        gimbalAngle1 = pullMotorAngle(gimbalTipAngle, gimbalTiltAngle, gimbal_to_motor_1_angle_table)
        gimbalAngle2 = pullMotorAngle(gimbalTipAngle, gimbalTiltAngle, gimbal_to_motor_2_angle_table)

    elif gimbalTipAngle % tableStepAngle == 0 or gimbalTiltAngle % tableStepAngle == 0:  # Linear interpolation required
        if gimbalTipAngle % tableStepAngle == 0:
            # Find the upper and lower interpolation table angle bounds for motor 2
            lowerMotor2Angle = tableStepAngle * math.floor(gimbalTiltAngle / tableStepAngle)
            upperMotor2Angle = tableStepAngle * math.ceil(gimbalTiltAngle / tableStepAngle)

            # Linearly interpolate the gimbal tip angle
            y1_tip = pullMotorAngle(gimbalTipAngle, lowerMotor2Angle, gimbal_to_motor_1_angle_table)
            y2_tip = pullMotorAngle(gimbalTipAngle, upperMotor2Angle, gimbal_to_motor_1_angle_table)
            gimbalAngle1 = linearInterpolation(lowerMotor2Angle, upperMotor2Angle, y1_tip, y2_tip, gimbalTiltAngle)

            # Linearly interpolate the gimbal tilt angle
            y1_tilt = pullMotorAngle(gimbalTipAngle, lowerMotor2Angle, gimbal_to_motor_2_angle_table)
            y2_tilt = pullMotorAngle(gimbalTipAngle, upperMotor2Angle, gimbal_to_motor_2_angle_table)
            gimbalAngle2 = linearInterpolation(lowerMotor2Angle, upperMotor2Angle, y1_tilt, y2_tilt, gimbalTiltAngle)

        else:
            # Find the upper and lower interpolation table angle bounds for motor 1
            lowerMotor1Angle = tableStepAngle * math.floor(gimbalTipAngle / tableStepAngle)
            upperMotor1Angle = tableStepAngle * math.ceil(gimbalTipAngle / tableStepAngle)

            # Linearly interpolate the gimbal tip angle
            y1_tip = pullMotorAngle(lowerMotor1Angle, gimbalTiltAngle, gimbal_to_motor_1_angle_table)
            y2_tip = pullMotorAngle(upperMotor1Angle, gimbalTiltAngle, gimbal_to_motor_1_angle_table)
            gimbalAngle1 = linearInterpolation(lowerMotor1Angle, upperMotor1Angle, y1_tip, y2_tip, gimbalTipAngle)

            # Linearly interpolate the gimbal tilt angle
            y1_tilt = pullMotorAngle(lowerMotor1Angle, gimbalTiltAngle, gimbal_to_motor_2_angle_table)
            y2_tilt = pullMotorAngle(upperMotor1Angle, gimbalTiltAngle, gimbal_to_motor_2_angle_table)
            gimbalAngle2 = linearInterpolation(lowerMotor1Angle, upperMotor1Angle, y1_tilt, y2_tilt, gimbalTipAngle)

    else:  # Bilinear interpolation required
        # Find the upper and lower interpolation table angle bounds for motor1 and motor 2
        lowerMotor1Angle = tableStepAngle * math.floor(gimbalTipAngle / tableStepAngle)
        upperMotor1Angle = tableStepAngle * math.ceil(gimbalTipAngle / tableStepAngle)
        lowerMotor2Angle = tableStepAngle * math.floor(gimbalTiltAngle / tableStepAngle)
        upperMotor2Angle = tableStepAngle * math.ceil(gimbalTiltAngle / tableStepAngle)

        # Bilinearly interpolate the gimbal tip angle
        z11_tip = pullMotorAngle(lowerMotor1Angle, lowerMotor2Angle, gimbal_to_motor_1_angle_table)
        z12_tip = pullMotorAngle(lowerMotor1Angle, upperMotor2Angle, gimbal_to_motor_1_angle_table)
        z21_tip = pullMotorAngle(upperMotor1Angle, lowerMotor2Angle, gimbal_to_motor_1_angle_table)
        z22_tip = pullMotorAngle(upperMotor1Angle, upperMotor2Angle, gimbal_to_motor_1_angle_table)
        gimbalAngle1 = bilinearInterpolation(
            lowerMotor1Angle,
            upperMotor1Angle,
            lowerMotor2Angle,
            upperMotor2Angle,
            z11_tip,
            z12_tip,
            z21_tip,
            z22_tip,
            gimbalTipAngle,
            gimbalTiltAngle,
        )

        # Bilinearly interpolate the gimbal tilt angle
        z11_tilt = pullMotorAngle(lowerMotor1Angle, lowerMotor2Angle, gimbal_to_motor_2_angle_table)
        z12_tilt = pullMotorAngle(lowerMotor1Angle, upperMotor2Angle, gimbal_to_motor_2_angle_table)
        z21_tilt = pullMotorAngle(upperMotor1Angle, lowerMotor2Angle, gimbal_to_motor_2_angle_table)
        z22_tilt = pullMotorAngle(upperMotor1Angle, upperMotor2Angle, gimbal_to_motor_2_angle_table)
        gimbalAngle2 = bilinearInterpolation(
            lowerMotor1Angle,
            upperMotor1Angle,
            lowerMotor2Angle,
            upperMotor2Angle,
            z11_tilt,
            z12_tilt,
            z21_tilt,
            z22_tilt,
            gimbalTipAngle,
            gimbalTiltAngle,
        )

    return gimbalAngle1, gimbalAngle2


# This function uses the given interpolation table and gimbal angles to pull the correct motor angle
def pullMotorAngle(gimbalTipAngle, gimbalTiltAngle, lookup_table_data):
    tableMotorStepAngle = 0.5  # [deg]
    gimbalTipAngleIdx = 40 + int(gimbalTipAngle / tableMotorStepAngle)
    gimbalTiltAngleIdx = 56 + int(gimbalTiltAngle / tableMotorStepAngle)

    return lookup_table_data[gimbalTiltAngleIdx][gimbalTipAngleIdx]


#  This function uses linear interpolation to solve for the value of an unknown function of a single variables f(x)
#  at the point x.
#  return: double
#  param x1: Data point x1
#  param x2: Data point x2
#  param y1: Function value at point x1
#  param y2: Function value at point x2
#  param x: Function x coordinate for interpolation
def linearInterpolation(x1, x2, y1, y2, x):
    return y1 * (x2 - x) / (x2 - x1) + y2 * (x - x1) / (x2 - x1)


#  This function uses bilinear interpolation to solve for the value of an unknown function of two variables f(x,y)
#  at the point (x,y).
#  return: double
#  param x1: Data point x1
#  param x2: Data point x2
#  param y1: Data point y1
#  param y2: Data point y2
#  param z11: Function value at point (x1, y1)
#  param z12: Function value at point (x1, y2)
#  param z21: Function value at point (x2, y1)
#  param z22: Function value at point (x2, y2)
#  param x: Function x coordinate for interpolation
#  param y: Function y coordinate for interpolation
def bilinearInterpolation(x1, x2, y1, y2, z11, z12, z21, z22, x, y):
    return (
            1
            / ((x2 - x1) * (y2 - y1))
            * (
                    z11 * (x2 - x) * (y2 - y)
                    + z21 * (x - x1) * (y2 - y)
                    + z12 * (x2 - x) * (y - y1)
                    + z22 * (x - x1) * (y - y1)
            )
    )


if __name__ == "__main__":
    test_twoAxisGimbalController(
        True,  # show_plots
        0.5 * macros.D2R,  # [rad] gimbalTipAngleRef
        0.5 * macros.D2R,  # [rad] gimbalTiltAngleRef
        1e-4,  # accuracy
    )
