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
import math
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import pytest
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.simulation import stepperMotor
from Basilisk.simulation import twoAxisGimbal
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


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

    # Create the messages for the initial motor angles
    motor1ThetaInit = 0.0 * macros.D2R  # [rad]
    Motor1ThetaInitMessageData = messaging.HingedRigidBodyMsgPayload()
    Motor1ThetaInitMessageData.theta = motor1ThetaInit
    Motor1ThetaInitMessageData.thetaDot = 0.0
    Motor1ThetaInitMessage = messaging.HingedRigidBodyMsg().write(Motor1ThetaInitMessageData)

    motor2ThetaInit = 0.0 * macros.D2R  # [rad]
    Motor2ThetaInitMessageData = messaging.HingedRigidBodyMsgPayload()
    Motor2ThetaInitMessageData.theta = motor2ThetaInit
    Motor2ThetaInitMessageData.thetaDot = 0.0
    Motor2ThetaInitMessage = messaging.HingedRigidBodyMsg().write(Motor2ThetaInitMessageData)

    # Create the stepper motors using two instances of the stepperMotor module
    motorStepAngle = 0.1 * macros.D2R  # [rad]
    motorStepTime = 0.1  # [s]
    stepperMotor1 = stepperMotor.StepperMotor()
    stepperMotor1.ModelTag = "stepperMotor1"
    stepperMotor1.setThetaInit(motor1ThetaInit)
    stepperMotor1.setStepAngle(motorStepAngle)
    stepperMotor1.setStepTime(motorStepTime)
    stepperMotor1.setThetaDDotMax(motorStepAngle / (0.25 * motorStepTime * motorStepTime))
    unitTestSim.AddModelToTask(unitTaskName, stepperMotor1)

    stepperMotor2 = stepperMotor.StepperMotor()
    stepperMotor2.ModelTag = "stepperMotor2"
    stepperMotor2.setThetaInit(motor2ThetaInit)
    stepperMotor2.setStepAngle(motorStepAngle)
    stepperMotor2.setStepTime(motorStepTime)
    stepperMotor2.setThetaDDotMax(motorStepAngle / (0.25 * motorStepTime * motorStepTime))
    unitTestSim.AddModelToTask(unitTaskName, stepperMotor2)

    # Create the StepperMotor input messages
    motor1StepsCommanded = 500
    motor2StepsCommanded = 200
    Motor1StepCommandMessageData = messaging.MotorStepCommandMsgPayload()
    Motor1StepCommandMessageData.stepsCommanded = motor1StepsCommanded
    Motor1StepCommandMessage = messaging.MotorStepCommandMsg().write(Motor1StepCommandMessageData)
    stepperMotor1.motorStepCommandInMsg.subscribeTo(Motor1StepCommandMessage)

    Motor2StepCommandMessageData = messaging.MotorStepCommandMsgPayload()
    Motor2StepCommandMessageData.stepsCommanded = motor2StepsCommanded
    Motor2StepCommandMessage = messaging.MotorStepCommandMsg().write(Motor2StepCommandMessageData)
    stepperMotor2.motorStepCommandInMsg.subscribeTo(Motor2StepCommandMessage)

    # Create the two axis gimbal using the twoAxisGimbal module
    rotHat1_M = np.array([1.0, 0.0, 0.0])
    rotHat2_F = np.array([0.0, 1.0, 0.0])
    gimbal = twoAxisGimbal.TwoAxisGimbal()
    gimbal.ModelTag = "twoAxisGimbal"
    gimbal.setGimbalRotHat1_M(rotHat1_M)
    gimbal.setGimbalRotHat2_F(rotHat2_F)
    gimbal.setMotorStepAngle(motorStepAngle)
    gimbal.setMotorStepTime(motorStepTime)
    gimbal.motor1InitStateInMsg.subscribeTo(Motor1ThetaInitMessage)
    gimbal.motor2InitStateInMsg.subscribeTo(Motor2ThetaInitMessage)
    gimbal.motor1StepCmdInMsg.subscribeTo(Motor1StepCommandMessage)
    gimbal.motor2StepCmdInMsg.subscribeTo(Motor2StepCommandMessage)
    gimbal.motor1StateInMsg.subscribeTo(stepperMotor1.stepperMotorOutMsg)
    gimbal.motor2StateInMsg.subscribeTo(stepperMotor2.stepperMotorOutMsg)
    unitTestSim.AddModelToTask(unitTaskName, gimbal)

    # Log module data for module unit test validation
    gimbalPrescribedRotStateData = gimbal.prescribedRotationOutMsg.recorder()
    gimbalTipTiltAngleData = gimbal.twoAxisGimbalOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, gimbalPrescribedRotStateData)
    unitTestSim.AddModelToTask(unitTaskName, gimbalTipTiltAngleData)

    # Run the simulation
    simTime = motor1StepsCommanded * motorStepTime + motor2StepsCommanded * motorStepTime + 5.0  # [s]
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))
    unitTestSim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    timespan = macros.NANO2SEC * gimbalPrescribedRotStateData.times()  # [s]
    gimbalTipAngle = macros.R2D * gimbalTipTiltAngleData.theta1  # [deg]
    gimbalTiltAngle = macros.R2D * gimbalTipTiltAngleData.theta2  # [deg]
    sigma_FM = gimbalPrescribedRotStateData.sigma_FM
    omega_FM_F = macros.R2D * gimbalPrescribedRotStateData.omega_FM_F  # [deg/s]
    omegaPrime_FM_F = macros.R2D * gimbalPrescribedRotStateData.omegaPrime_FM_F  # [deg/s^2]

    #
    # Unit Test Verification:
    #

    # Calculate the final gimbal tip and tilt angles
    gimbalTipAngleFinal , gimbalTiltAngleFinal = motorToGimbalAngles(motorStepAngle, motor1ThetaInit, motor2ThetaInit, motor1StepsCommanded, motor2StepsCommanded)

    # Print interpolation results
    print("\nGimbal Tip Reference Angle: ")
    print(gimbalTipAngleFinal)
    print("\nGimbal Tilt Reference Angle: ")
    print(gimbalTiltAngleFinal)
    print("********************\n")

    # Check that the final gimbal angles match the calculated final gimbal angles
    accuracy = 0.001
    # np.testing.assert_allclose(gimbalTipAngleFinal,
    #                            gimbalTipAngle[-1],
    #                            atol=accuracy,
    #                            verbose=True)
    #
    # np.testing.assert_allclose(gimbalTiltAngleFinal,
    #                            gimbalTiltAngle[-1],
    #                            atol=accuracy,
    #                            verbose=True)

    if show_plots:
        # 1. Plot the gimbal tip and tilt angles
        plt.figure()
        plt.clf()
        plt.plot(timespan, gimbalTipAngle, label=r"$\psi$")
        plt.plot(timespan, gimbalTiltAngle, label=r"$\phi$")
        plt.title(r'Two-Axis Gimbal Tip $\psi$ and Tilt $\phi$ Angles', fontsize=14)
        plt.ylabel('(deg)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='upper right', prop={'size': 12})
        plt.grid(True)

        # 2. Plot the gimbal prescribed rotational states
        # 2A. Plot PRV angle from sigma_FM
        phi_FM = []
        for i in range(len(timespan)):
            phi_FM.append(macros.R2D * 4 * np.arctan(np.linalg.norm(sigma_FM[i, :])))  # [deg]

        plt.figure()
        plt.clf()
        plt.plot(timespan, phi_FM, label=r"$\Phi$")
        plt.title(r'Profiled PRV Angle $\Phi_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
        plt.ylabel('(deg)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='center right', prop={'size': 14})
        plt.grid(True)

        # 2B. Plot gimbal hub-relative angular velocity omega_FM_F
        plt.figure()
        plt.clf()
        plt.plot(timespan, omega_FM_F[:, 0], label=r'$\omega_{1}$')
        plt.plot(timespan, omega_FM_F[:, 1], label=r'$\omega_{2}$')
        plt.plot(timespan, omega_FM_F[:, 2], label=r'$\omega_{3}$')
        plt.title(r'Profiled Angular Velocity ${}^\mathcal{F} \omega_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
        plt.ylabel('(deg/s)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='upper right', prop={'size': 14})
        plt.grid(True)

        # 2C. Plot gimbal hub-relative angular acceleration omegaPrime_FM_F
        plt.figure()
        plt.clf()
        plt.plot(timespan, omegaPrime_FM_F[:, 0], label=r'1')
        plt.plot(timespan, omegaPrime_FM_F[:, 1], label=r'2')
        plt.plot(timespan, omegaPrime_FM_F[:, 2], label=r'3')
        plt.title(r'Profiled Angular Acceleration ${}^\mathcal{F} \omega$Prime$_{\mathcal{F}/\mathcal{M}}$',
                  fontsize=14)
        plt.ylabel('(deg/s$^2$)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='upper right', prop={'size': 14})
        plt.grid(True)

        plt.show()
    plt.close("all")

def motorToGimbalAngles(motorStepAngle, motor1InitAngle, motor2InitAngle, motor1StepsCommanded, motor2StepsCommanded):
    # Determine reference motor angles
    motor1Angle = motor1InitAngle + motorStepAngle * motor1StepsCommanded  # [deg]
    motor2Angle = motor2InitAngle + motorStepAngle * motor2StepsCommanded  # [deg]

    print("\n********************\nMotor 1 Reference Angle: ")
    print(motor1Angle)
    print("\n********************\nMotor 2 Reference Angle: ")
    print(motor2Angle)

    # Read in the lookup tables
    path_to_file = "/Users/leahkiner/Desktop/motor_to_gimbal_tip_angle.csv"
    motor_to_gimbal_tip_angle = pd.read_csv(path_to_file)
    path_to_file = "/Users/leahkiner/Desktop/motor_to_gimbal_tilt_angle.csv"
    motor_to_gimbal_tilt_angle = pd.read_csv(path_to_file)
    tableStepAngle = 0.5  # [deg]

    if motor1Angle % tableStepAngle == 0 and motor2Angle % tableStepAngle == 0:  # Do not need to interpolate
        gimbalAngle1 = pullGimbalAngle(motor1Angle, motor2Angle, motor_to_gimbal_tip_angle)
        gimbalAngle2 = pullGimbalAngle(motor1Angle, motor2Angle, motor_to_gimbal_tilt_angle)
    elif motor1Angle % tableStepAngle == 0 or motor2Angle % tableStepAngle == 0:  # Linear interpolation required
        if motor1Angle % tableStepAngle == 0:
            lowerMotor2Angle = tableStepAngle * math.floor(motor2Angle / tableStepAngle)
            upperMotor2Angle = tableStepAngle * math.ceil(motor2Angle / tableStepAngle)

            z1_tip = pullGimbalAngle(motor1Angle, lowerMotor2Angle, motor_to_gimbal_tip_angle)
            z2_tip = pullGimbalAngle(motor1Angle, upperMotor2Angle, motor_to_gimbal_tip_angle)
            gimbalAngle1 = linearInterpolation(lowerMotor2Angle, upperMotor2Angle, z1_tip, z2_tip, motor2Angle)

            z1_tilt = pullGimbalAngle(motor1Angle, lowerMotor2Angle, motor_to_gimbal_tilt_angle)
            z2_tilt = pullGimbalAngle(motor1Angle, upperMotor2Angle, motor_to_gimbal_tilt_angle)
            gimbalAngle2 = linearInterpolation(lowerMotor2Angle, upperMotor2Angle, z1_tilt, z2_tilt, motor2Angle)
        else:
            lowerMotor1Angle = tableStepAngle * math.floor(motor1Angle / tableStepAngle)
            upperMotor1Angle = tableStepAngle * math.ceil(motor1Angle / tableStepAngle)

            z1_tip = pullGimbalAngle(lowerMotor1Angle, motor2Angle, motor_to_gimbal_tip_angle)
            z2_tip = pullGimbalAngle(upperMotor1Angle, motor2Angle, motor_to_gimbal_tip_angle)
            gimbalAngle1 = linearInterpolation(lowerMotor1Angle, upperMotor1Angle, z1_tip, z2_tip, motor1Angle)

            z1_tilt = pullGimbalAngle(lowerMotor1Angle, motor2Angle, motor_to_gimbal_tilt_angle)
            z2_tilt = pullGimbalAngle(upperMotor1Angle, motor2Angle, motor_to_gimbal_tilt_angle)
            gimbalAngle2 = linearInterpolation(lowerMotor1Angle, upperMotor1Angle, z1_tilt, z2_tilt, motor1Angle)
    else:  # Bilinear interpolation required
        lowerMotor1Angle = tableStepAngle * math.floor(motor1Angle / tableStepAngle)
        upperMotor1Angle = tableStepAngle * math.ceil(motor1Angle / tableStepAngle)
        lowerMotor2Angle = tableStepAngle * math.floor(motor2Angle / tableStepAngle)
        upperMotor2Angle = tableStepAngle * math.ceil(motor2Angle / tableStepAngle)

        z11_tip = pullGimbalAngle(lowerMotor1Angle, lowerMotor2Angle, motor_to_gimbal_tip_angle)
        z12_tip = pullGimbalAngle(lowerMotor1Angle, upperMotor2Angle, motor_to_gimbal_tip_angle)
        z21_tip = pullGimbalAngle(upperMotor1Angle, lowerMotor2Angle, motor_to_gimbal_tip_angle)
        z22_tip = pullGimbalAngle(upperMotor1Angle, upperMotor2Angle, motor_to_gimbal_tip_angle)

        z11_tilt = pullGimbalAngle(lowerMotor1Angle, lowerMotor2Angle, motor_to_gimbal_tilt_angle)
        z12_tilt = pullGimbalAngle(lowerMotor1Angle, upperMotor2Angle, motor_to_gimbal_tilt_angle)
        z21_tilt = pullGimbalAngle(upperMotor1Angle, lowerMotor2Angle, motor_to_gimbal_tilt_angle)
        z22_tilt = pullGimbalAngle(upperMotor1Angle, upperMotor2Angle, motor_to_gimbal_tilt_angle)

        gimbalAngle1 = bilinearInterpolation(
            lowerMotor1Angle,
            upperMotor1Angle,
            lowerMotor2Angle,
            upperMotor2Angle,
            z11_tip,
            z12_tip,
            z21_tip,
            z22_tip,
            motor1Angle,
            motor2Angle,
        )
        gimbalAngle2 = bilinearInterpolation(
            lowerMotor1Angle,
            upperMotor1Angle,
            lowerMotor2Angle,
            upperMotor2Angle,
            z11_tilt,
            z12_tilt,
            z21_tilt,
            z22_tilt,
            motor1Angle,
            motor2Angle,
        )
    return gimbalAngle1, gimbalAngle2

def pullGimbalAngle(motor1Angle, motor2Angle, lookup_table_data):
    tableMotorStepAngle = 0.5  # [deg]
    motor1Idx = int(motor1Angle / tableMotorStepAngle)
    motor2Idx = int(motor2Angle / tableMotorStepAngle)
    return lookup_table_data.iat[motor2Idx, motor1Idx]

def linearInterpolation(x1, x2, z1, z2, x):
    return z1 * (x2 - x) / (x2 - x1) + z2 * (x - x1) / (x2 - x1)

def bilinearInterpolation(x1, x2, y1, y2, z11, z12, z21, z22, x, y):
    return (1 / ((x2 - x1) * (y2 - y1)) * ( z11 * (x2 - x) * (y2 - y)
                                        + z21 * (x - x1) * (y2 - y)
                                        + z12 * (x2 - x) * (y - y1)
                                        + z22 * (x - x1) * (y - y1)))

if __name__ == "__main__":
    test_twoAxisGimbal(
        True,  # show_plots
    )
