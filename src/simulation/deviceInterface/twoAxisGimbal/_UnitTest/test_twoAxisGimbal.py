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
from matplotlib import collections as mc

@pytest.mark.parametrize("accuracy", [0.01])
@pytest.mark.parametrize("motor1StepsCommanded", [1000, 2000, 5000])
@pytest.mark.parametrize("motor2StepsCommanded", [1000, 5000])
def test_twoAxisGimbal(show_plots, motor1StepsCommanded, motor2StepsCommanded, accuracy):
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

    testTimeStepSec = 0.001  # [s]
    testProcessRate = macros.sec2nano(testTimeStepSec)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the initial angle message for stepper motor 1
    motor1ThetaInit = 0.0 * macros.D2R  # [rad]
    Motor1ThetaInitMessageData = messaging.HingedRigidBodyMsgPayload()
    Motor1ThetaInitMessageData.theta = motor1ThetaInit
    Motor1ThetaInitMessageData.thetaDot = 0.0
    Motor1ThetaInitMessage = messaging.HingedRigidBodyMsg().write(Motor1ThetaInitMessageData)

    # Create the initial angle message for stepper motor 2
    motor2ThetaInit = 0.0 * macros.D2R  # [rad]
    Motor2ThetaInitMessageData = messaging.HingedRigidBodyMsgPayload()
    Motor2ThetaInitMessageData.theta = motor2ThetaInit
    Motor2ThetaInitMessageData.thetaDot = 0.0
    Motor2ThetaInitMessage = messaging.HingedRigidBodyMsg().write(Motor2ThetaInitMessageData)

    # Define stepper motor parameters
    motorStepAngle = 0.008 * macros.D2R  # [rad]
    motorStepTime = 0.008  # [s]
    motorThetaDDotMax = motorStepAngle / (0.25 * motorStepTime * motorStepTime)  # [rad/s^2]

    # Create stepper motor 1
    stepperMotor1 = stepperMotor.StepperMotor()
    stepperMotor1.ModelTag = "stepperMotor1"
    stepperMotor1.setThetaInit(motor1ThetaInit)
    stepperMotor1.setStepAngle(motorStepAngle)
    stepperMotor1.setStepTime(motorStepTime)
    stepperMotor1.setThetaDDotMax(motorThetaDDotMax)
    unitTestSim.AddModelToTask(unitTaskName, stepperMotor1)

    # Create stepper motor 2
    stepperMotor2 = stepperMotor.StepperMotor()
    stepperMotor2.ModelTag = "stepperMotor2"
    stepperMotor2.setThetaInit(motor2ThetaInit)
    stepperMotor2.setStepAngle(motorStepAngle)
    stepperMotor2.setStepTime(motorStepTime)
    stepperMotor2.setThetaDDotMax(motorThetaDDotMax)
    unitTestSim.AddModelToTask(unitTaskName, stepperMotor2)

    # Create the step command message for stepper motor 1
    Motor1StepCommandMessageData = messaging.MotorStepCommandMsgPayload()
    Motor1StepCommandMessageData.stepsCommanded = motor1StepsCommanded
    Motor1StepCommandMessage = messaging.MotorStepCommandMsg().write(Motor1StepCommandMessageData)
    stepperMotor1.motorStepCommandInMsg.subscribeTo(Motor1StepCommandMessage)

    # Create the step command message for stepper motor 2
    Motor2StepCommandMessageData = messaging.MotorStepCommandMsgPayload()
    Motor2StepCommandMessageData.stepsCommanded = motor2StepsCommanded
    Motor2StepCommandMessage = messaging.MotorStepCommandMsg().write(Motor2StepCommandMessageData)
    stepperMotor2.motorStepCommandInMsg.subscribeTo(Motor2StepCommandMessage)

    # Define the two-axis gimbal parameters
    rotHat1_M = np.array([1.0, 0.0, 0.0])
    rotHat2_F = np.array([0.0, 1.0, 0.0])

    # Create the two-axis gimbal (Module tested in this script)
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

    # Set up data logging for the gimbal module unit test
    gimbalPrescribedRotStateData = gimbal.prescribedRotationOutMsg.recorder()
    gimbalTipTiltAngleData = gimbal.twoAxisGimbalOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, gimbalPrescribedRotStateData)
    unitTestSim.AddModelToTask(unitTaskName, gimbalTipTiltAngleData)

    # Determine the simulation time
    if motor1StepsCommanded > motor2StepsCommanded:
        simSegment1Time = motor2StepsCommanded * motorStepTime  # [s]
        simSegment2Time = (motor1StepsCommanded - motor2StepsCommanded) * motorStepTime  # [s]
    elif motor1StepsCommanded < motor2StepsCommanded:
        simSegment1Time = motor1StepsCommanded * motorStepTime  # [s]
        simSegment2Time = (motor2StepsCommanded - motor1StepsCommanded) * motorStepTime  # [s]
    else:
        simSegment1Time = motor1StepsCommanded * motorStepTime  # [s]
        simSegment2Time = 0  # [s]
    simExtraTime = 10.0  # [s]
    simTimeTotal = simSegment1Time + simSegment2Time + simExtraTime  # [s]

    # Run the simulation
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTimeTotal))
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

    # Calculate the initial gimbal tip and tilt angles
    gimbalTipAngleInit , gimbalTiltAngleInit = motorToGimbalAngles(macros.R2D * motor1ThetaInit, macros.R2D * motor2ThetaInit)

    # Print interpolation results
    print("\nInitial Gimbal Tip Angle: ")
    print(gimbalTipAngleInit)
    print("\nInitial Gimbal Tilt Angle: ")
    print(gimbalTiltAngleInit)

    if motor1StepsCommanded > motor2StepsCommanded:
        motor1Segment1FinalAngle = motor1ThetaInit + motor2StepsCommanded * motorStepAngle  # [rad]
        motor2Segment1FinalAngle = motor2ThetaInit + motor2StepsCommanded * motorStepAngle  # [rad]
        gimbalTipAngleSegment1Final, gimbalTiltAngleSegment1Final = motorToGimbalAngles(macros.R2D * motor1Segment1FinalAngle, macros.R2D * motor2Segment1FinalAngle)

        motor1Segment2FinalAngle = motor1ThetaInit + motor1StepsCommanded * motorStepAngle  # [rad]
        motor2Segment2FinalAngle = motor2ThetaInit + motor2StepsCommanded * motorStepAngle  # [rad]
        gimbalTipAngleSegment2Final, gimbalTiltAngleSegment2Final = motorToGimbalAngles(macros.R2D * motor1Segment2FinalAngle, macros.R2D * motor2Segment2FinalAngle)  # [deg]

        gimbalTipAngleCheckList = [gimbalTipAngleSegment1Final, gimbalTipAngleSegment2Final]  # [deg]
        gimbalTiltAngleCheckList = [gimbalTiltAngleSegment1Final, gimbalTiltAngleSegment2Final]  # [deg]

        segment1StopTimeIdx = int(round(simSegment1Time / testTimeStepSec)) + 1
        gimbalTipAngleSimList = [gimbalTipAngle[segment1StopTimeIdx], gimbalTipAngle[-1]]  # [deg]
        gimbalTiltAngleSimList = [gimbalTiltAngle[segment1StopTimeIdx], gimbalTiltAngle[-1]]  # [deg]

    elif motor1StepsCommanded < motor2StepsCommanded:
        motor1Segment1FinalAngle = motor1ThetaInit + motor1StepsCommanded * motorStepAngle  # [rad]
        motor2Segment1FinalAngle = motor2ThetaInit + motor1StepsCommanded * motorStepAngle  # [rad]
        gimbalTipAngleSegment1Final, gimbalTiltAngleSegment1Final = motorToGimbalAngles(macros.R2D * motor1Segment1FinalAngle, macros.R2D * motor2Segment1FinalAngle)  # [deg]

        motor1Segment2FinalAngle = motor1ThetaInit + motor1StepsCommanded * motorStepAngle  # [rad]
        motor2Segment2FinalAngle = motor2ThetaInit + motor2StepsCommanded * motorStepAngle  # [rad]
        gimbalTipAngleSegment2Final, gimbalTiltAngleSegment2Final = motorToGimbalAngles(macros.R2D * motor1Segment2FinalAngle, macros.R2D * motor2Segment2FinalAngle)  # [deg]

        gimbalTipAngleCheckList = [gimbalTipAngleSegment1Final, gimbalTipAngleSegment2Final]  # [deg]
        gimbalTiltAngleCheckList = [gimbalTiltAngleSegment1Final, gimbalTiltAngleSegment2Final]  # [deg]

        segment1StopTimeIdx = int(round(simSegment1Time / testTimeStepSec)) + 1
        gimbalTipAngleSimList = [gimbalTipAngle[segment1StopTimeIdx], gimbalTipAngle[-1]]  # [deg]
        gimbalTiltAngleSimList = [gimbalTiltAngle[segment1StopTimeIdx], gimbalTiltAngle[-1]]  # [deg]

    else:
        motor1FinalAngle = motor1ThetaInit + motor1StepsCommanded * motorStepAngle  # [rad]
        motor2FinalAngle = motor2ThetaInit + motor2StepsCommanded * motorStepAngle  # [rad]
        gimbalTipAngleFinal, gimbalTiltAngleFinal = motorToGimbalAngles(macros.R2D * motor1FinalAngle, macros.R2D * motor2FinalAngle)  # [deg]

        gimbalTipAngleCheckList = [gimbalTipAngleFinal]  # [deg]
        gimbalTiltAngleCheckList = [gimbalTiltAngleFinal]  # [deg]

        gimbalTipAngleSimList = [gimbalTipAngle[-1]]  # [deg]
        gimbalTiltAngleSimList = [gimbalTiltAngle[-1]]  # [deg]


    # Print gimbal angle checks
    print("GIMBAL TIP ANGLE CHECK TRUTH VALUES: ")
    print(gimbalTipAngleCheckList)
    print("GIMBAL TIP ANGLE MODULE VALUES:")
    print(gimbalTipAngleSimList)

    print("\n\nGIMBAL TILT ANGLE CHECK TRUTH VALUES: ")
    print(gimbalTiltAngleCheckList)
    print("GIMBAL TILT ANGLE MODULE VALUES:")
    print(gimbalTiltAngleSimList)

    # # Check that the gimbal angles converge to the desired values for each actuation segment
    # np.testing.assert_allclose(gimbalTipAngleCheckList,
    #                            gimbalTipAngleSimList,
    #                            atol=accuracy,
    #                            verbose=True)
    #
    # np.testing.assert_allclose(gimbalTiltAngleCheckList,
    #                            gimbalTiltAngleSimList,
    #                            atol=accuracy,
    #                            verbose=True)

    if show_plots:
        # 1. Plot the gimbal tip and tilt angles
        # 1A. Plot gimbal tip angle
        gimbalTipAngleInitPlotting = np.ones(len(timespan)) * gimbalTipAngleInit  # [deg]
        gimbalTipAngleFinalPlotting = np.ones(len(timespan)) * gimbalTipAngleCheckList[-1]  # [deg]
        plt.figure()
        plt.clf()
        plt.plot(timespan, gimbalTipAngle, label=r"$\psi$")
        plt.plot(timespan, gimbalTipAngleInitPlotting, '--', label=r"$\psi_0$")
        plt.plot(timespan, gimbalTipAngleFinalPlotting, '--', label=r"$\psi_{\text{ref}}$")
        plt.title(r'Gimbal Tip Angle $\psi$', fontsize=14)
        plt.ylabel('(deg)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='center right', prop={'size': 12})
        plt.grid(True)

        # 1B. Plot gimbal tilt angle
        gimbalTiltAngleInitPlotting = np.ones(len(timespan)) * gimbalTiltAngleInit  # [deg]
        gimbalTiltAngleFinalPlotting = np.ones(len(timespan)) * gimbalTiltAngleCheckList[-1]  # [deg]
        plt.figure()
        plt.clf()
        plt.plot(timespan, gimbalTiltAngle, label=r"$\phi$")
        plt.plot(timespan, gimbalTiltAngleInitPlotting, '--', label=r"$\phi_0$")
        plt.plot(timespan, gimbalTiltAngleFinalPlotting, '--', label=r"$\phi_{\text{ref}}$")
        plt.title(r'Gimbal Tilt Angle $\phi$', fontsize=14)
        plt.ylabel('(deg)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='center right', prop={'size': 12})
        plt.grid(True)

        # Plot 2D Diamond and the corresponding gimbal trajectory
        line1 = [(18.342, 0.0), (0.676, 27.43)]
        line2 = [(-18.764, 0.0), (0.676, 27.43)]
        line3 = [(-18.764, 0.0), (0.676, -27.43)]
        line4 = [(18.342, 0.0), (0.676, -27.43)]
        lines = [line1, line2, line3, line4]
        lc = mc.LineCollection(lines, colors="crimson", linewidths=2, linestyles="dashed")
        plt.figure()
        plt.clf()
        ax = plt.axes()
        ax.add_collection(lc)
        plt.plot(gimbalTipAngle, gimbalTiltAngle, linewidth=1, color='black')
        ax.scatter(gimbalTipAngle[0], gimbalTiltAngle[0], marker='.', linewidth=4, color='springgreen', label='Initial')
        ax.scatter(gimbalTipAngle[-1], gimbalTiltAngle[-1], marker='*', linewidth=4, color='magenta', label='Final')
        plt.title('Gimbal Sequential Trajectory', fontsize=14)
        plt.ylabel(r'$\psi$ (deg)', fontsize=16)
        plt.xlabel(r'$\phi$ (deg)', fontsize=16)
        plt.legend(loc='upper right', prop={'size': 12})
        plt.grid(True)
        plt.axis("equal")

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

        # # 2B. Plot gimbal hub-relative angular velocity omega_FM_F
        # plt.figure()
        # plt.clf()
        # plt.plot(timespan, omega_FM_F[:, 0], label=r'$\omega_{1}$')
        # plt.plot(timespan, omega_FM_F[:, 1], label=r'$\omega_{2}$')
        # plt.plot(timespan, omega_FM_F[:, 2], label=r'$\omega_{3}$')
        # plt.title(r'Profiled Angular Velocity ${}^\mathcal{F} \omega_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
        # plt.ylabel('(deg/s)', fontsize=14)
        # plt.xlabel('Time (s)', fontsize=14)
        # plt.legend(loc='upper right', prop={'size': 14})
        # plt.grid(True)
        #
        # # 2C. Plot gimbal hub-relative angular acceleration omegaPrime_FM_F
        # plt.figure()
        # plt.clf()
        # plt.plot(timespan, omegaPrime_FM_F[:, 0], label=r'1')
        # plt.plot(timespan, omegaPrime_FM_F[:, 1], label=r'2')
        # plt.plot(timespan, omegaPrime_FM_F[:, 2], label=r'3')
        # plt.title(r'Profiled Angular Acceleration ${}^\mathcal{F} \omega$Prime$_{\mathcal{F}/\mathcal{M}}$',
        #           fontsize=14)
        # plt.ylabel('(deg/s$^2$)', fontsize=14)
        # plt.xlabel('Time (s)', fontsize=14)
        # plt.legend(loc='upper right', prop={'size': 14})
        # plt.grid(True)

        plt.show()
    plt.close("all")

def motorToGimbalAngles(motor1Angle, motor2Angle):

    # Read in the lookup tables
    path_to_file = "/Users/leahkiner/Desktop/motor_to_gimbal_tip_angle.csv"
    motor_to_gimbal_tip_angle = pd.read_csv(path_to_file)
    path_to_file = "/Users/leahkiner/Desktop/motor_to_gimbal_tilt_angle.csv"
    motor_to_gimbal_tilt_angle = pd.read_csv(path_to_file)
    tableStepAngle = 0.5  # [deg]

    motor_to_gimbal_tip_angle_table = motor_to_gimbal_tip_angle.to_numpy()
    motor_to_gimbal_tilt_angle_table = motor_to_gimbal_tilt_angle.to_numpy()

    if motor1Angle % tableStepAngle == 0 and motor2Angle % tableStepAngle == 0:  # Do not need to interpolate
        gimbalAngle1 = pullGimbalAngle(motor1Angle, motor2Angle, motor_to_gimbal_tip_angle_table)
        gimbalAngle2 = pullGimbalAngle(motor1Angle, motor2Angle, motor_to_gimbal_tilt_angle_table)
    elif motor1Angle % tableStepAngle == 0 or motor2Angle % tableStepAngle == 0:  # Linear interpolation required
        if motor1Angle % tableStepAngle == 0:
            lowerMotor2Angle = tableStepAngle * math.floor(motor2Angle / tableStepAngle)
            upperMotor2Angle = tableStepAngle * math.ceil(motor2Angle / tableStepAngle)

            z1_tip = pullGimbalAngle(motor1Angle, lowerMotor2Angle, motor_to_gimbal_tip_angle_table)
            z2_tip = pullGimbalAngle(motor1Angle, upperMotor2Angle, motor_to_gimbal_tip_angle_table)
            gimbalAngle1 = linearInterpolation(lowerMotor2Angle, upperMotor2Angle, z1_tip, z2_tip, motor2Angle)

            z1_tilt = pullGimbalAngle(motor1Angle, lowerMotor2Angle, motor_to_gimbal_tilt_angle_table)
            z2_tilt = pullGimbalAngle(motor1Angle, upperMotor2Angle, motor_to_gimbal_tilt_angle_table)
            gimbalAngle2 = linearInterpolation(lowerMotor2Angle, upperMotor2Angle, z1_tilt, z2_tilt, motor2Angle)
        else:
            lowerMotor1Angle = tableStepAngle * math.floor(motor1Angle / tableStepAngle)
            upperMotor1Angle = tableStepAngle * math.ceil(motor1Angle / tableStepAngle)

            z1_tip = pullGimbalAngle(lowerMotor1Angle, motor2Angle, motor_to_gimbal_tip_angle_table)
            z2_tip = pullGimbalAngle(upperMotor1Angle, motor2Angle, motor_to_gimbal_tip_angle_table)
            gimbalAngle1 = linearInterpolation(lowerMotor1Angle, upperMotor1Angle, z1_tip, z2_tip, motor1Angle)

            z1_tilt = pullGimbalAngle(lowerMotor1Angle, motor2Angle, motor_to_gimbal_tilt_angle_table)
            z2_tilt = pullGimbalAngle(upperMotor1Angle, motor2Angle, motor_to_gimbal_tilt_angle_table)
            gimbalAngle2 = linearInterpolation(lowerMotor1Angle, upperMotor1Angle, z1_tilt, z2_tilt, motor1Angle)
    else:  # Bilinear interpolation required
        lowerMotor1Angle = tableStepAngle * math.floor(motor1Angle / tableStepAngle)
        upperMotor1Angle = tableStepAngle * math.ceil(motor1Angle / tableStepAngle)
        lowerMotor2Angle = tableStepAngle * math.floor(motor2Angle / tableStepAngle)
        upperMotor2Angle = tableStepAngle * math.ceil(motor2Angle / tableStepAngle)

        z11_tip = pullGimbalAngle(lowerMotor1Angle, lowerMotor2Angle, motor_to_gimbal_tip_angle_table)
        z12_tip = pullGimbalAngle(lowerMotor1Angle, upperMotor2Angle, motor_to_gimbal_tip_angle_table)
        z21_tip = pullGimbalAngle(upperMotor1Angle, lowerMotor2Angle, motor_to_gimbal_tip_angle_table)
        z22_tip = pullGimbalAngle(upperMotor1Angle, upperMotor2Angle, motor_to_gimbal_tip_angle_table)

        z11_tilt = pullGimbalAngle(lowerMotor1Angle, lowerMotor2Angle, motor_to_gimbal_tilt_angle_table)
        z12_tilt = pullGimbalAngle(lowerMotor1Angle, upperMotor2Angle, motor_to_gimbal_tilt_angle_table)
        z21_tilt = pullGimbalAngle(upperMotor1Angle, lowerMotor2Angle, motor_to_gimbal_tilt_angle_table)
        z22_tilt = pullGimbalAngle(upperMotor1Angle, upperMotor2Angle, motor_to_gimbal_tilt_angle_table)

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

    return lookup_table_data[motor2Idx][motor1Idx]

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
        1000,  # motor1StepsCommanded
        16000,  # motor2StepsCommanded
        0.01,  # accuracy
    )
