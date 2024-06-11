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

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

@pytest.mark.parametrize("motor1StepsCommanded_Stage1", [211])
@pytest.mark.parametrize("motor2StepsCommanded_Stage1", [211])
@pytest.mark.parametrize("motor1StepsCommanded_Stage2", [0, 53])
@pytest.mark.parametrize("motor2StepsCommanded_Stage2", [0, 53])
@pytest.mark.parametrize("interruptFraction", [0.0, 0.25, 0.5, 0.75])
@pytest.mark.parametrize("accuracy", [1e-4])
def test_twoAxisGimbalInterrupt(show_plots,
                       motor1StepsCommanded_Stage1,
                       motor2StepsCommanded_Stage1,
                       motor1StepsCommanded_Stage2,
                       motor2StepsCommanded_Stage2,
                       interruptFraction,
                       accuracy):
    r"""
    **Validation Test Description**

    This is the second unit test for the two-axis gimbal simulation module. This unit test script specifically tests
    that the gimbal actuates correctly when a current actuation command is interrupted by a new actuation
    command.

    **Test Parameters**

    Args:
        show_plots (bool):                          Variable for choosing whether plots should be displayed
        motor1StepsCommanded_Stage1 (int):          Number of steps commanded to motor 1 during stage 1
        motor2StepsCommanded_Stage1 (int):          Number of steps commanded to motor 2 during stage 1
        motor1StepsCommanded_Stage2 (int):          Number of steps commanded to motor 1 during stage 2
        motor2StepsCommanded_Stage2 (int):          Number of steps commanded to motor 2 during stage 2
        interruptFraction (float):                  Specifies what fraction of a gimbal step is completed when the interrupted message is written
        accuracy:                                   Accuracy used in unit test verification

    **Description of Variables Being Tested**

    This test checks that the gimbal actuates to the correct final angles as a result of the interrupted aactuation.
    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    testTimeStepSec = 0.005  # [s]
    testProcessRate = macros.sec2nano(testTimeStepSec)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Define stepper motor parameters
    motor1ThetaInit = 0.0 * macros.D2R  # [rad]
    motor2ThetaInit = 0.0 * macros.D2R  # [rad]
    motorStepAngle = 0.25 * macros.D2R  # [rad]
    motorStepTime = 0.25  # [s]
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
    Motor1StepCommandMessageData.stepsCommanded = motor1StepsCommanded_Stage1
    Motor1StepCommandMessage = messaging.MotorStepCommandMsg().write(Motor1StepCommandMessageData)
    stepperMotor1.motorStepCommandInMsg.subscribeTo(Motor1StepCommandMessage)

    # Create the step command message for stepper motor 2
    Motor2StepCommandMessageData = messaging.MotorStepCommandMsgPayload()
    Motor2StepCommandMessageData.stepsCommanded = motor2StepsCommanded_Stage1
    Motor2StepCommandMessage = messaging.MotorStepCommandMsg().write(Motor2StepCommandMessageData)
    stepperMotor2.motorStepCommandInMsg.subscribeTo(Motor2StepCommandMessage)

    # Create the two-axis gimbal (Module tested in this script)
    motorToGimbalTipAnglePath = path + "/../motor_to_gimbal_tip_angle.csv"
    motorToGimbalTiltAnglePath = path + "/../motor_to_gimbal_tilt_angle.csv"
    gimbal = twoAxisGimbal.TwoAxisGimbal(motorToGimbalTipAnglePath, motorToGimbalTiltAnglePath)
    gimbal.ModelTag = "twoAxisGimbal"
    gimbal.setMotorStepAngle(motorStepAngle)
    gimbal.setMotorStepTime(motorStepTime)
    gimbal.motor1StepCmdInMsg.subscribeTo(Motor1StepCommandMessage)
    gimbal.motor2StepCmdInMsg.subscribeTo(Motor2StepCommandMessage)
    gimbal.motor1StateInMsg.subscribeTo(stepperMotor1.stepperMotorOutMsg)
    gimbal.motor2StateInMsg.subscribeTo(stepperMotor2.stepperMotorOutMsg)
    unitTestSim.AddModelToTask(unitTaskName, gimbal)

    # Set up data logging for the gimbal module unit test
    gimbalPrescribedRotStateData = gimbal.prescribedRotationOutMsg.recorder()
    gimbalTipTiltAngleData = gimbal.twoAxisGimbalOutMsg.recorder()
    stepperMotor1DataLog = stepperMotor1.stepperMotorOutMsg.recorder()
    stepperMotor2DataLog = stepperMotor2.stepperMotorOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, gimbalPrescribedRotStateData)
    unitTestSim.AddModelToTask(unitTaskName, gimbalTipTiltAngleData)
    unitTestSim.AddModelToTask(unitTaskName, stepperMotor1DataLog)
    unitTestSim.AddModelToTask(unitTaskName, stepperMotor2DataLog)

    # Determine the simulation time if the gimbal actuates stage without interruption
    if np.abs(motor1StepsCommanded_Stage1) > np.abs(motor2StepsCommanded_Stage1):
        simTime_Stage1Segment1 = np.abs(motor2StepsCommanded_Stage1) * motorStepTime  # [s]
        simTime_Stage1Segment2 = (np.abs(motor1StepsCommanded_Stage1)
                                  - np.abs(motor2StepsCommanded_Stage1)) * motorStepTime  # [s]

    elif np.abs(motor1StepsCommanded_Stage1) < np.abs(motor2StepsCommanded_Stage1):
        simTime_Stage1Segment1 = np.abs(motor1StepsCommanded_Stage1) * motorStepTime  # [s]
        simTime_Stage1Segment2 = (np.abs(motor2StepsCommanded_Stage1)
                                  - np.abs(motor1StepsCommanded_Stage1)) * motorStepTime  # [s]

    else:
        simTime_Stage1Segment1 = np.abs(motor1StepsCommanded_Stage1) * motorStepTime  # [s]
        simTime_Stage1Segment2 = 0  # [s]

    simTimeTotal_Stage1 = simTime_Stage1Segment1 + simTime_Stage1Segment2  # [s]

    # Determine the simulation time for the interrupted message
    simTimeInterrupt_Stage1 = motorStepTime * np.ceil(simTimeTotal_Stage1 / (2 * motorStepTime)) + interruptFraction

    # Run the simulation (Stage 1)
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTimeInterrupt_Stage1))
    unitTestSim.ExecuteSimulation()

    # Create the step command message for stepper motor 1 (Stage 2)
    Motor1StepCommandMessageData = messaging.MotorStepCommandMsgPayload()
    Motor1StepCommandMessageData.stepsCommanded = motor1StepsCommanded_Stage2
    Motor1StepCommandMessage = messaging.MotorStepCommandMsg().write(Motor1StepCommandMessageData,
                                                                     unitTestSim.TotalSim.CurrentNanos)

    # Create the step command message for stepper motor 2 (Stage 2)
    Motor2StepCommandMessageData = messaging.MotorStepCommandMsgPayload()
    Motor2StepCommandMessageData.stepsCommanded = motor2StepsCommanded_Stage2
    Motor2StepCommandMessage = messaging.MotorStepCommandMsg().write(Motor2StepCommandMessageData,
                                                                     unitTestSim.TotalSim.CurrentNanos)

    # Re-subscribe module messages for the second simulation chunk (Stage 2)
    stepperMotor1.motorStepCommandInMsg.subscribeTo(Motor1StepCommandMessage)
    stepperMotor2.motorStepCommandInMsg.subscribeTo(Motor2StepCommandMessage)
    gimbal.motor1StepCmdInMsg.subscribeTo(Motor1StepCommandMessage)
    gimbal.motor2StepCmdInMsg.subscribeTo(Motor2StepCommandMessage)

    # Determine the simulation time (Stage 2)
    if np.abs(motor1StepsCommanded_Stage2) > np.abs(motor2StepsCommanded_Stage2):
        simTime_Stage2Segment1 = np.abs(motor2StepsCommanded_Stage2) * motorStepTime  # [s]
        simTime_Stage2Segment2 = (np.abs(motor1StepsCommanded_Stage2)
                                  - np.abs(motor2StepsCommanded_Stage2)) * motorStepTime  # [s]

    elif np.abs(motor1StepsCommanded_Stage2) < np.abs(motor2StepsCommanded_Stage2):
        simTime_Stage2Segment1 = np.abs(motor1StepsCommanded_Stage2) * motorStepTime  # [s]
        simTime_Stage2Segment2 = (np.abs(motor2StepsCommanded_Stage2)
                                  - np.abs(motor1StepsCommanded_Stage2)) * motorStepTime  # [s]

    else:
        simTime_Stage2Segment1 = np.abs(motor1StepsCommanded_Stage2) * motorStepTime  # [s]
        simTime_Stage2Segment2 = 0  # [s]

    simExtraTime = 10.0  # [s]
    simTimeTotal_Stage2 = simTime_Stage2Segment1 + simTime_Stage2Segment2 + simExtraTime  # [s]

    # Run the simulation (Stage 2)
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTimeInterrupt_Stage1 + simTimeTotal_Stage2))
    unitTestSim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    timespan = macros.NANO2SEC * gimbalPrescribedRotStateData.times()  # [s]
    sigma_FM = gimbalPrescribedRotStateData.sigma_FM
    gimbalTipAngle = macros.R2D * gimbalTipTiltAngleData.theta1  # [deg]
    gimbalTiltAngle = macros.R2D * gimbalTipTiltAngleData.theta2  # [deg]
    motor1Theta = macros.R2D * stepperMotor1DataLog.theta  # [deg]
    motor2Theta = macros.R2D * stepperMotor2DataLog.theta  # [deg]
    motor1StepCount = stepperMotor1DataLog.stepCount
    motor2StepCount = stepperMotor2DataLog.stepCount
    motor1CommandedSteps = stepperMotor1DataLog.stepsCommanded
    motor2CommandedSteps = stepperMotor2DataLog.stepsCommanded

    #
    # Unit Test Verification
    #

    # Read in the lookup tables
    path_to_file = "/Users/leahkiner/Desktop/motor_to_gimbal_tip_angle.csv"
    motor_to_gimbal_tip_angle = pd.read_csv(path_to_file)
    path_to_file = "/Users/leahkiner/Desktop/motor_to_gimbal_tilt_angle.csv"
    motor_to_gimbal_tilt_angle = pd.read_csv(path_to_file)
    motor_to_gimbal_tip_angle_table = motor_to_gimbal_tip_angle.to_numpy()
    motor_to_gimbal_tilt_angle_table = motor_to_gimbal_tilt_angle.to_numpy()

    # Compute the maximum possible number of motor steps taken before the interruption
    possibleNumStepsTaken = np.ceil(simTimeInterrupt_Stage1 / motorStepTime)

    # Determine the number of motor steps taken before the interruption
    if possibleNumStepsTaken > motor1StepsCommanded_Stage1:
        motor1StepsTakenBeforeInterrupt = motor1StepsCommanded_Stage1
    else:
        motor1StepsTakenBeforeInterrupt = possibleNumStepsTaken

    if possibleNumStepsTaken > motor2StepsCommanded_Stage1:
        motor2StepsTakenBeforeInterrupt = motor2StepsCommanded_Stage1
    else:
        motor2StepsTakenBeforeInterrupt = possibleNumStepsTaken

    # Determine the true number of steps taken for each motor for the entire simulation
    motor1StepsTaken_Truth = motor1StepsTakenBeforeInterrupt + motor1StepsCommanded_Stage2
    motor2StepsTaken_Truth = motor2StepsTakenBeforeInterrupt + motor2StepsCommanded_Stage2

    # Determine the final motor angles at the end of the simulation
    motor1Theta_Final = motor1ThetaInit + motorStepAngle * motor1StepsTaken_Truth
    motor2Theta_Final = motor2ThetaInit + motorStepAngle * motor2StepsTaken_Truth

    # Compute the true gimbal tip and tilt angles at the end of the simulation
    gimbalTipAngleCheck, gimbalTiltAngleCheck = motorToGimbalAngles(
            macros.R2D * motor1Theta_Final,
            macros.R2D * motor2Theta_Final,
            motor_to_gimbal_tip_angle_table,
            motor_to_gimbal_tilt_angle_table)  # [deg]

    # Check that the gimbal angles converge to the desired values for each actuation segment
    np.testing.assert_allclose(gimbalTipAngleCheck,
                               gimbalTipAngle[-1],
                               atol=accuracy,
                               verbose=True)

    np.testing.assert_allclose(gimbalTiltAngleCheck,
                               gimbalTiltAngle[-1],
                               atol=accuracy,
                               verbose=True)

    # # Print gimbal angle checks
    # print("GIMBAL TIP ANGLE CHECK TRUTH VALUE: ")
    # print(gimbalTipAngleCheck)
    # print("GIMBAL TIP ANGLE MODULE VALUE:")
    # print(gimbalTipAngle[-1])
    #
    # print("\n\nGIMBAL TILT ANGLE CHECK TRUTH VALUE: ")
    # print(gimbalTiltAngleCheck)
    # print("GIMBAL TILT ANGLE MODULE VALUE:")
    # print(gimbalTiltAngle[-1])

    if show_plots:
        # Calculate the initial gimbal tip and tilt angles
        gimbalTipAngleInit, gimbalTiltAngleInit = motorToGimbalAngles(macros.R2D * motor1ThetaInit,
                                                                      macros.R2D * motor2ThetaInit,
                                                                      motor_to_gimbal_tip_angle_table,
                                                                      motor_to_gimbal_tilt_angle_table)
        # 1. Plot the gimbal tip and tilt angles
        # 1A. Plot gimbal tip angle
        gimbalTipAngleInitPlotting = np.ones(len(timespan)) * gimbalTipAngleInit  # [deg]
        gimbalTipAngleRefPlotting = np.ones(len(timespan)) * gimbalTipAngleCheck  # [deg]
        plt.figure()
        plt.clf()
        plt.plot(timespan, gimbalTipAngle, label=r"$\psi$")
        plt.plot(timespan, gimbalTipAngleInitPlotting, '--', label=r"$\psi_0$")
        plt.plot(timespan, gimbalTipAngleRefPlotting, '--', label=r'$\psi_{\text{ref}}$')
        plt.title(r'Gimbal Tip Angle $\psi$', fontsize=14)
        plt.ylabel('(deg)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='center right', prop={'size': 12})
        plt.grid(True)

        # 1B. Plot gimbal tilt angle
        gimbalTiltAngleInitPlotting = np.ones(len(timespan)) * gimbalTiltAngleInit  # [deg]
        gimbalTiltAngleRefPlotting = np.ones(len(timespan)) * gimbalTiltAngleCheck  # [deg]
        plt.figure()
        plt.clf()
        plt.plot(timespan, gimbalTiltAngle, label=r"$\phi$")
        plt.plot(timespan, gimbalTiltAngleInitPlotting, '--', label=r"$\phi_0$")
        plt.plot(timespan, gimbalTiltAngleRefPlotting, '--', label=r'$\phi_{\text{ref}}$')
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
        plt.ylabel(r'$\phi$ (deg)', fontsize=16)
        plt.xlabel(r'$\psi$ (deg)', fontsize=16)
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

        # Plot stepper motor angles
        plt.figure()
        plt.clf()
        plt.plot(timespan, motor1Theta, label=r"$\theta_1$")
        plt.plot(timespan, motor2Theta, label=r"$\theta_2$")
        plt.title(r'Stepper Motor Angles', fontsize=14)
        plt.ylabel('(deg)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='upper right', prop={'size': 12})
        plt.grid(True)

        # Plot steps commanded and motor steps taken for each motor
        plt.figure()
        plt.clf()
        plt.plot(timespan, motor1StepCount, label='Motor 1 Step Count')
        plt.plot(timespan, motor2StepCount, label='Motor 2 Step Count')
        plt.plot(timespan, motor1CommandedSteps, '--', label=' Motor 1 Commanded Steps')
        plt.plot(timespan, motor2CommandedSteps, '--', label=' Motor 2 Commanded Steps')
        plt.title(r'Motor Step History', fontsize=14)
        plt.ylabel('Steps', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='upper right', prop={'size': 12})
        plt.grid(True)

        plt.show()
    plt.close("all")


# This function is used to convert given motor angles to gimbal tip and tilt angles
def motorToGimbalAngles(motor1Angle, motor2Angle, motor_to_gimbal_tip_angle_table, motor_to_gimbal_tilt_angle_table):
    tableStepAngle = 0.5  # [deg]

    if motor1Angle % tableStepAngle == 0 and motor2Angle % tableStepAngle == 0:  # Do not need to interpolate
        gimbalAngle1 = pullGimbalAngle(motor1Angle, motor2Angle, motor_to_gimbal_tip_angle_table)
        gimbalAngle2 = pullGimbalAngle(motor1Angle, motor2Angle, motor_to_gimbal_tilt_angle_table)

    elif motor1Angle % tableStepAngle == 0 or motor2Angle % tableStepAngle == 0:  # Linear interpolation required
        if motor1Angle % tableStepAngle == 0:
            # Find the upper and lower interpolation table angle bounds for motor 2
            lowerMotor2Angle = tableStepAngle * math.floor(motor2Angle / tableStepAngle)
            upperMotor2Angle = tableStepAngle * math.ceil(motor2Angle / tableStepAngle)

            # Linearly interpolate the gimbal tip angle
            y1_tip = pullGimbalAngle(motor1Angle, lowerMotor2Angle, motor_to_gimbal_tip_angle_table)
            y2_tip = pullGimbalAngle(motor1Angle, upperMotor2Angle, motor_to_gimbal_tip_angle_table)
            gimbalAngle1 = linearInterpolation(lowerMotor2Angle, upperMotor2Angle, y1_tip, y2_tip, motor2Angle)

            # Linearly interpolate the gimbal tilt angle
            y1_tilt = pullGimbalAngle(motor1Angle, lowerMotor2Angle, motor_to_gimbal_tilt_angle_table)
            y2_tilt = pullGimbalAngle(motor1Angle, upperMotor2Angle, motor_to_gimbal_tilt_angle_table)
            gimbalAngle2 = linearInterpolation(lowerMotor2Angle, upperMotor2Angle, y1_tilt, y2_tilt, motor2Angle)

        else:
            # Find the upper and lower interpolation table angle bounds for motor 1
            lowerMotor1Angle = tableStepAngle * math.floor(motor1Angle / tableStepAngle)
            upperMotor1Angle = tableStepAngle * math.ceil(motor1Angle / tableStepAngle)

            # Linearly interpolate the gimbal tip angle
            y1_tip = pullGimbalAngle(lowerMotor1Angle, motor2Angle, motor_to_gimbal_tip_angle_table)
            y2_tip = pullGimbalAngle(upperMotor1Angle, motor2Angle, motor_to_gimbal_tip_angle_table)
            gimbalAngle1 = linearInterpolation(lowerMotor1Angle, upperMotor1Angle, y1_tip, y2_tip, motor1Angle)

            # Linearly interpolate the gimbal tilt angle
            y1_tilt = pullGimbalAngle(lowerMotor1Angle, motor2Angle, motor_to_gimbal_tilt_angle_table)
            y2_tilt = pullGimbalAngle(upperMotor1Angle, motor2Angle, motor_to_gimbal_tilt_angle_table)
            gimbalAngle2 = linearInterpolation(lowerMotor1Angle, upperMotor1Angle, y1_tilt, y2_tilt, motor1Angle)

    else:  # Bilinear interpolation required
        # Find the upper and lower interpolation table angle bounds for motor1 and motor 2
        lowerMotor1Angle = tableStepAngle * math.floor(motor1Angle / tableStepAngle)
        upperMotor1Angle = tableStepAngle * math.ceil(motor1Angle / tableStepAngle)
        lowerMotor2Angle = tableStepAngle * math.floor(motor2Angle / tableStepAngle)
        upperMotor2Angle = tableStepAngle * math.ceil(motor2Angle / tableStepAngle)

        # Bilinearly interpolate the gimbal tip angle
        z11_tip = pullGimbalAngle(lowerMotor1Angle, lowerMotor2Angle, motor_to_gimbal_tip_angle_table)
        z12_tip = pullGimbalAngle(lowerMotor1Angle, upperMotor2Angle, motor_to_gimbal_tip_angle_table)
        z21_tip = pullGimbalAngle(upperMotor1Angle, lowerMotor2Angle, motor_to_gimbal_tip_angle_table)
        z22_tip = pullGimbalAngle(upperMotor1Angle, upperMotor2Angle, motor_to_gimbal_tip_angle_table)
        gimbalAngle1 = bilinearInterpolation(lowerMotor1Angle,
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

        # Bilinearly interpolate the gimbal tilt angle
        z11_tilt = pullGimbalAngle(lowerMotor1Angle, lowerMotor2Angle, motor_to_gimbal_tilt_angle_table)
        z12_tilt = pullGimbalAngle(lowerMotor1Angle, upperMotor2Angle, motor_to_gimbal_tilt_angle_table)
        z21_tilt = pullGimbalAngle(upperMotor1Angle, lowerMotor2Angle, motor_to_gimbal_tilt_angle_table)
        z22_tilt = pullGimbalAngle(upperMotor1Angle, upperMotor2Angle, motor_to_gimbal_tilt_angle_table)
        gimbalAngle2 = bilinearInterpolation(lowerMotor1Angle,
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

# This function uses the given interpolation table and motor angles to pull the correct gimbal angle
def pullGimbalAngle(motor1Angle, motor2Angle, lookup_table_data):
    tableMotorStepAngle = 0.5  # [deg]
    motor1Idx = int(motor1Angle / tableMotorStepAngle)
    motor2Idx = int(motor2Angle / tableMotorStepAngle)

    return lookup_table_data[motor2Idx][motor1Idx]

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
    return (1 / ((x2 - x1) * (y2 - y1)) * (z11 * (x2 - x) * (y2 - y)
                                           + z21 * (x - x1) * (y2 - y)
                                           + z12 * (x2 - x) * (y - y1)
                                           + z22 * (x - x1) * (y - y1)))

if __name__ == "__main__":
    test_twoAxisGimbalInterrupt(
        True,  # show_plots
        211,  # motor1StepsCommanded_Stage1
        211,  # motor2StepsCommanded_Stage1
        0,  # motor1StepsCommanded_Stage2
        53,  # motor2StepsCommanded_Stage2
        0.0,  # interruptFraction
        1e-4,  # accuracy
    )
