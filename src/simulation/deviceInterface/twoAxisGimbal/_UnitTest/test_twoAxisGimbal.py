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

@pytest.mark.parametrize("motor1StepsCommanded_Stage1", [157, 351])
@pytest.mark.parametrize("motor2StepsCommanded_Stage1", [157, 351])
@pytest.mark.parametrize("motor1StepsCommanded_Stage2", [0, -53, -104])
@pytest.mark.parametrize("motor2StepsCommanded_Stage2", [0, -53, -104])
@pytest.mark.parametrize("accuracy", [1e-4])
def test_twoAxisGimbal(show_plots,
                       motor1StepsCommanded_Stage1,
                       motor2StepsCommanded_Stage1,
                       motor1StepsCommanded_Stage2,
                       motor2StepsCommanded_Stage2,
                       accuracy):
    r"""
    **Validation Test Description**

    The unit test for this module ensures that the two-axis gimbal is correctly simulated. The two-axis gimbal is
    actuated using two stepper motors that move with a constant step angle and step time. Using the provided
    motor-to-gimbal tip and tilt angle interpolation tables, this unit test computes the truth initial, intermediate,
    and final gimbal tip and tilt angles in time for two separate actuation commands (stage 1 and stage 2). The time
    that the gimbal should arrive at these truth values is also computed. This truth information is checked with the
    simulated gimbal trajectory to ensure that the gimbal starts and ends in the correct locations.

    In this test both stepper motor initial angles are set to zero. These motor angles correspond to the gimbal starting
    from the stowed location of (tip, tilt) = (psi, phi) = (18.34, 0.0) degrees. During the first commanding stage, both
    motors are commanded to move with positive steps. The second commanding stage commands the motors to either move
    zero steps or to move negative steps. Note that both motors must be given either positive or negative steps to
    simulate gimbal motion that is physically possible.

    **Test Parameters**

    Args:
        show_plots (bool):                          Variable for choosing whether plots should be displayed
        motor1StepsCommanded_Stage1 (int):          Number of steps commanded to motor 1 during stage 1
        motor2StepsCommanded_Stage1 (int):          Number of steps commanded to motor 2 during stage 1
        motor1StepsCommanded_Stage2 (int):          Number of steps commanded to motor 1 during stage 2
        motor2StepsCommanded_Stage2 (int):          Number of steps commanded to motor 2 during stage 2
        accuracy:                                   Accuracy used in unit test verification

    **Description of Variables Being Tested**

    Note that both motors are assumed to actuate simultaneously. With this consideration, the unit test checks that
    the gimbal tip and tilt angles converge to the correct values for each "segment" of gimbal motion. If the steps
    commanded to each motor during each stage do not match, there are two assumed segments of motion. The first segment
    of gimbal motion is defined as the time when both motors are actuating together. During the second segment of
    motion, either motor 1 or motor 2 actuates and finishes its number of commanded steps depending on which motor has
    steps left over. If both motors have the same number of steps commanded during a stage, there is no second segment
    of motion.  This test checks that the gimbal angles converge to the correct angles at the end of each segment of
    gimbal motion.
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
    unitTestSim.AddModelToTask(unitTaskName, gimbalPrescribedRotStateData)
    unitTestSim.AddModelToTask(unitTaskName, gimbalTipTiltAngleData)

    # Determine the simulation time
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

    simExtraTime = 10.0  # [s]
    simTimeTotal_Stage1 = simTime_Stage1Segment1 + simTime_Stage1Segment2 + simExtraTime  # [s]

    # Run the simulation (Stage 1)
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTimeTotal_Stage1))
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
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTimeTotal_Stage1 + simTimeTotal_Stage2))
    unitTestSim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    timespan = macros.NANO2SEC * gimbalPrescribedRotStateData.times()  # [s]
    gimbalTipAngle = macros.R2D * gimbalTipTiltAngleData.theta1  # [deg]
    gimbalTiltAngle = macros.R2D * gimbalTipTiltAngleData.theta2  # [deg]
    sigma_FM = gimbalPrescribedRotStateData.sigma_FM
    omega_FM_F = macros.R2D * gimbalPrescribedRotStateData.omega_FM_F  # [deg/s]
    omegaPrime_FM_F = macros.R2D * gimbalPrescribedRotStateData.omegaPrime_FM_F  # [deg/s^2]

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

    gimbalTipAngleCheckList = []
    gimbalTiltAngleCheckList = []
    gimbalTipAngleSimList = []
    gimbalTiltAngleSimList = []

    # Compute the unit test check data for the first stage
    if (motor1StepsCommanded_Stage1 > motor2StepsCommanded_Stage1) or (
            motor1StepsCommanded_Stage1 < motor2StepsCommanded_Stage1):

        # Find the gimbal truth angles at the end of the first segment of stage 1
        if np.abs(motor1StepsCommanded_Stage1) > np.abs(motor2StepsCommanded_Stage1):
            motor1FinalAngle_Stage1Segment1 = motor1ThetaInit + motor2StepsCommanded_Stage1 * motorStepAngle  # [rad]
            motor2FinalAngle_Stage1Segment1 = motor2ThetaInit + motor2StepsCommanded_Stage1 * motorStepAngle  # [rad]
        else:
            motor1FinalAngle_Stage1Segment1 = motor1ThetaInit + motor1StepsCommanded_Stage1 * motorStepAngle  # [rad]
            motor2FinalAngle_Stage1Segment1 = motor2ThetaInit + motor1StepsCommanded_Stage1 * motorStepAngle  # [rad]
        gimbalTipAngleFinal_Stage1Segment1, gimbalTiltAngleFinal_Stage1Segment1 = motorToGimbalAngles(
            macros.R2D * motor1FinalAngle_Stage1Segment1,
            macros.R2D * motor2FinalAngle_Stage1Segment1,
            motor_to_gimbal_tip_angle_table,
            motor_to_gimbal_tilt_angle_table)
        gimbalTipAngleCheckList.append(gimbalTipAngleFinal_Stage1Segment1)  # [deg]
        gimbalTiltAngleCheckList.append(gimbalTiltAngleFinal_Stage1Segment1)  # [deg]

        # Find the gimbal truth angles at the end of the second segment of stage 1
        motor1FinalAngle_Stage1Segment2 = motor1ThetaInit + motor1StepsCommanded_Stage1 * motorStepAngle  # [rad]
        motor2FinalAngle_Stage1Segment2 = motor2ThetaInit + motor2StepsCommanded_Stage1 * motorStepAngle  # [rad]
        gimbalTipAngleFinal_Stage1Segment2, gimbalTiltAngleFinal_Stage1Segment2 = motorToGimbalAngles(
            macros.R2D * motor1FinalAngle_Stage1Segment2,
            macros.R2D * motor2FinalAngle_Stage1Segment2,
            motor_to_gimbal_tip_angle_table,
            motor_to_gimbal_tilt_angle_table)  # [deg]
        gimbalTipAngleCheckList.append(gimbalTipAngleFinal_Stage1Segment2)  # [deg]
        gimbalTiltAngleCheckList.append(gimbalTiltAngleFinal_Stage1Segment2)  # [deg]

        # Grab the simulated gimbal angles at the end of each segment of stage 1
        stopTimeIdx_Stage1Segment1 = int(round(simTime_Stage1Segment1 / testTimeStepSec)) + 1
        stopTimeIdx_Stage1Segment2 = int(round((simTime_Stage1Segment1 + simTime_Stage1Segment2) / testTimeStepSec)) + 1
        gimbalTipAngleSimList.append(gimbalTipAngle[stopTimeIdx_Stage1Segment1])  # [deg]
        gimbalTipAngleSimList.append(gimbalTipAngle[stopTimeIdx_Stage1Segment2])  # [deg]
        gimbalTiltAngleSimList.append(gimbalTiltAngle[stopTimeIdx_Stage1Segment1])  # [deg]
        gimbalTiltAngleSimList.append(gimbalTiltAngle[stopTimeIdx_Stage1Segment2])  # [deg]

    else:
        # Find the gimbal truth angles at the end of stage 1
        motor1FinalAngle_Stage1Segment1 = motor1ThetaInit + motor1StepsCommanded_Stage1 * motorStepAngle  # [rad]
        motor2FinalAngle_Stage1Segment1 = motor2ThetaInit + motor2StepsCommanded_Stage1 * motorStepAngle  # [rad]
        gimbalTipAngleFinal, gimbalTiltAngleFinal = motorToGimbalAngles(macros.R2D * motor1FinalAngle_Stage1Segment1,
                                                                        macros.R2D * motor2FinalAngle_Stage1Segment1,
                                                                        motor_to_gimbal_tip_angle_table,
                                                                        motor_to_gimbal_tilt_angle_table)  # [deg]
        gimbalTipAngleCheckList.append(gimbalTipAngleFinal)  # [deg]
        gimbalTiltAngleCheckList.append(gimbalTiltAngleFinal)  # [deg]
        motor1FinalAngle_Stage1Segment2 = motor1FinalAngle_Stage1Segment1
        motor2FinalAngle_Stage1Segment2 = motor2FinalAngle_Stage1Segment1

        # Grab the simulated gimbal angles at the end of stage 1
        stopTimeIdx_Stage1 = int(round((simTime_Stage1Segment1 + simTime_Stage1Segment2) / testTimeStepSec)) + 1
        gimbalTipAngleSimList.append(gimbalTipAngle[stopTimeIdx_Stage1])  # [deg]
        gimbalTiltAngleSimList.append(gimbalTiltAngle[stopTimeIdx_Stage1])  # [deg]

    # Compute the unit test check data for the second stage
    if (motor1StepsCommanded_Stage2 > motor2StepsCommanded_Stage2) or (
            motor1StepsCommanded_Stage2 < motor2StepsCommanded_Stage2):
        # Find the gimbal truth angles at the end of the first segment of stage 2
        if np.abs(motor1StepsCommanded_Stage2) > np.abs(motor2StepsCommanded_Stage2):
            motor1FinalAngle_Stage2Segment1 = (motor1FinalAngle_Stage1Segment2
                                               + motor2StepsCommanded_Stage2 * motorStepAngle)  # [rad]
            motor2FinalAngle_Stage2Segment1 = (motor2FinalAngle_Stage1Segment2
                                               + motor2StepsCommanded_Stage2 * motorStepAngle)  # [rad]
        else:
            motor1FinalAngle_Stage2Segment1 = (motor1FinalAngle_Stage1Segment2
                                               + motor1StepsCommanded_Stage2 * motorStepAngle)  # [rad]
            motor2FinalAngle_Stage2Segment1 = (motor2FinalAngle_Stage1Segment2
                                               + motor1StepsCommanded_Stage2 * motorStepAngle)  # [rad]
        gimbalTipAngleFinal_Stage2Segment1, gimbalTiltAngleFinal_Stage2Segment1 = motorToGimbalAngles(
            macros.R2D * motor1FinalAngle_Stage2Segment1,
            macros.R2D * motor2FinalAngle_Stage2Segment1,
            motor_to_gimbal_tip_angle_table,
            motor_to_gimbal_tilt_angle_table)
        gimbalTipAngleCheckList.append(gimbalTipAngleFinal_Stage2Segment1)  # [deg]
        gimbalTiltAngleCheckList.append(gimbalTiltAngleFinal_Stage2Segment1)  # [deg]

        # Find the gimbal truth angles at the end of the second segment of stage 2
        motor1FinalAngle_Stage2Segment2 = (motor1FinalAngle_Stage1Segment2
                                           + motor1StepsCommanded_Stage2 * motorStepAngle)  # [rad]
        motor2FinalAngle_Stage2Segment2 = (motor2FinalAngle_Stage1Segment2
                                           + motor2StepsCommanded_Stage2 * motorStepAngle)  # [rad]
        gimbalTipAngleFinal_Stage2Segment2, gimbalTiltAngleFinal_Stage2Segment2 = motorToGimbalAngles(
            macros.R2D * motor1FinalAngle_Stage2Segment2,
            macros.R2D * motor2FinalAngle_Stage2Segment2,
            motor_to_gimbal_tip_angle_table,
            motor_to_gimbal_tilt_angle_table)  # [deg]
        gimbalTipAngleCheckList.append(gimbalTipAngleFinal_Stage2Segment2)  # [deg]
        gimbalTiltAngleCheckList.append(gimbalTiltAngleFinal_Stage2Segment2)  # [deg]

        # Grab the simulated gimbal angles at the end of each segment of stage 2
        stopTimeIdx_Stage2Segment1 = int(round((simTimeTotal_Stage1 + simTime_Stage2Segment1) / testTimeStepSec)) + 1
        stopTimeIdx_Stage2Segment2 = int(
            round((simTimeTotal_Stage1 + simTime_Stage2Segment1 + simTime_Stage2Segment2) / testTimeStepSec)) + 1
        gimbalTipAngleSimList.append(gimbalTipAngle[stopTimeIdx_Stage2Segment1])  # [deg]
        gimbalTipAngleSimList.append(gimbalTipAngle[stopTimeIdx_Stage2Segment2])  # [deg]
        gimbalTiltAngleSimList.append(gimbalTiltAngle[stopTimeIdx_Stage2Segment1])  # [deg]
        gimbalTiltAngleSimList.append(gimbalTiltAngle[stopTimeIdx_Stage2Segment2])  # [deg]

    else:
        # Find the gimbal truth angles at the end of stage 2
        motor1FinalAngle_Stage2Segment1 = (motor1FinalAngle_Stage1Segment2
                                           + motor1StepsCommanded_Stage2 * motorStepAngle)  # [rad]
        motor2FinalAngle_Stage2Segment1 = (motor2FinalAngle_Stage1Segment2
                                           + motor2StepsCommanded_Stage2 * motorStepAngle)  # [rad]
        gimbalTipAngleFinal, gimbalTiltAngleFinal = motorToGimbalAngles(macros.R2D * motor1FinalAngle_Stage2Segment1,
                                                                        macros.R2D * motor2FinalAngle_Stage2Segment1,
                                                                        motor_to_gimbal_tip_angle_table,
                                                                        motor_to_gimbal_tilt_angle_table)  # [deg]
        gimbalTipAngleCheckList.append(gimbalTipAngleFinal)  # [deg]
        gimbalTiltAngleCheckList.append(gimbalTiltAngleFinal)  # [deg]

        # Grab the simulated gimbal angles at the end of stage 2
        stopTimeIdx_Stage2 = int(
            round((simTimeTotal_Stage1 + simTime_Stage2Segment1 + simTime_Stage2Segment2) / testTimeStepSec)) + 1
        gimbalTipAngleSimList.append(gimbalTipAngle[stopTimeIdx_Stage2])  # [deg]
        gimbalTiltAngleSimList.append(gimbalTiltAngle[stopTimeIdx_Stage2])  # [deg]

    # Check that the gimbal angles converge to the desired values for each actuation segment
    np.testing.assert_allclose(gimbalTipAngleCheckList,
                               gimbalTipAngleSimList,
                               atol=accuracy,
                               verbose=True)

    np.testing.assert_allclose(gimbalTiltAngleCheckList,
                               gimbalTiltAngleSimList,
                               atol=accuracy,
                               verbose=True)

    # # Print gimbal angle checks
    # print("GIMBAL TIP ANGLE CHECK TRUTH VALUES: ")
    # print(gimbalTipAngleCheckList)
    # print("GIMBAL TIP ANGLE MODULE VALUES:")
    # print(gimbalTipAngleSimList)
    #
    # print("\n\nGIMBAL TILT ANGLE CHECK TRUTH VALUES: ")
    # print(gimbalTiltAngleCheckList)
    # print("GIMBAL TILT ANGLE MODULE VALUES:")
    # print(gimbalTiltAngleSimList)

    if show_plots:
        # Calculate the initial gimbal tip and tilt angles
        gimbalTipAngleInit, gimbalTiltAngleInit = motorToGimbalAngles(macros.R2D * motor1ThetaInit,
                                                                      macros.R2D * motor2ThetaInit,
                                                                      motor_to_gimbal_tip_angle_table,
                                                                      motor_to_gimbal_tilt_angle_table)
        # 1. Plot the gimbal tip and tilt angles
        # 1A. Plot gimbal tip angle
        gimbalTipAngleInitPlotting = np.ones(len(timespan)) * gimbalTipAngleInit  # [deg]
        plt.figure()
        plt.clf()
        plt.plot(timespan, gimbalTipAngle, label=r"$\psi$")
        plt.plot(timespan, gimbalTipAngleInitPlotting, '--', label=r"$\psi_0$")
        for idx in range(len(gimbalTipAngleCheckList)):
            gimbalTipAnglePlotting = np.ones(len(timespan)) * gimbalTipAngleCheckList[idx]  # [deg]
            plt.plot(timespan, gimbalTipAnglePlotting, '--', label=r'$\psi_{\text{ref}_{' + str(idx + 1) + '}}$')
        plt.title(r'Gimbal Tip Angle $\psi$', fontsize=14)
        plt.ylabel('(deg)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='center right', prop={'size': 12})
        plt.grid(True)

        # 1B. Plot gimbal tilt angle
        gimbalTiltAngleInitPlotting = np.ones(len(timespan)) * gimbalTiltAngleInit  # [deg]
        plt.figure()
        plt.clf()
        plt.plot(timespan, gimbalTiltAngle, label=r"$\phi$")
        plt.plot(timespan, gimbalTiltAngleInitPlotting, '--', label=r"$\phi_0$")
        for idx in range(len(gimbalTiltAngleCheckList)):
            gimbalTiltAnglePlotting = np.ones(len(timespan)) * gimbalTiltAngleCheckList[idx]  # [deg]
            plt.plot(timespan, gimbalTiltAnglePlotting, '--', label=r'$\phi_{\text{ref}_{' + str(idx + 1) + '}}$')
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
    test_twoAxisGimbal(
        True,  # show_plots
        300,  # motor1StepsCommanded_Stage1
        100,  # motor2StepsCommanded_Stage1
        -50,  # motor1StepsCommanded_Stage2
        -100,  # motor2StepsCommanded_Stage2
        1e-4,  # accuracy
    )
