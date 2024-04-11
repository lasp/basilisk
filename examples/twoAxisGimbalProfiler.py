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

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import math
from matplotlib import collections as mc

from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel
from Basilisk.simulation import stepperMotor
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

def test_twoAxisGimbalProfiler(show_plots):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.SetProgressBar(True)

    testTimeStepSec = 0.5  # [s]
    testProcessRate = macros.sec2nano(testTimeStepSec)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the stepper motor simulation modules
    stepAngle = macros.D2R * 1.0  # [rad]
    stepTime = 1.0  # [s]
    initialMotorAngle1 = 0.0  # [deg]
    initialMotorAngle2 = 0.0  # [deg]

    StepperMotor1 = stepperMotor.StepperMotor()
    StepperMotor1.setThetaInit(initialMotorAngle1)
    StepperMotor1.setStepAngle(stepAngle)
    StepperMotor1.setStepTime(stepTime)
    StepperMotor1.setThetaDDotMax(stepAngle / (0.25 * stepTime * stepTime))
    StepperMotor1.ModelTag = "StepperMotor1"
    unitTestSim.AddModelToTask(unitTaskName, StepperMotor1)

    StepperMotor2 = stepperMotor.StepperMotor()
    StepperMotor2.setThetaInit(initialMotorAngle2)
    StepperMotor2.setStepAngle(stepAngle)
    StepperMotor2.setStepTime(stepTime)
    StepperMotor2.setThetaDDotMax(stepAngle / (0.25 * stepTime * stepTime))
    StepperMotor2.ModelTag = "StepperMotor2"
    unitTestSim.AddModelToTask(unitTaskName, StepperMotor2)

    # Create the stepper motor fsw messages
    motor1StepsCommanded = 5
    motor2StepsCommanded = 5

    Motor1StepCommandMessageData = messaging.MotorStepCommandMsgPayload()
    Motor1StepCommandMessageData.stepsCommanded = motor1StepsCommanded
    Motor1StepCommandMessage = messaging.MotorStepCommandMsg().write(Motor1StepCommandMessageData)

    Motor2StepCommandMessageData = messaging.MotorStepCommandMsgPayload()
    Motor2StepCommandMessageData.stepsCommanded = motor2StepsCommanded
    Motor2StepCommandMessage = messaging.MotorStepCommandMsg().write(Motor2StepCommandMessageData)

    StepperMotor1.motorStepCommandInMsg.subscribeTo(Motor1StepCommandMessage)
    StepperMotor2.motorStepCommandInMsg.subscribeTo(Motor2StepCommandMessage)

    # Create the two-axis gimbal simulation module
    rotAxis_1 = np.array([1.0, 0.0, 0.0])
    rotAxis_2 = np.array([0.0, 1.0, 0.0])
    path_to_file = "/Users/leahkiner/Repositories/BASILISK/examples/gimbal_data.csv"
    gimbal_data = pd.read_csv(path_to_file)

    twoAxisGimbal = TwoAxisGimbalProfiler(rotAxis_1, rotAxis_2, gimbal_data)
    twoAxisGimbal.ModelTag = "twoAxisGimbal"
    unitTestSim.AddModelToTask(unitTaskName, twoAxisGimbal)

    twoAxisGimbal.stepperMotor1InMsg.subscribeTo(StepperMotor1.stepperMotorOutMsg)
    twoAxisGimbal.stepperMotor2InMsg.subscribeTo(StepperMotor2.stepperMotorOutMsg)

    # Set up data logging
    gimbalAnglesDataLog = twoAxisGimbal.twoAxisGimbalOutMsg.recorder()
    gimbalPrescribedMotionDataLog = twoAxisGimbal.prescribedRotationOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, gimbalAnglesDataLog)
    unitTestSim.AddModelToTask(unitTaskName, gimbalPrescribedMotionDataLog)

    # Run the simulation
    simTime = 5 * 60  # [s]
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))
    unitTestSim.ExecuteSimulation()

    # Extract logged data
    timespan = macros.NANO2SEC * gimbalAnglesDataLog.times()  # [s]
    gimbalAngle1 = macros.R2D * gimbalAnglesDataLog.theta1  # [deg]
    gimbalAngle2 = macros.R2D * gimbalAnglesDataLog.theta2  # [deg]
    omega_FM_F = macros.R2D * gimbalPrescribedMotionDataLog.omega_FM_F  # [deg/s]
    omegaPrime_FM_F = macros.R2D * gimbalPrescribedMotionDataLog.omegaPrime_FM_F  # [deg/s^2]
    sigma_FM = gimbalPrescribedMotionDataLog.sigma_FM

    if show_plots:
        # 1. Plot gimbal angles
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
        plt.plot(gimbalAngle1, gimbalAngle2, linewidth=1, color='black')
        ax.scatter(gimbalAngle1[0], gimbalAngle2[0], marker='.', linewidth=4, color='springgreen', label='Initial')
        ax.scatter(gimbalAngle1[-1], gimbalAngle2[-1], marker='*', linewidth=4, color='magenta', label='Final')
        plt.title('Gimbal Trajectory', fontsize=14)
        plt.ylabel(r'Angle 1: $\psi$ (deg)', fontsize=16)
        plt.xlabel(r'Angle 2: $\phi$ (deg)', fontsize=16)
        plt.legend(loc='upper right', prop={'size': 12})
        plt.grid(True)
        plt.axis("equal")

        # Plot prescribed gimbal states on a single figure
        fig, plts = plt.subplots(3, 1, sharex=True)
        l1, = plts[0].plot(timespan, sigma_FM[:, 0], color='darkturquoise')
        l2, = plts[0].plot(timespan, sigma_FM[:, 1], color='orchid')
        l3, = plts[0].plot(timespan, sigma_FM[:, 2], color='mediumslateblue')
        l4, = plts[1].plot(timespan, omega_FM_F[:, 0], color='darkturquoise')
        l5, = plts[1].plot(timespan, omega_FM_F[:, 1], color='orchid')
        l6, = plts[1].plot(timespan, omega_FM_F[:, 2], color='mediumslateblue')
        l7, = plts[2].plot(timespan, omegaPrime_FM_F[:, 0], color='darkturquoise')
        l8, = plts[2].plot(timespan, omegaPrime_FM_F[:, 1], color='orchid')
        l9, = plts[2].plot(timespan, omegaPrime_FM_F[:, 2], color='mediumslateblue')
        plts.flat[0].set(ylabel=r'$\sigma_{\mathcal{F} / \mathcal{M}}$')
        plts.flat[1].set(ylabel=r'${}^{\mathcal{F}} \omega_{\mathcal{F} / \mathcal{M}}$ (deg/s)')
        plts.flat[2].set(xlabel='Time (s)', ylabel=r'${}^{\mathcal{F}} \alpha_{\mathcal{F} / \mathcal{M}}$ (deg/s$^2$)')
        plts[0].legend((l1, l2, l3), (r'$\sigma_{1}$', r'$\sigma_{2}$', r'$\sigma_{3}$',), loc='upper right', prop={'size': 12})
        plts[1].legend((l4, l5, l6), (r'$\omega_{1}$', r'$\omega_{2}$', r'$\omega_{3}$',), loc='upper right', prop={'size': 12})
        plts[2].legend((l7, l8, l9), (r'$\alpha_{1}$', r'$\alpha_{2}$', r'$\alpha_{3}$',), loc='upper right', prop={'size': 12})
        plts[0].set_title('Gimbal Prescribed States')
        plts[0].grid(True)
        plts[1].grid(True)
        plts[2].grid(True)

        plt.show()
    plt.close("all")

class TwoAxisGimbalProfiler(sysModel.SysModel):
    """
    Python module to simulate a prescribed motion two-axis gimbal
    """
    def __init__(self, rotHat_1, rotHat_2, gimbal_data):
        super(TwoAxisGimbalProfiler, self).__init__()

        self.rotHat_1 = rotHat_1
        self.rotHat_2 = rotHat_2
        self.gimbal_data = gimbal_data
        self.motor1Theta = 0.0  # [rad]
        self.motor2Theta = 0.0  # [rad]
        self.motor1ThetaDot = 0.0  # [rad/s]
        self.motor2ThetaDot = 0.0  # [rad/s]
        self.motor1ThetaDDot = 0.0  # [rad/s^2]
        self.motor2ThetaDDot = 0.0  # [rad/s^2]

        self.stepperMotor1InMsg = messaging.StepperMotorMsgReader()
        self.stepperMotor2InMsg = messaging.StepperMotorMsgReader()
        self.twoAxisGimbalOutMsg = messaging.TwoAxisGimbalMsg()
        self.prescribedRotationOutMsg = messaging.PrescribedRotationMsg()

        return

    def Reset(self, CurrentSimNanos):
        # Ensure the input messages are linked
        if not self.stepperMotor1InMsg.isLinked():
            self.bskLogger.bskLog(
                bskLogging.BSK_ERROR, "TwoAxisGimbalProfiler.stepperMotor1InMsg is not linked."
            )
        if not self.stepperMotor2InMsg.isLinked():
            self.bskLogger.bskLog(
                bskLogging.BSK_ERROR, "TwoAxisGimbalProfiler.stepperMotor2InMsg is not linked."
            )

        # # Initialize the output messages
        # payload = self.twoAxisGimbalOutMsg.zeroMsgPayload
        # payload.theta1 = 0.0  # [rad]
        # payload.theta2 = 0.0  # [rad]
        # self.twoAxisGimbalOutMsg.write(payload, CurrentSimNanos, self.moduleID)
        #
        # payload = self.prescribedRotationOutMsg.zeroMsgPayload
        # payload.omega_FM_F = np.array([0, 0, 0])  # [rad/s]
        # payload.omegaPrime_FM_F = np.array([0, 0, 0])  # [rad/s^2]
        # payload.sigma_FM = np.array([0, 0, 0])
        # self.prescribedRotationOutMsg.write(payload, CurrentSimNanos, self.moduleID)

        return

    def UpdateState(self, CurrentSimNanos):
        # Read the input messages
        motor1InMsgBuffer = self.stepperMotor1InMsg()
        motor2InMsgBuffer = self.stepperMotor2InMsg()

        # Grab the input message data
        self.motor1Theta = motor1InMsgBuffer.theta  # [rad]
        self.motor2Theta = motor2InMsgBuffer.theta  # [rad]
        self.motor1ThetaDot = motor1InMsgBuffer.thetaDot  # [rad/s]
        self.motor2ThetaDot = motor2InMsgBuffer.thetaDot  # [rad/s]
        self.motor1ThetaDDot = motor1InMsgBuffer.thetaDDot  # [rad/s^2]
        self.motor2ThetaDDot = motor2InMsgBuffer.thetaDDot  # [rad/s^2]

        # Interpolate the gimbal angles from the motor angles
        gimbalTheta1, gimbalTheta2, gimbalThetaDot1, gimbalThetaDot2, gimbalThetaDDot1, gimbalThetaDDot2 = self.motorToGimbal()

        # Determine gimbal attitude relative to the mount frame
        prv_F1M = gimbalTheta1 * self.rotHat_1
        prv_F2F1 = gimbalTheta2 * self.rotHat_2
        dcm_F1M = rbk.PRV2C(prv_F1M)
        dcm_F2F1 = rbk.PRV2C(prv_F2F1)
        dcm_FM = np.matmul(dcm_F2F1, dcm_F1M)

        # Solve for the gimbal prescribed rotational states
        omega_FM_F = np.matmul(dcm_FM,  np.transpose(gimbalThetaDot1 * self.rotHat_1)) + (gimbalThetaDot2 * self.rotHat_2)  # [rad/s]
        omegaPrime_FM_F = np.matmul(dcm_FM, np.transpose(gimbalThetaDDot1 * self.rotHat_1)) + (gimbalThetaDDot2 * self.rotHat_2)  # [rad/s^2]
        sigma_FM = rbk.C2MRP(dcm_FM)

        # Create the output message buffers
        twoAxisGimbalOutMsgBuffer = messaging.TwoAxisGimbalMsgPayload()
        prescribedRotationOutMsgBuffer = messaging.PrescribedRotationMsgPayload()

        # Populate the output message buffer data
        twoAxisGimbalOutMsgBuffer.theta1 = gimbalTheta1
        twoAxisGimbalOutMsgBuffer.theta2 = gimbalTheta2
        prescribedRotationOutMsgBuffer.omega_FM_F = omega_FM_F
        prescribedRotationOutMsgBuffer.omegaPrime_FM_F = omegaPrime_FM_F
        prescribedRotationOutMsgBuffer.sigma_FM = sigma_FM

        # Write the output messages
        self.twoAxisGimbalOutMsg.write(twoAxisGimbalOutMsgBuffer, CurrentSimNanos, self.moduleID)
        self.prescribedRotationOutMsg.write(prescribedRotationOutMsgBuffer, CurrentSimNanos, self.moduleID)

        return
    
    def motorToGimbal(self):
        tableStepAngle = 2.5  # [deg]

        # Find motor angles in the provided table that most closely match to motor angles
        if self.motor1ThetaDot > 0.0:
            motor1 = True
        else:
            motor1 = False

        if motor1:       
            lowerMotorAngle = tableStepAngle * math.floor(self.motor1Theta / tableStepAngle)
            upperMotorAngle = tableStepAngle * math.ceil(self.motor1Theta / tableStepAngle)
        else:
            lowerMotorAngle = tableStepAngle * math.floor(self.motor2Theta / tableStepAngle)
            upperMotorAngle = tableStepAngle * math.ceil(self.motor2Theta / tableStepAngle)

        # Extract the correct data for interpolation
        numRows = self.gimbal_data.shape[0]
        numCols = self.gimbal_data.shape[1]
        lowerGimbal1Angle = 360
        lowerGimbal2Angle = 360
        upperGimbal1Angle = 360
        upperGimbal2Angle = 360

        for idx in range(numRows):
            if motor1:
                if ((self.gimbal_data.iat[idx, 0] == lowerMotorAngle) and (self.gimbal_data.iat[idx, 1] == self.motor2Theta)):
                    lowerGimbal1Angle = self.gimbal_data.iat[idx, 2]
                    lowerGimbal2Angle = self.gimbal_data.iat[idx, 3]
    
                if ((self.gimbal_data.iat[idx, 0] == upperMotorAngle) and (self.gimbal_data.iat[idx, 1] == self.motor2Theta)):
                    upperGimbal1Angle = self.gimbal_data.iat[idx, 2]
                    upperGimbal2Angle = self.gimbal_data.iat[idx, 3]
            else:
                if ((self.gimbal_data.iat[idx, 1] == lowerMotorAngle) and (self.gimbal_data.iat[idx, 0] == self.motor1Theta)):
                    lowerGimbal1Angle = self.gimbal_data.iat[idx, 2]
                    lowerGimbal2Angle = self.gimbal_data.iat[idx, 3]
    
                if ((self.gimbal_data.iat[idx, 1] == upperMotorAngle) and (self.gimbal_data.iat[idx, 0] == self.motor1Theta)):
                    upperGimbal1Angle = self.gimbal_data.iat[idx, 2]
                    upperGimbal2Angle = self.gimbal_data.iat[idx, 3]
    
            if ((lowerGimbal1Angle != 360) and (lowerGimbal2Angle != 360) and (upperGimbal1Angle != 360) and (upperGimbal2Angle != 360)):
                break
    
        # Interpolate data to get estimated gimbal angles
        if motor1:
            if upperMotorAngle != lowerMotorAngle:
                gimbal1Angle = ((lowerGimbal1Angle * (upperMotorAngle - self.motor1Theta)) + (upperGimbal1Angle * (self.motor1Theta - lowerMotorAngle))) / (upperMotorAngle - lowerMotorAngle)
                gimbal2Angle = ((lowerGimbal2Angle * (upperMotorAngle - self.motor1Theta)) + (upperGimbal2Angle * (self.motor1Theta - lowerMotorAngle))) / (upperMotorAngle - lowerMotorAngle)
                gimbal1AngleRate = self.motor1ThetaDot * ((upperGimbal1Angle - lowerGimbal1Angle) / (upperMotorAngle - lowerMotorAngle))
                gimbal2AngleRate = self.motor1ThetaDot * ((upperGimbal2Angle - lowerGimbal2Angle) / (upperMotorAngle - lowerMotorAngle))
                gimbal1AngularAccel = self.motor1ThetaDDot * ((upperGimbal1Angle - lowerGimbal1Angle) / (upperMotorAngle - lowerMotorAngle))
                gimbal2AngularAccel = self.motor1ThetaDDot * ((upperGimbal2Angle - lowerGimbal2Angle) / (upperMotorAngle - lowerMotorAngle))
            else:
                gimbal1Angle = lowerGimbal1Angle
                gimbal2Angle = lowerGimbal2Angle
                gimbal1AngleRate = (gimbal1Angle / self.motor1Theta) * self.motor1ThetaDot
                gimbal2AngleRate = (gimbal2Angle / self.motor1Theta) * self.motor1ThetaDot
                gimbal1AngularAccel = (gimbal1Angle / self.motor1Theta) * self.motor1ThetaDDot
                gimbal2AngularAccel = (gimbal2Angle / self.motor1Theta) * self.motor1ThetaDDot
        else:
            if upperMotorAngle != lowerMotorAngle:
                gimbal1Angle = ((lowerGimbal1Angle * (upperMotorAngle - self.motor2Theta)) + (upperGimbal1Angle * (self.motor2Theta - lowerMotorAngle))) / (upperMotorAngle - lowerMotorAngle)
                gimbal2Angle = ((lowerGimbal2Angle * (upperMotorAngle - self.motor2Theta)) + (upperGimbal2Angle * (self.motor2Theta - lowerMotorAngle))) / (upperMotorAngle - lowerMotorAngle)
                gimbal1AngleRate = self.motor2ThetaDot * ((upperGimbal1Angle - lowerGimbal1Angle) / (upperMotorAngle - lowerMotorAngle))
                gimbal2AngleRate = self.motor2ThetaDot * ((upperGimbal2Angle - lowerGimbal2Angle) / (upperMotorAngle - lowerMotorAngle))
                gimbal1AngularAccel = self.motor2ThetaDDot * ((upperGimbal1Angle - lowerGimbal1Angle) / (upperMotorAngle - lowerMotorAngle))
                gimbal2AngularAccel = self.motor2ThetaDDot * ((upperGimbal2Angle - lowerGimbal2Angle) / (upperMotorAngle - lowerMotorAngle))
            else:
                gimbal1Angle = lowerGimbal1Angle
                gimbal2Angle = lowerGimbal2Angle
                gimbal1AngleRate = (gimbal1Angle / self.motor2Theta) * self.motor2ThetaDot
                gimbal2AngleRate = (gimbal2Angle / self.motor2Theta) * self.motor2ThetaDot
                gimbal1AngularAccel = (gimbal1Angle / self.motor2Theta) * self.motor2ThetaDDot
                gimbal2AngularAccel = (gimbal2Angle / self.motor2Theta) * self.motor2ThetaDDot

        return macros.D2R * gimbal1Angle, macros.D2R * gimbal2Angle, macros.D2R * gimbal1AngleRate, macros.D2R * gimbal2AngleRate, macros.D2R * gimbal1AngularAccel, macros.D2R * gimbal2AngularAccel

if __name__ == "__main__":
    test_twoAxisGimbalProfiler(
        True,  # show_plots
    )
