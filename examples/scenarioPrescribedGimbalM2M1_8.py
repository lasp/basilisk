#
#  ISC License

#  Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.

#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

#
# Gimbal Prescribed Motion Scenario
#
# Purpose:  Simulate the prescribed gimbal motion using two single-axis stepper motor actuators.
# Author:   Leah Kiner
# Creation Date:  July 31, 2023
#

import os
import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from Basilisk.utilities import RigidBodyKinematics as rbk
import pylab as pl
from matplotlib import collections as mc
import matplotlib
matplotlib.rc('xtick', labelsize=16)
matplotlib.rc('ytick', labelsize=16)
from matplotlib import rcParams


def run():
    deg2Rad = np.pi / 180

    path_to_file = "/Users/leahkiner/Repositories/MAX_Basilisk/examples/gimbal_dataPaper.csv"
    gimbal_data = pd.read_csv(path_to_file)

    # Define stepper motor fixed parameters
    motorStepAngle = 0.5  # [deg]
    tableStepAngle = 2.0  # [deg]

    # Define gimbal fixed parameters
    rotAxis1 = np.array([1.0, 0.0, 0.0])
    rotAxis2 = np.array([0.0, 1.0, 0.0])

    # Choose which motor is used
    motor1 = False
    motor2 = True

    # Choose method of determining gimbal attitude
    oneTwo = True
    twoOne = False

    # Define initial motor states
    motor1InitAngle = 0
    motor2InitAngle = 0
    gimbal1InitAngle = 25.0
    gimbal2InitAngle = 0.0
    thetaDotInit = 0.0
    thetaDotRef = 0.0

    # Define number of steps each motor takes
    numSteps1 = 64  # Note that numSteps * motorStepAngle must be in increments of 2 degrees
    numSteps2 = 64

    # Check to make sure that the final motor 1 angle exists in a row of the given motor/gimbal table data
    if ((numSteps1 * motorStepAngle) % 2.0 != 0):
        print("ERROR: Motor 1 Must End At An Increment of 2.1 Degrees")

    # Calculate reference motor angles from given steps
    motor1RefAngle = motor1InitAngle + (numSteps1 * motorStepAngle)
    motor2RefAngle = motor2InitAngle + (numSteps2 * motorStepAngle)

    # Create storage arrays for data
    motor1AngleData = np.array(motor1InitAngle)
    motor1AngleRateData = np.array(thetaDotInit)
    motor1AngularAccelData = np.array(0.0)
    motor2AngleData = np.array(motor2InitAngle)
    motor2AngleRateData = np.array(thetaDotInit)
    motor2AngularAccelData = np.array(0.0)
    gimbal1AngleData = np.array(gimbal1InitAngle)
    gimbal2AngleData = np.array(gimbal2InitAngle)
    thrustAxis = np.array([0.0, 0.0, 0.0])

    dcm1 = np.array([[1,  0, 0],
                    [0, np.cos(deg2Rad * gimbal1InitAngle), np.sin(deg2Rad * gimbal1InitAngle)],
                    [0, -np.sin(deg2Rad * gimbal1InitAngle), np.cos(deg2Rad * gimbal1InitAngle)]])
    dcm2 = np.array([[np.cos(deg2Rad * gimbal2InitAngle),  0, -np.sin(deg2Rad * gimbal2InitAngle)],
                     [0, 1, 0],
                     [np.sin(deg2Rad * gimbal2InitAngle),  0, np.cos(deg2Rad * gimbal2InitAngle)]])
    dcm_FMInit = np.matmul(dcm2, dcm1)
    sigma_FMInit = rbk.C2MRP(dcm_FMInit)
    sigma_FMData = np.array([sigma_FMInit[0], sigma_FMInit[1], sigma_FMInit[2]])
    omega_FM_FData = np.array([0.0, 0.0, 0.0])
    omegaPrime_FM_FData = np.array([0.0, 0.0, 0.0])

    # Define temporal parameters
    dt = 0.01  # [s]
    t = 0.0
    motorMaxAccel = 100 * ((0.5 * np.abs(motorStepAngle)) * 8) / 100
    stepSimTime = np.sqrt( ((0.5 * np.abs(motorStepAngle)) * 8) / motorMaxAccel )  # [s]
    n_s = int(stepSimTime / dt)
    timespan = np.array(t)

    # MOVE MOTOR 2
    intermediateInitialAngle = motor2InitAngle
    motor2FinalAngle = motor2InitAngle
    for idx1 in range(numSteps1):
        motor2FinalAngle = motor2InitAngle + ((idx1 + 1) * motorStepAngle)
        tInit = t

        # Find switch time and final time
        ts = tInit + (0.5 * stepSimTime)
        tf = tInit + stepSimTime

        # Find parabolic constants
        a = (0.5 * motorStepAngle) / ((ts - tInit) * (ts - tInit))
        b = (-0.5 * motorStepAngle) / ((ts - tf) * (ts - tf))

        # Calculate the current motor angles
        motor1Angle = motor1InitAngle  # [CONSTANT]
        motor1AngleRate = 0.0  # [CONSTANT]
        motor1AngularAccel = 0.0  # [CONSTANT]

        # Find gimbal initial angles
        # Interpolate to find gimbal angles
        gimbal1InitAngle, gimbal2InitAngle = motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, motor1Angle, intermediateInitialAngle)

        # Find initial PRV
        dcm1 = np.array([[1,  0, 0],
                         [0, np.cos(deg2Rad * gimbal1InitAngle), np.sin(deg2Rad * gimbal1InitAngle)],
                         [0, -np.sin(deg2Rad * gimbal1InitAngle), np.cos(deg2Rad * gimbal1InitAngle)]])
        dcm2 = np.array([[np.cos(deg2Rad * gimbal2InitAngle),  0, -np.sin(deg2Rad * gimbal2InitAngle)],
                         [0, 1, 0],
                         [np.sin(deg2Rad * gimbal2InitAngle),  0, np.cos(deg2Rad * gimbal2InitAngle)]])

        if oneTwo:
            dcm_FM = np.matmul(dcm2, dcm1)
        else:
            dcm_FM = np.matmul(dcm1, dcm2)

        PRV_FM1 = rbk.C2PRV(dcm_FM)

        # Find final PRV
        # Interpolate to find gimbal angles
        gimbal1FinalAngle, gimbal2FinalAngle = motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, motor1Angle, motor2FinalAngle)

        dcm1 = np.array([[1,  0, 0],
                         [0, np.cos(deg2Rad * gimbal1FinalAngle), np.sin(deg2Rad * gimbal1FinalAngle)],
                         [0, -np.sin(deg2Rad * gimbal1FinalAngle), np.cos(deg2Rad * gimbal1FinalAngle)]])
        dcm2 = np.array([[np.cos(deg2Rad * gimbal2FinalAngle),  0, -np.sin(deg2Rad * gimbal2FinalAngle)],
                         [0, 1, 0],
                         [np.sin(deg2Rad * gimbal2FinalAngle),  0, np.cos(deg2Rad * gimbal2FinalAngle)]])

        if oneTwo:
            dcm_FM = np.matmul(dcm2, dcm1)
        else:
            dcm_FM = np.matmul(dcm1, dcm2)

        PRV_FM2 = rbk.C2PRV(dcm_FM)

        # Find relative PRV during the step
        relativePRV = rbk.subPRV(PRV_FM2, PRV_FM1)
        phiRef = np.linalg.norm(relativePRV)
        eHat = relativePRV / phiRef

        gimbalMaxAccel = phiRef / ((0.5 * stepSimTime) * (0.5 * stepSimTime))

        # Find gimbal parabolic constants
        c = (0.5 * phiRef) / ((ts - tInit) * (ts - tInit))
        d = (-0.5 * phiRef) / ((ts - tf) * (ts - tf))

        for idx2 in range(n_s):
            # Update current time
            t = t + dt

            if ((t < ts or t == ts) and tf - tInit != 0):
                motor2AngularAccel = motorMaxAccel
                motor2AngleRate = motor2AngularAccel * (t - tInit) + thetaDotInit
                motor2Angle = a * (t - tInit) * (t - tInit) + intermediateInitialAngle

                phiDDot = gimbalMaxAccel
                phiDot = phiDDot * (t - tInit)
                phi = c * (t - tInit) * (t - tInit)

            elif ( t > ts and t <= tf and tf - tInit != 0):
                motor2AngularAccel = -1 * motorMaxAccel
                motor2AngleRate = motor2AngularAccel * (t - tInit) + thetaDotInit - motor2AngularAccel * (tf - tInit)
                motor2Angle = b * (t - tf) * (t - tf) + motor2FinalAngle

                phiDDot = -1 * gimbalMaxAccel
                phiDot = phiDDot * (t - tInit) - phiDDot * (tf - tInit)
                phi = d * (t - tf) * (t - tf) + phiRef
            else:
                motor2AngularAccel = 0.0
                motor2AngleRate = thetaDotRef
                motor2Angle = motor2FinalAngle

                phiDDot = 0.0
                phiDot = 0.0
                phi = phiRef

            # Store time current motor and gimbal states into the data arrays
            timespan = np.append(timespan, t)
            motor1AngleData = np.append(motor1AngleData, motor1Angle)
            motor1AngleRateData = np.append(motor1AngleRateData, 0.0)
            motor1AngularAccelData = np.append(motor1AngularAccelData, 0.0)
            motor2AngleData = np.append(motor2AngleData, motor2Angle)
            motor2AngleRateData = np.append(motor2AngleRateData, motor2AngleRate)
            motor2AngularAccelData = np.append(motor2AngularAccelData, motor2AngularAccel)

            relativePRV = phi * eHat
            PRV_FMCurrent = rbk.addPRV(PRV_FM1, relativePRV)

            dcm_FM = rbk.PRV2C(PRV_FMCurrent)
            thrustAx = dcm_FM[2, :]
            thrustAxis = np.vstack((thrustAxis, thrustAx))

            gimbal1Angle = np.arctan(dcm_FM[1, 2] / dcm_FM[1, 1]) / deg2Rad
            gimbal2Angle = np.arctan(dcm_FM[2, 0] / dcm_FM[0, 0]) / deg2Rad
            gimbal1AngleData = np.append(gimbal1AngleData, gimbal1Angle)
            gimbal2AngleData = np.append(gimbal2AngleData, gimbal2Angle)

            sigma_FM = rbk.PRV2MRP(PRV_FMCurrent)
            sigma_FMData = np.vstack((sigma_FMData, np.array(sigma_FM)))

            omega_FM_F = phiDot * eHat
            omega_FM_FData = np.vstack((omega_FM_FData, omega_FM_F))

            omegaPrime_FM_F = phiDDot * eHat
            omegaPrime_FM_FData = np.vstack((omegaPrime_FM_FData, omegaPrime_FM_F))

        # Update motor states after a step has been completed
        intermediateInitialAngle = motor2FinalAngle

    # MOVE MOTOR 1
    motor1 = True
    motor2 = False
    intermediateInitialAngle = motor1InitAngle
    for idx3 in range(numSteps2):
        motor1FinalAngle = motor1InitAngle + ((idx3 + 1) * motorStepAngle)
        tInit = t

        # Find switch time and final time
        ts = tInit + (0.5 * stepSimTime)
        tf = tInit + stepSimTime

        # Find parabolic constants
        a = (0.5 * motorStepAngle) / ((ts - tInit) * (ts - tInit))
        b = (-0.5 * motorStepAngle) / ((ts - tf) * (ts - tf))

        # Calculate the current motor angles
        motor2Angle = motor2FinalAngle  # [CONSTANT]
        motor2AngleRate = 0.0
        motor2AngularAccel = 0.0  # [CONSTANT]

        # Find gimbal initial angles
        # Interpolate to find gimbal angles
        gimbal1InitAngle, gimbal2InitAngle = motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, intermediateInitialAngle, motor2Angle)

        # Find initial PRV
        dcm1 = np.array([[1,  0, 0],
                         [0, np.cos(deg2Rad * gimbal1InitAngle), np.sin(deg2Rad * gimbal1InitAngle)],
                         [0, -np.sin(deg2Rad * gimbal1InitAngle), np.cos(deg2Rad * gimbal1InitAngle)]])
        dcm2 = np.array([[np.cos(deg2Rad * gimbal2InitAngle),  0, -np.sin(deg2Rad * gimbal2InitAngle)],
                         [0, 1, 0],
                         [np.sin(deg2Rad * gimbal2InitAngle),  0, np.cos(deg2Rad * gimbal2InitAngle)]])

        if oneTwo:
            dcm_FM = np.matmul(dcm2, dcm1)
        else:
            dcm_FM = np.matmul(dcm1, dcm2)

        PRV_FM1 = rbk.C2PRV(dcm_FM)

        # Find final PRV
        # Interpolate to find gimbal angles
        gimbal1FinalAngle, gimbal2FinalAngle = motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, motor1FinalAngle, motor2Angle)

        dcm1 = np.array([[1,  0, 0],
                         [0, np.cos(deg2Rad * gimbal1FinalAngle), np.sin(deg2Rad * gimbal1FinalAngle)],
                         [0, -np.sin(deg2Rad * gimbal1FinalAngle), np.cos(deg2Rad * gimbal1FinalAngle)]])
        dcm2 = np.array([[np.cos(deg2Rad * gimbal2FinalAngle),  0, -np.sin(deg2Rad * gimbal2FinalAngle)],
                         [0, 1, 0],
                         [np.sin(deg2Rad * gimbal2FinalAngle),  0, np.cos(deg2Rad * gimbal2FinalAngle)]])

        if oneTwo:
            dcm_FM = np.matmul(dcm2, dcm1)
        else:
            dcm_FM = np.matmul(dcm1, dcm2)

        PRV_FM2 = rbk.C2PRV(dcm_FM)

        # Find relative PRV during the step
        relativePRV = rbk.subPRV(PRV_FM2, PRV_FM1)
        phiRef = np.linalg.norm(relativePRV)
        eHat = relativePRV / phiRef

        gimbalMaxAccel = phiRef / ((0.5 * stepSimTime) * (0.5 * stepSimTime))

        # Find gimbal parabolic constants
        c = (0.5 * phiRef) / ((ts - tInit) * (ts - tInit))
        d = (-0.5 * phiRef) / ((ts - tf) * (ts - tf))

        for idx4 in range(n_s):
            # Update current time
            t = t + dt

            if ((t < ts or t == ts) and tf - tInit != 0):
                motor1AngularAccel = motorMaxAccel
                motor1AngleRate = motor1AngularAccel * (t - tInit) + thetaDotInit
                motor1Angle = a * (t - tInit) * (t - tInit) + intermediateInitialAngle

                phiDDot = gimbalMaxAccel
                phiDot = phiDDot * (t - tInit)
                phi = c * (t - tInit) * (t - tInit)

            elif ( t > ts and t <= tf and tf - tInit != 0):
                motor1AngularAccel = -1 * motorMaxAccel
                motor1AngleRate = motor1AngularAccel * (t - tInit) + thetaDotInit - motor1AngularAccel * (tf - tInit)
                motor1Angle = b * (t - tf) * (t - tf) + motor1FinalAngle

                phiDDot = -1 * gimbalMaxAccel
                phiDot = phiDDot * (t - tInit) - phiDDot * (tf - tInit)
                phi = d * (t - tf) * (t - tf) + phiRef
            else:
                motor1AngularAccel = 0.0
                motor1AngleRate = thetaDotRef
                motor1Angle = motor1FinalAngle

                phiDDot = 0.0
                phiDot = 0.0
                phi = phiRef

            # Store time current motor and gimbal states into the data arrays
            timespan = np.append(timespan, t)
            motor2AngleData = np.append(motor2AngleData, motor2Angle)
            motor2AngleRateData = np.append(motor2AngleRateData, 0.0)
            motor2AngularAccelData = np.append(motor2AngularAccelData, 0.0)
            motor1AngleData = np.append(motor1AngleData, motor1Angle)
            motor1AngleRateData = np.append(motor1AngleRateData, motor1AngleRate)
            motor1AngularAccelData = np.append(motor1AngularAccelData, motor1AngularAccel)

            relativePRV = phi * eHat
            PRV_FMCurrent = rbk.addPRV(PRV_FM1, relativePRV)

            dcm_FM = rbk.PRV2C(PRV_FMCurrent)
            thrustAx = dcm_FM[2, :]
            thrustAxis = np.vstack((thrustAxis, thrustAx))

            gimbal1Angle = np.arctan(dcm_FM[1, 2] / dcm_FM[1, 1]) / deg2Rad
            gimbal2Angle = np.arctan(dcm_FM[2, 0] / dcm_FM[0, 0]) / deg2Rad
            gimbal1AngleData = np.append(gimbal1AngleData, gimbal1Angle)
            gimbal2AngleData = np.append(gimbal2AngleData, gimbal2Angle)

            sigma_FM = rbk.PRV2MRP(PRV_FMCurrent)
            sigma_FMData = np.vstack((sigma_FMData, np.array(sigma_FM)))

            omega_FM_F = phiDot * eHat
            omega_FM_FData = np.vstack((omega_FM_FData, omega_FM_F))

            omegaPrime_FM_F = phiDDot * eHat
            omegaPrime_FM_FData = np.vstack((omegaPrime_FM_FData, omegaPrime_FM_F))

        # Update motor states after a step has been completed
        intermediateInitialAngle = motor1FinalAngle

    # SEQUENCE 2
    motor1InitAngle = motor1FinalAngle
    motor2InitAngle = motor2Angle

    # MOVE MOTOR 2
    motor1 = False
    motor2 = True
    intermediateInitialAngle = motor2Angle
    motor2FinalAngle = motor2Angle
    for idx1 in range(numSteps1):
        motor2FinalAngle = motor2InitAngle + ((idx1 + 1) * motorStepAngle)
        tInit = t

        # Find switch time and final time
        ts = tInit + (0.5 * stepSimTime)
        tf = tInit + stepSimTime

        # Find parabolic constants
        a = (0.5 * motorStepAngle) / ((ts - tInit) * (ts - tInit))
        b = (-0.5 * motorStepAngle) / ((ts - tf) * (ts - tf))

        # Calculate the current motor angles
        motor1Angle = motor1InitAngle  # [CONSTANT]
        motor1AngleRate = 0.0  # [CONSTANT]
        motor1AngularAccel = 0.0  # [CONSTANT]

        # Find gimbal initial angles
        # Interpolate to find gimbal angles
        gimbal1InitAngle, gimbal2InitAngle = motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, motor1Angle, intermediateInitialAngle)

        # Find initial PRV
        dcm1 = np.array([[1,  0, 0],
                         [0, np.cos(deg2Rad * gimbal1InitAngle), np.sin(deg2Rad * gimbal1InitAngle)],
                         [0, -np.sin(deg2Rad * gimbal1InitAngle), np.cos(deg2Rad * gimbal1InitAngle)]])
        dcm2 = np.array([[np.cos(deg2Rad * gimbal2InitAngle),  0, -np.sin(deg2Rad * gimbal2InitAngle)],
                         [0, 1, 0],
                         [np.sin(deg2Rad * gimbal2InitAngle),  0, np.cos(deg2Rad * gimbal2InitAngle)]])

        if oneTwo:
            dcm_FM = np.matmul(dcm2, dcm1)
        else:
            dcm_FM = np.matmul(dcm1, dcm2)

        PRV_FM1 = rbk.C2PRV(dcm_FM)

        # Find final PRV
        # Interpolate to find gimbal angles
        gimbal1FinalAngle, gimbal2FinalAngle = motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, motor1Angle, motor2FinalAngle)

        dcm1 = np.array([[1,  0, 0],
                         [0, np.cos(deg2Rad * gimbal1FinalAngle), np.sin(deg2Rad * gimbal1FinalAngle)],
                         [0, -np.sin(deg2Rad * gimbal1FinalAngle), np.cos(deg2Rad * gimbal1FinalAngle)]])
        dcm2 = np.array([[np.cos(deg2Rad * gimbal2FinalAngle),  0, -np.sin(deg2Rad * gimbal2FinalAngle)],
                         [0, 1, 0],
                         [np.sin(deg2Rad * gimbal2FinalAngle),  0, np.cos(deg2Rad * gimbal2FinalAngle)]])

        if oneTwo:
            dcm_FM = np.matmul(dcm2, dcm1)
        else:
            dcm_FM = np.matmul(dcm1, dcm2)

        PRV_FM2 = rbk.C2PRV(dcm_FM)

        # Find relative PRV during the step
        relativePRV = rbk.subPRV(PRV_FM2, PRV_FM1)
        phiRef = np.linalg.norm(relativePRV)
        eHat = relativePRV / phiRef

        gimbalMaxAccel = phiRef / ((0.5 * stepSimTime) * (0.5 * stepSimTime))

        # Find gimbal parabolic constants
        c = (0.5 * phiRef) / ((ts - tInit) * (ts - tInit))
        d = (-0.5 * phiRef) / ((ts - tf) * (ts - tf))

        for idx2 in range(n_s):
            # Update current time
            t = t + dt

            if ((t < ts or t == ts) and tf - tInit != 0):
                motor2AngularAccel = motorMaxAccel
                motor2AngleRate = motor2AngularAccel * (t - tInit) + thetaDotInit
                motor2Angle = a * (t - tInit) * (t - tInit) + intermediateInitialAngle

                phiDDot = gimbalMaxAccel
                phiDot = phiDDot * (t - tInit)
                phi = c * (t - tInit) * (t - tInit)

            elif ( t > ts and t <= tf and tf - tInit != 0):
                motor2AngularAccel = -1 * motorMaxAccel
                motor2AngleRate = motor2AngularAccel * (t - tInit) + thetaDotInit - motor2AngularAccel * (tf - tInit)
                motor2Angle = b * (t - tf) * (t - tf) + motor2FinalAngle

                phiDDot = -1 * gimbalMaxAccel
                phiDot = phiDDot * (t - tInit) - phiDDot * (tf - tInit)
                phi = d * (t - tf) * (t - tf) + phiRef
            else:
                motor2AngularAccel = 0.0
                motor2AngleRate = thetaDotRef
                motor2Angle = motor2FinalAngle

                phiDDot = 0.0
                phiDot = 0.0
                phi = phiRef

            # Store time current motor and gimbal states into the data arrays
            timespan = np.append(timespan, t)
            motor1AngleData = np.append(motor1AngleData, motor1Angle)
            motor1AngleRateData = np.append(motor1AngleRateData, 0.0)
            motor1AngularAccelData = np.append(motor1AngularAccelData, 0.0)
            motor2AngleData = np.append(motor2AngleData, motor2Angle)
            motor2AngleRateData = np.append(motor2AngleRateData, motor2AngleRate)
            motor2AngularAccelData = np.append(motor2AngularAccelData, motor2AngularAccel)

            relativePRV = phi * eHat
            PRV_FMCurrent = rbk.addPRV(PRV_FM1, relativePRV)

            dcm_FM = rbk.PRV2C(PRV_FMCurrent)
            thrustAx = dcm_FM[2, :]
            thrustAxis = np.vstack((thrustAxis, thrustAx))

            gimbal1Angle = np.arctan(dcm_FM[1, 2] / dcm_FM[1, 1]) / deg2Rad
            gimbal2Angle = np.arctan(dcm_FM[2, 0] / dcm_FM[0, 0]) / deg2Rad
            gimbal1AngleData = np.append(gimbal1AngleData, gimbal1Angle)
            gimbal2AngleData = np.append(gimbal2AngleData, gimbal2Angle)

            sigma_FM = rbk.PRV2MRP(PRV_FMCurrent)
            sigma_FMData = np.vstack((sigma_FMData, np.array(sigma_FM)))

            omega_FM_F = phiDot * eHat
            omega_FM_FData = np.vstack((omega_FM_FData, omega_FM_F))

            omegaPrime_FM_F = phiDDot * eHat
            omegaPrime_FM_FData = np.vstack((omegaPrime_FM_FData, omegaPrime_FM_F))

        # Update motor states after a step has been completed
        intermediateInitialAngle = motor2FinalAngle

    # MOVE MOTOR 1
    motor1 = True
    motor2 = False
    intermediateInitialAngle = motor1InitAngle
    for idx3 in range(numSteps2):
        motor1FinalAngle = motor1InitAngle + ((idx3 + 1) * motorStepAngle)
        tInit = t

        # Find switch time and final time
        ts = tInit + (0.5 * stepSimTime)
        tf = tInit + stepSimTime

        # Find parabolic constants
        a = (0.5 * motorStepAngle) / ((ts - tInit) * (ts - tInit))
        b = (-0.5 * motorStepAngle) / ((ts - tf) * (ts - tf))

        # Calculate the current motor angles
        motor2Angle = motor2FinalAngle  # [CONSTANT]
        motor2AngleRate = 0.0
        motor2AngularAccel = 0.0  # [CONSTANT]

        # Find gimbal initial angles
        # Interpolate to find gimbal angles
        gimbal1InitAngle, gimbal2InitAngle = motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, intermediateInitialAngle, motor2Angle)

        # Find initial PRV
        dcm1 = np.array([[1,  0, 0],
                         [0, np.cos(deg2Rad * gimbal1InitAngle), np.sin(deg2Rad * gimbal1InitAngle)],
                         [0, -np.sin(deg2Rad * gimbal1InitAngle), np.cos(deg2Rad * gimbal1InitAngle)]])
        dcm2 = np.array([[np.cos(deg2Rad * gimbal2InitAngle),  0, -np.sin(deg2Rad * gimbal2InitAngle)],
                         [0, 1, 0],
                         [np.sin(deg2Rad * gimbal2InitAngle),  0, np.cos(deg2Rad * gimbal2InitAngle)]])

        if oneTwo:
            dcm_FM = np.matmul(dcm2, dcm1)
        else:
            dcm_FM = np.matmul(dcm1, dcm2)

        PRV_FM1 = rbk.C2PRV(dcm_FM)

        # Find final PRV
        # Interpolate to find gimbal angles
        gimbal1FinalAngle, gimbal2FinalAngle = motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, motor1FinalAngle, motor2Angle)

        dcm1 = np.array([[1,  0, 0],
                         [0, np.cos(deg2Rad * gimbal1FinalAngle), np.sin(deg2Rad * gimbal1FinalAngle)],
                         [0, -np.sin(deg2Rad * gimbal1FinalAngle), np.cos(deg2Rad * gimbal1FinalAngle)]])
        dcm2 = np.array([[np.cos(deg2Rad * gimbal2FinalAngle),  0, -np.sin(deg2Rad * gimbal2FinalAngle)],
                         [0, 1, 0],
                         [np.sin(deg2Rad * gimbal2FinalAngle),  0, np.cos(deg2Rad * gimbal2FinalAngle)]])

        if oneTwo:
            dcm_FM = np.matmul(dcm2, dcm1)
        else:
            dcm_FM = np.matmul(dcm1, dcm2)

        PRV_FM2 = rbk.C2PRV(dcm_FM)

        # Find relative PRV during the step
        relativePRV = rbk.subPRV(PRV_FM2, PRV_FM1)
        phiRef = np.linalg.norm(relativePRV)
        eHat = relativePRV / phiRef

        gimbalMaxAccel = phiRef / ((0.5 * stepSimTime) * (0.5 * stepSimTime))

        # Find gimbal parabolic constants
        c = (0.5 * phiRef) / ((ts - tInit) * (ts - tInit))
        d = (-0.5 * phiRef) / ((ts - tf) * (ts - tf))

        for idx4 in range(n_s):
            # Update current time
            t = t + dt

            if ((t < ts or t == ts) and tf - tInit != 0):
                motor1AngularAccel = motorMaxAccel
                motor1AngleRate = motor1AngularAccel * (t - tInit) + thetaDotInit
                motor1Angle = a * (t - tInit) * (t - tInit) + intermediateInitialAngle

                phiDDot = gimbalMaxAccel
                phiDot = phiDDot * (t - tInit)
                phi = c * (t - tInit) * (t - tInit)

            elif ( t > ts and t <= tf and tf - tInit != 0):
                motor1AngularAccel = -1 * motorMaxAccel
                motor1AngleRate = motor1AngularAccel * (t - tInit) + thetaDotInit - motor1AngularAccel * (tf - tInit)
                motor1Angle = b * (t - tf) * (t - tf) + motor1FinalAngle

                phiDDot = -1 * gimbalMaxAccel
                phiDot = phiDDot * (t - tInit) - phiDDot * (tf - tInit)
                phi = d * (t - tf) * (t - tf) + phiRef
            else:
                motor1AngularAccel = 0.0
                motor1AngleRate = thetaDotRef
                motor1Angle = motor1FinalAngle

                phiDDot = 0.0
                phiDot = 0.0
                phi = phiRef

            # Store time current motor and gimbal states into the data arrays
            timespan = np.append(timespan, t)
            motor2AngleData = np.append(motor2AngleData, motor2Angle)
            motor2AngleRateData = np.append(motor2AngleRateData, 0.0)
            motor2AngularAccelData = np.append(motor2AngularAccelData, 0.0)
            motor1AngleData = np.append(motor1AngleData, motor1Angle)
            motor1AngleRateData = np.append(motor1AngleRateData, motor1AngleRate)
            motor1AngularAccelData = np.append(motor1AngularAccelData, motor1AngularAccel)

            relativePRV = phi * eHat
            PRV_FMCurrent = rbk.addPRV(PRV_FM1, relativePRV)

            dcm_FM = rbk.PRV2C(PRV_FMCurrent)
            thrustAx = dcm_FM[2, :]
            thrustAxis = np.vstack((thrustAxis, thrustAx))

            gimbal1Angle = np.arctan(dcm_FM[1, 2] / dcm_FM[1, 1]) / deg2Rad
            gimbal2Angle = np.arctan(dcm_FM[2, 0] / dcm_FM[0, 0]) / deg2Rad
            gimbal1AngleData = np.append(gimbal1AngleData, gimbal1Angle)
            gimbal2AngleData = np.append(gimbal2AngleData, gimbal2Angle)

            sigma_FM = rbk.PRV2MRP(PRV_FMCurrent)
            sigma_FMData = np.vstack((sigma_FMData, np.array(sigma_FM)))

            omega_FM_F = phiDot * eHat
            omega_FM_FData = np.vstack((omega_FM_FData, omega_FM_F))

            omegaPrime_FM_F = phiDDot * eHat
            omegaPrime_FM_FData = np.vstack((omegaPrime_FM_FData, omegaPrime_FM_F))

        # Update motor states after a step has been completed
        intermediateInitialAngle = motor1FinalAngle

    # SEQUENCE 3
    motor1InitAngle = motor1FinalAngle
    motor2InitAngle = motor2Angle

    # MOVE MOTOR 2
    motor1 = False
    motor2 = True
    intermediateInitialAngle = motor2Angle
    motor2FinalAngle = motor2Angle
    for idx1 in range(numSteps1):
        motor2FinalAngle = motor2InitAngle + ((idx1 + 1) * motorStepAngle)
        tInit = t

        # Find switch time and final time
        ts = tInit + (0.5 * stepSimTime)
        tf = tInit + stepSimTime

        # Find parabolic constants
        a = (0.5 * motorStepAngle) / ((ts - tInit) * (ts - tInit))
        b = (-0.5 * motorStepAngle) / ((ts - tf) * (ts - tf))

        # Calculate the current motor angles
        motor1Angle = motor1InitAngle  # [CONSTANT]
        motor1AngleRate = 0.0  # [CONSTANT]
        motor1AngularAccel = 0.0  # [CONSTANT]

        # Find gimbal initial angles
        # Interpolate to find gimbal angles
        gimbal1InitAngle, gimbal2InitAngle = motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, motor1Angle, intermediateInitialAngle)

        # Find initial PRV
        dcm1 = np.array([[1,  0, 0],
                         [0, np.cos(deg2Rad * gimbal1InitAngle), np.sin(deg2Rad * gimbal1InitAngle)],
                         [0, -np.sin(deg2Rad * gimbal1InitAngle), np.cos(deg2Rad * gimbal1InitAngle)]])
        dcm2 = np.array([[np.cos(deg2Rad * gimbal2InitAngle),  0, -np.sin(deg2Rad * gimbal2InitAngle)],
                         [0, 1, 0],
                         [np.sin(deg2Rad * gimbal2InitAngle),  0, np.cos(deg2Rad * gimbal2InitAngle)]])

        if oneTwo:
            dcm_FM = np.matmul(dcm2, dcm1)
        else:
            dcm_FM = np.matmul(dcm1, dcm2)

        PRV_FM1 = rbk.C2PRV(dcm_FM)

        # Find final PRV
        # Interpolate to find gimbal angles
        gimbal1FinalAngle, gimbal2FinalAngle = motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, motor1Angle, motor2FinalAngle)

        dcm1 = np.array([[1,  0, 0],
                         [0, np.cos(deg2Rad * gimbal1FinalAngle), np.sin(deg2Rad * gimbal1FinalAngle)],
                         [0, -np.sin(deg2Rad * gimbal1FinalAngle), np.cos(deg2Rad * gimbal1FinalAngle)]])
        dcm2 = np.array([[np.cos(deg2Rad * gimbal2FinalAngle),  0, -np.sin(deg2Rad * gimbal2FinalAngle)],
                         [0, 1, 0],
                         [np.sin(deg2Rad * gimbal2FinalAngle),  0, np.cos(deg2Rad * gimbal2FinalAngle)]])

        if oneTwo:
            dcm_FM = np.matmul(dcm2, dcm1)
        else:
            dcm_FM = np.matmul(dcm1, dcm2)

        PRV_FM2 = rbk.C2PRV(dcm_FM)

        # Find relative PRV during the step
        relativePRV = rbk.subPRV(PRV_FM2, PRV_FM1)
        phiRef = np.linalg.norm(relativePRV)
        eHat = relativePRV / phiRef

        gimbalMaxAccel = phiRef / ((0.5 * stepSimTime) * (0.5 * stepSimTime))

        # Find gimbal parabolic constants
        c = (0.5 * phiRef) / ((ts - tInit) * (ts - tInit))
        d = (-0.5 * phiRef) / ((ts - tf) * (ts - tf))

        for idx2 in range(n_s):
            # Update current time
            t = t + dt

            if ((t < ts or t == ts) and tf - tInit != 0):
                motor2AngularAccel = motorMaxAccel
                motor2AngleRate = motor2AngularAccel * (t - tInit) + thetaDotInit
                motor2Angle = a * (t - tInit) * (t - tInit) + intermediateInitialAngle

                phiDDot = gimbalMaxAccel
                phiDot = phiDDot * (t - tInit)
                phi = c * (t - tInit) * (t - tInit)

            elif ( t > ts and t <= tf and tf - tInit != 0):
                motor2AngularAccel = -1 * motorMaxAccel
                motor2AngleRate = motor2AngularAccel * (t - tInit) + thetaDotInit - motor2AngularAccel * (tf - tInit)
                motor2Angle = b * (t - tf) * (t - tf) + motor2FinalAngle

                phiDDot = -1 * gimbalMaxAccel
                phiDot = phiDDot * (t - tInit) - phiDDot * (tf - tInit)
                phi = d * (t - tf) * (t - tf) + phiRef
            else:
                motor2AngularAccel = 0.0
                motor2AngleRate = thetaDotRef
                motor2Angle = motor2FinalAngle

                phiDDot = 0.0
                phiDot = 0.0
                phi = phiRef

            # Store time current motor and gimbal states into the data arrays
            timespan = np.append(timespan, t)
            motor1AngleData = np.append(motor1AngleData, motor1Angle)
            motor1AngleRateData = np.append(motor1AngleRateData, 0.0)
            motor1AngularAccelData = np.append(motor1AngularAccelData, 0.0)
            motor2AngleData = np.append(motor2AngleData, motor2Angle)
            motor2AngleRateData = np.append(motor2AngleRateData, motor2AngleRate)
            motor2AngularAccelData = np.append(motor2AngularAccelData, motor2AngularAccel)

            relativePRV = phi * eHat
            PRV_FMCurrent = rbk.addPRV(PRV_FM1, relativePRV)

            dcm_FM = rbk.PRV2C(PRV_FMCurrent)
            thrustAx = dcm_FM[2, :]
            thrustAxis = np.vstack((thrustAxis, thrustAx))

            gimbal1Angle = np.arctan(dcm_FM[1, 2] / dcm_FM[1, 1]) / deg2Rad
            gimbal2Angle = np.arctan(dcm_FM[2, 0] / dcm_FM[0, 0]) / deg2Rad
            gimbal1AngleData = np.append(gimbal1AngleData, gimbal1Angle)
            gimbal2AngleData = np.append(gimbal2AngleData, gimbal2Angle)

            sigma_FM = rbk.PRV2MRP(PRV_FMCurrent)
            sigma_FMData = np.vstack((sigma_FMData, np.array(sigma_FM)))

            omega_FM_F = phiDot * eHat
            omega_FM_FData = np.vstack((omega_FM_FData, omega_FM_F))

            omegaPrime_FM_F = phiDDot * eHat
            omegaPrime_FM_FData = np.vstack((omegaPrime_FM_FData, omegaPrime_FM_F))

        # Update motor states after a step has been completed
        intermediateInitialAngle = motor2FinalAngle

    # MOVE MOTOR 1
    motor1 = True
    motor2 = False
    intermediateInitialAngle = motor1InitAngle
    for idx3 in range(numSteps2):
        motor1FinalAngle = motor1InitAngle + ((idx3 + 1) * motorStepAngle)
        tInit = t

        # Find switch time and final time
        ts = tInit + (0.5 * stepSimTime)
        tf = tInit + stepSimTime

        # Find parabolic constants
        a = (0.5 * motorStepAngle) / ((ts - tInit) * (ts - tInit))
        b = (-0.5 * motorStepAngle) / ((ts - tf) * (ts - tf))

        # Calculate the current motor angles
        motor2Angle = motor2FinalAngle  # [CONSTANT]
        motor2AngleRate = 0.0
        motor2AngularAccel = 0.0  # [CONSTANT]

        # Find gimbal initial angles
        # Interpolate to find gimbal angles
        gimbal1InitAngle, gimbal2InitAngle = motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, intermediateInitialAngle, motor2Angle)

        # Find initial PRV
        dcm1 = np.array([[1,  0, 0],
                         [0, np.cos(deg2Rad * gimbal1InitAngle), np.sin(deg2Rad * gimbal1InitAngle)],
                         [0, -np.sin(deg2Rad * gimbal1InitAngle), np.cos(deg2Rad * gimbal1InitAngle)]])
        dcm2 = np.array([[np.cos(deg2Rad * gimbal2InitAngle),  0, -np.sin(deg2Rad * gimbal2InitAngle)],
                         [0, 1, 0],
                         [np.sin(deg2Rad * gimbal2InitAngle),  0, np.cos(deg2Rad * gimbal2InitAngle)]])

        if oneTwo:
            dcm_FM = np.matmul(dcm2, dcm1)
        else:
            dcm_FM = np.matmul(dcm1, dcm2)

        PRV_FM1 = rbk.C2PRV(dcm_FM)

        # Find final PRV
        # Interpolate to find gimbal angles
        gimbal1FinalAngle, gimbal2FinalAngle = motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, motor1FinalAngle, motor2Angle)

        dcm1 = np.array([[1,  0, 0],
                         [0, np.cos(deg2Rad * gimbal1FinalAngle), np.sin(deg2Rad * gimbal1FinalAngle)],
                         [0, -np.sin(deg2Rad * gimbal1FinalAngle), np.cos(deg2Rad * gimbal1FinalAngle)]])
        dcm2 = np.array([[np.cos(deg2Rad * gimbal2FinalAngle),  0, -np.sin(deg2Rad * gimbal2FinalAngle)],
                         [0, 1, 0],
                         [np.sin(deg2Rad * gimbal2FinalAngle),  0, np.cos(deg2Rad * gimbal2FinalAngle)]])

        if oneTwo:
            dcm_FM = np.matmul(dcm2, dcm1)
        else:
            dcm_FM = np.matmul(dcm1, dcm2)

        PRV_FM2 = rbk.C2PRV(dcm_FM)

        # Find relative PRV during the step
        relativePRV = rbk.subPRV(PRV_FM2, PRV_FM1)
        phiRef = np.linalg.norm(relativePRV)
        eHat = relativePRV / phiRef

        gimbalMaxAccel = phiRef / ((0.5 * stepSimTime) * (0.5 * stepSimTime))

        # Find gimbal parabolic constants
        c = (0.5 * phiRef) / ((ts - tInit) * (ts - tInit))
        d = (-0.5 * phiRef) / ((ts - tf) * (ts - tf))

        for idx4 in range(n_s):
            # Update current time
            t = t + dt

            if ((t < ts or t == ts) and tf - tInit != 0):
                motor1AngularAccel = motorMaxAccel
                motor1AngleRate = motor1AngularAccel * (t - tInit) + thetaDotInit
                motor1Angle = a * (t - tInit) * (t - tInit) + intermediateInitialAngle

                phiDDot = gimbalMaxAccel
                phiDot = phiDDot * (t - tInit)
                phi = c * (t - tInit) * (t - tInit)

            elif ( t > ts and t <= tf and tf - tInit != 0):
                motor1AngularAccel = -1 * motorMaxAccel
                motor1AngleRate = motor1AngularAccel * (t - tInit) + thetaDotInit - motor1AngularAccel * (tf - tInit)
                motor1Angle = b * (t - tf) * (t - tf) + motor1FinalAngle

                phiDDot = -1 * gimbalMaxAccel
                phiDot = phiDDot * (t - tInit) - phiDDot * (tf - tInit)
                phi = d * (t - tf) * (t - tf) + phiRef
            else:
                motor1AngularAccel = 0.0
                motor1AngleRate = thetaDotRef
                motor1Angle = motor1FinalAngle

                phiDDot = 0.0
                phiDot = 0.0
                phi = phiRef

            # Store time current motor and gimbal states into the data arrays
            timespan = np.append(timespan, t)
            motor2AngleData = np.append(motor2AngleData, motor2Angle)
            motor2AngleRateData = np.append(motor2AngleRateData, 0.0)
            motor2AngularAccelData = np.append(motor2AngularAccelData, 0.0)
            motor1AngleData = np.append(motor1AngleData, motor1Angle)
            motor1AngleRateData = np.append(motor1AngleRateData, motor1AngleRate)
            motor1AngularAccelData = np.append(motor1AngularAccelData, motor1AngularAccel)

            relativePRV = phi * eHat
            PRV_FMCurrent = rbk.addPRV(PRV_FM1, relativePRV)

            dcm_FM = rbk.PRV2C(PRV_FMCurrent)
            thrustAx = dcm_FM[2, :]
            thrustAxis = np.vstack((thrustAxis, thrustAx))

            gimbal1Angle = np.arctan(dcm_FM[1, 2] / dcm_FM[1, 1]) / deg2Rad
            gimbal2Angle = np.arctan(dcm_FM[2, 0] / dcm_FM[0, 0]) / deg2Rad
            gimbal1AngleData = np.append(gimbal1AngleData, gimbal1Angle)
            gimbal2AngleData = np.append(gimbal2AngleData, gimbal2Angle)

            sigma_FM = rbk.PRV2MRP(PRV_FMCurrent)
            sigma_FMData = np.vstack((sigma_FMData, np.array(sigma_FM)))

            omega_FM_F = phiDot * eHat
            omega_FM_FData = np.vstack((omega_FM_FData, omega_FM_F))

            omegaPrime_FM_F = phiDDot * eHat
            omegaPrime_FM_FData = np.vstack((omegaPrime_FM_FData, omegaPrime_FM_F))

        # Update motor states after a step has been completed
        intermediateInitialAngle = motor1FinalAngle

    # SEQUENCE 4
    motor1InitAngle = motor1FinalAngle
    motor2InitAngle = motor2Angle

    # MOVE MOTOR 2
    motor1 = False
    motor2 = True
    intermediateInitialAngle = motor2Angle
    motor2FinalAngle = motor2Angle
    for idx1 in range(numSteps1):
        motor2FinalAngle = motor2InitAngle + ((idx1 + 1) * motorStepAngle)
        tInit = t

        # Find switch time and final time
        ts = tInit + (0.5 * stepSimTime)
        tf = tInit + stepSimTime

        # Find parabolic constants
        a = (0.5 * motorStepAngle) / ((ts - tInit) * (ts - tInit))
        b = (-0.5 * motorStepAngle) / ((ts - tf) * (ts - tf))

        # Calculate the current motor angles
        motor1Angle = motor1InitAngle  # [CONSTANT]
        motor1AngleRate = 0.0  # [CONSTANT]
        motor1AngularAccel = 0.0  # [CONSTANT]

        # Find gimbal initial angles
        # Interpolate to find gimbal angles
        gimbal1InitAngle, gimbal2InitAngle = motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, motor1Angle, intermediateInitialAngle)

        # Find initial PRV
        dcm1 = np.array([[1,  0, 0],
                         [0, np.cos(deg2Rad * gimbal1InitAngle), np.sin(deg2Rad * gimbal1InitAngle)],
                         [0, -np.sin(deg2Rad * gimbal1InitAngle), np.cos(deg2Rad * gimbal1InitAngle)]])
        dcm2 = np.array([[np.cos(deg2Rad * gimbal2InitAngle),  0, -np.sin(deg2Rad * gimbal2InitAngle)],
                         [0, 1, 0],
                         [np.sin(deg2Rad * gimbal2InitAngle),  0, np.cos(deg2Rad * gimbal2InitAngle)]])

        if oneTwo:
            dcm_FM = np.matmul(dcm2, dcm1)
        else:
            dcm_FM = np.matmul(dcm1, dcm2)

        PRV_FM1 = rbk.C2PRV(dcm_FM)

        # Find final PRV
        # Interpolate to find gimbal angles
        gimbal1FinalAngle, gimbal2FinalAngle = motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, motor1Angle, motor2FinalAngle)

        dcm1 = np.array([[1,  0, 0],
                         [0, np.cos(deg2Rad * gimbal1FinalAngle), np.sin(deg2Rad * gimbal1FinalAngle)],
                         [0, -np.sin(deg2Rad * gimbal1FinalAngle), np.cos(deg2Rad * gimbal1FinalAngle)]])
        dcm2 = np.array([[np.cos(deg2Rad * gimbal2FinalAngle),  0, -np.sin(deg2Rad * gimbal2FinalAngle)],
                         [0, 1, 0],
                         [np.sin(deg2Rad * gimbal2FinalAngle),  0, np.cos(deg2Rad * gimbal2FinalAngle)]])

        if oneTwo:
            dcm_FM = np.matmul(dcm2, dcm1)
        else:
            dcm_FM = np.matmul(dcm1, dcm2)

        PRV_FM2 = rbk.C2PRV(dcm_FM)

        # Find relative PRV during the step
        relativePRV = rbk.subPRV(PRV_FM2, PRV_FM1)
        phiRef = np.linalg.norm(relativePRV)
        eHat = relativePRV / phiRef

        gimbalMaxAccel = phiRef / ((0.5 * stepSimTime) * (0.5 * stepSimTime))

        # Find gimbal parabolic constants
        c = (0.5 * phiRef) / ((ts - tInit) * (ts - tInit))
        d = (-0.5 * phiRef) / ((ts - tf) * (ts - tf))

        for idx2 in range(n_s):
            # Update current time
            t = t + dt

            if ((t < ts or t == ts) and tf - tInit != 0):
                motor2AngularAccel = motorMaxAccel
                motor2AngleRate = motor2AngularAccel * (t - tInit) + thetaDotInit
                motor2Angle = a * (t - tInit) * (t - tInit) + intermediateInitialAngle

                phiDDot = gimbalMaxAccel
                phiDot = phiDDot * (t - tInit)
                phi = c * (t - tInit) * (t - tInit)

            elif ( t > ts and t <= tf and tf - tInit != 0):
                motor2AngularAccel = -1 * motorMaxAccel
                motor2AngleRate = motor2AngularAccel * (t - tInit) + thetaDotInit - motor2AngularAccel * (tf - tInit)
                motor2Angle = b * (t - tf) * (t - tf) + motor2FinalAngle

                phiDDot = -1 * gimbalMaxAccel
                phiDot = phiDDot * (t - tInit) - phiDDot * (tf - tInit)
                phi = d * (t - tf) * (t - tf) + phiRef
            else:
                motor2AngularAccel = 0.0
                motor2AngleRate = thetaDotRef
                motor2Angle = motor2FinalAngle

                phiDDot = 0.0
                phiDot = 0.0
                phi = phiRef

            # Store time current motor and gimbal states into the data arrays
            timespan = np.append(timespan, t)
            motor1AngleData = np.append(motor1AngleData, motor1Angle)
            motor1AngleRateData = np.append(motor1AngleRateData, 0.0)
            motor1AngularAccelData = np.append(motor1AngularAccelData, 0.0)
            motor2AngleData = np.append(motor2AngleData, motor2Angle)
            motor2AngleRateData = np.append(motor2AngleRateData, motor2AngleRate)
            motor2AngularAccelData = np.append(motor2AngularAccelData, motor2AngularAccel)

            relativePRV = phi * eHat
            PRV_FMCurrent = rbk.addPRV(PRV_FM1, relativePRV)

            dcm_FM = rbk.PRV2C(PRV_FMCurrent)
            thrustAx = dcm_FM[2, :]
            thrustAxis = np.vstack((thrustAxis, thrustAx))

            gimbal1Angle = np.arctan(dcm_FM[1, 2] / dcm_FM[1, 1]) / deg2Rad
            gimbal2Angle = np.arctan(dcm_FM[2, 0] / dcm_FM[0, 0]) / deg2Rad
            gimbal1AngleData = np.append(gimbal1AngleData, gimbal1Angle)
            gimbal2AngleData = np.append(gimbal2AngleData, gimbal2Angle)

            sigma_FM = rbk.PRV2MRP(PRV_FMCurrent)
            sigma_FMData = np.vstack((sigma_FMData, np.array(sigma_FM)))

            omega_FM_F = phiDot * eHat
            omega_FM_FData = np.vstack((omega_FM_FData, omega_FM_F))

            omegaPrime_FM_F = phiDDot * eHat
            omegaPrime_FM_FData = np.vstack((omegaPrime_FM_FData, omegaPrime_FM_F))

        # Update motor states after a step has been completed
        intermediateInitialAngle = motor2FinalAngle

    # MOVE MOTOR 1
    motor1 = True
    motor2 = False
    intermediateInitialAngle = motor1InitAngle
    for idx3 in range(numSteps2):
        motor1FinalAngle = motor1InitAngle + ((idx3 + 1) * motorStepAngle)
        tInit = t

        # Find switch time and final time
        ts = tInit + (0.5 * stepSimTime)
        tf = tInit + stepSimTime

        # Find parabolic constants
        a = (0.5 * motorStepAngle) / ((ts - tInit) * (ts - tInit))
        b = (-0.5 * motorStepAngle) / ((ts - tf) * (ts - tf))

        # Calculate the current motor angles
        motor2Angle = motor2FinalAngle  # [CONSTANT]
        motor2AngleRate = 0.0
        motor2AngularAccel = 0.0  # [CONSTANT]

        # Find gimbal initial angles
        # Interpolate to find gimbal angles
        gimbal1InitAngle, gimbal2InitAngle = motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, intermediateInitialAngle, motor2Angle)

        # Find initial PRV
        dcm1 = np.array([[1,  0, 0],
                         [0, np.cos(deg2Rad * gimbal1InitAngle), np.sin(deg2Rad * gimbal1InitAngle)],
                         [0, -np.sin(deg2Rad * gimbal1InitAngle), np.cos(deg2Rad * gimbal1InitAngle)]])
        dcm2 = np.array([[np.cos(deg2Rad * gimbal2InitAngle),  0, -np.sin(deg2Rad * gimbal2InitAngle)],
                         [0, 1, 0],
                         [np.sin(deg2Rad * gimbal2InitAngle),  0, np.cos(deg2Rad * gimbal2InitAngle)]])

        if oneTwo:
            dcm_FM = np.matmul(dcm2, dcm1)
        else:
            dcm_FM = np.matmul(dcm1, dcm2)

        PRV_FM1 = rbk.C2PRV(dcm_FM)

        # Find final PRV
        # Interpolate to find gimbal angles
        gimbal1FinalAngle, gimbal2FinalAngle = motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, motor1FinalAngle, motor2Angle)

        dcm1 = np.array([[1,  0, 0],
                         [0, np.cos(deg2Rad * gimbal1FinalAngle), np.sin(deg2Rad * gimbal1FinalAngle)],
                         [0, -np.sin(deg2Rad * gimbal1FinalAngle), np.cos(deg2Rad * gimbal1FinalAngle)]])
        dcm2 = np.array([[np.cos(deg2Rad * gimbal2FinalAngle),  0, -np.sin(deg2Rad * gimbal2FinalAngle)],
                         [0, 1, 0],
                         [np.sin(deg2Rad * gimbal2FinalAngle),  0, np.cos(deg2Rad * gimbal2FinalAngle)]])

        if oneTwo:
            dcm_FM = np.matmul(dcm2, dcm1)
        else:
            dcm_FM = np.matmul(dcm1, dcm2)

        PRV_FM2 = rbk.C2PRV(dcm_FM)

        # Find relative PRV during the step
        relativePRV = rbk.subPRV(PRV_FM2, PRV_FM1)
        phiRef = np.linalg.norm(relativePRV)
        eHat = relativePRV / phiRef

        gimbalMaxAccel = phiRef / ((0.5 * stepSimTime) * (0.5 * stepSimTime))

        # Find gimbal parabolic constants
        c = (0.5 * phiRef) / ((ts - tInit) * (ts - tInit))
        d = (-0.5 * phiRef) / ((ts - tf) * (ts - tf))

        for idx4 in range(n_s):
            # Update current time
            t = t + dt

            if ((t < ts or t == ts) and tf - tInit != 0):
                motor1AngularAccel = motorMaxAccel
                motor1AngleRate = motor1AngularAccel * (t - tInit) + thetaDotInit
                motor1Angle = a * (t - tInit) * (t - tInit) + intermediateInitialAngle

                phiDDot = gimbalMaxAccel
                phiDot = phiDDot * (t - tInit)
                phi = c * (t - tInit) * (t - tInit)

            elif ( t > ts and t <= tf and tf - tInit != 0):
                motor1AngularAccel = -1 * motorMaxAccel
                motor1AngleRate = motor1AngularAccel * (t - tInit) + thetaDotInit - motor1AngularAccel * (tf - tInit)
                motor1Angle = b * (t - tf) * (t - tf) + motor1FinalAngle

                phiDDot = -1 * gimbalMaxAccel
                phiDot = phiDDot * (t - tInit) - phiDDot * (tf - tInit)
                phi = d * (t - tf) * (t - tf) + phiRef
            else:
                motor1AngularAccel = 0.0
                motor1AngleRate = thetaDotRef
                motor1Angle = motor2FinalAngle

                phiDDot = 0.0
                phiDot = 0.0
                phi = phiRef

            # Store time current motor and gimbal states into the data arrays
            timespan = np.append(timespan, t)
            motor2AngleData = np.append(motor2AngleData, motor2Angle)
            motor2AngleRateData = np.append(motor2AngleRateData, 0.0)
            motor2AngularAccelData = np.append(motor2AngularAccelData, 0.0)
            motor1AngleData = np.append(motor1AngleData, motor1Angle)
            motor1AngleRateData = np.append(motor1AngleRateData, motor1AngleRate)
            motor1AngularAccelData = np.append(motor1AngularAccelData, motor1AngularAccel)

            relativePRV = phi * eHat
            PRV_FMCurrent = rbk.addPRV(PRV_FM1, relativePRV)

            dcm_FM = rbk.PRV2C(PRV_FMCurrent)
            thrustAx = dcm_FM[2, :]
            thrustAxis = np.vstack((thrustAxis, thrustAx))

            gimbal1Angle = np.arctan(dcm_FM[1, 2] / dcm_FM[1, 1]) / deg2Rad
            gimbal2Angle = np.arctan(dcm_FM[2, 0] / dcm_FM[0, 0]) / deg2Rad
            gimbal1AngleData = np.append(gimbal1AngleData, gimbal1Angle)
            gimbal2AngleData = np.append(gimbal2AngleData, gimbal2Angle)

            sigma_FM = rbk.PRV2MRP(PRV_FMCurrent)
            sigma_FMData = np.vstack((sigma_FMData, np.array(sigma_FM)))

            omega_FM_F = phiDot * eHat
            omega_FM_FData = np.vstack((omega_FM_FData, omega_FM_F))

            omegaPrime_FM_F = phiDDot * eHat
            omegaPrime_FM_FData = np.vstack((omegaPrime_FM_FData, omegaPrime_FM_F))

        # Update motor states after a step has been completed
        intermediateInitialAngle = motor1FinalAngle

    # Call functions to plot results
    plotMotorData((1 / 60) * timespan, motor1AngleData, motor1AngleRateData, motor1AngularAccelData, motor2AngleData,
                                                                                          motor2AngleRateData,
                                                                                          motor2AngularAccelData,
                                                                                          motor1RefAngle,
                                                                                          motor2RefAngle)
    plotPrescribedGimbalStates((1 / 60) * timespan, sigma_FMData, omega_FM_FData, omegaPrime_FM_FData)

    plot2DGimbalMotion((1 / 60) * timespan, gimbal1AngleData, gimbal2AngleData)

    plotThrustAxis((1 / 60) * timespan, thrustAxis)

    # Write attitude data to a text file
    # sigmaFM_data_file = open(r"/Users/leahkiner/Desktop/sigma_FMDataM2M1_8.txt", "w+")
    # np.savetxt(sigmaFM_data_file, sigma_FMData)
    # sigmaFM_data_file.close()

    # Write gimbal angle data to a text file for 3d concept figure plotting
    gimbalTipAngles_data_file = open(r"/Users/leahkiner/Desktop/gimbal_tip_dataM2M1_8.txt", "w+")
    np.savetxt(gimbalTipAngles_data_file, gimbal1AngleData)
    gimbalTipAngles_data_file.close()

    gimbalTiltAngles_data_file = open(r"/Users/leahkiner/Desktop/gimbal_tilt_dataM2M1_8.txt", "w+")
    np.savetxt(gimbalTiltAngles_data_file, gimbal2AngleData)
    gimbalTiltAngles_data_file.close()

    # timespan_data_file = open(r"/Users/leahkiner/Desktop/timespan.txt", "w+")
    # np.savetxt(timespan_data_file, timespan)
    # timespan_data_file.close()

def motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, motor1Angle, motor2Angle):
    deg2Rad = np.pi / 180

    # Find motor angles in the provided table that most closely match to motor angles
    if motor1:
        lowerMotorAngle = tableStepAngle * math.floor(motor1Angle / tableStepAngle)
        upperMotorAngle = tableStepAngle * math.ceil(motor1Angle / tableStepAngle)
    else:
        lowerMotorAngle = tableStepAngle * math.floor(motor2Angle / tableStepAngle)
        upperMotorAngle = tableStepAngle * math.ceil(motor2Angle / tableStepAngle)

    # Extract the correct data for interpolation
    numRows = gimbal_data.shape[0]
    numCols = gimbal_data.shape[1]
    lowerGimbal1Angle = 360
    lowerGimbal2Angle = 360
    upperGimbal1Angle = 360
    upperGimbal2Angle = 360

    for idx in range(numRows):
        if motor1:
            if ((gimbal_data.iat[idx, 0] == lowerMotorAngle) and (gimbal_data.iat[idx, 1] == motor2Angle)):
                lowerGimbal1Angle = gimbal_data.iat[idx, 2]
                lowerGimbal2Angle = gimbal_data.iat[idx, 3]

            if ((gimbal_data.iat[idx, 0] == upperMotorAngle) and (gimbal_data.iat[idx, 1] == motor2Angle)):
                upperGimbal1Angle = gimbal_data.iat[idx, 2]
                upperGimbal2Angle = gimbal_data.iat[idx, 3]
        else:
            if ((gimbal_data.iat[idx, 1] == lowerMotorAngle) and (gimbal_data.iat[idx, 0] == motor1Angle)):
                lowerGimbal1Angle = gimbal_data.iat[idx, 2]
                lowerGimbal2Angle = gimbal_data.iat[idx, 3]

            if ((gimbal_data.iat[idx, 1] == upperMotorAngle) and (gimbal_data.iat[idx, 0] == motor1Angle)):
                upperGimbal1Angle = gimbal_data.iat[idx, 2]
                upperGimbal2Angle = gimbal_data.iat[idx, 3]

        if ((lowerGimbal1Angle != 360) and (lowerGimbal2Angle != 360) and (upperGimbal1Angle != 360) and (upperGimbal2Angle != 360)):
            break

    if upperMotorAngle != lowerMotorAngle:
        # Interpolate data to get estimated gimbal angles
        # Find lower PRV
        dcm1Lower = np.array([[1,  0, 0],
                              [0, np.cos(deg2Rad * lowerGimbal1Angle), np.sin(deg2Rad * lowerGimbal1Angle)],
                              [0, -np.sin(deg2Rad * lowerGimbal1Angle), np.cos(deg2Rad * lowerGimbal1Angle)]])
        dcm2Lower = np.array([[np.cos(deg2Rad * lowerGimbal2Angle),  0, -np.sin(deg2Rad * lowerGimbal2Angle)],
                              [0, 1, 0],
                              [np.sin(deg2Rad * lowerGimbal2Angle),  0, np.cos(deg2Rad * lowerGimbal2Angle)]])
        dcm_FMLower = np.matmul(dcm2Lower, dcm1Lower)
        PRV_FMLower = rbk.C2PRV(dcm_FMLower)

        # Find upper PRV
        dcm1Upper = np.array([[1,  0, 0],
                              [0, np.cos(deg2Rad * upperGimbal1Angle), np.sin(deg2Rad * upperGimbal1Angle)],
                              [0, -np.sin(deg2Rad * upperGimbal1Angle), np.cos(deg2Rad * upperGimbal1Angle)]])
        dcm2Upper = np.array([[np.cos(deg2Rad * upperGimbal2Angle),  0, -np.sin(deg2Rad * upperGimbal2Angle)],
                              [0, 1, 0],
                              [np.sin(deg2Rad * upperGimbal2Angle),  0, np.cos(deg2Rad * upperGimbal2Angle)]])
        dcm_FMUpper = np.matmul(dcm2Upper, dcm1Upper)
        PRV_FMUpper = rbk.C2PRV(dcm_FMUpper)

        # Find relative PRV
        PRV_relative = rbk.subPRV(PRV_FMUpper, PRV_FMLower)
        phi_relative = np.linalg.norm(PRV_relative)
        eHat_relative = PRV_relative / phi_relative

        if motor1:
            phi_interpolated = (phi_relative * (motor1Angle - lowerMotorAngle)) / (upperMotorAngle - lowerMotorAngle)
        else:
            phi_interpolated = (phi_relative * (motor2Angle - lowerMotorAngle)) / (upperMotorAngle - lowerMotorAngle)
        PRV_relative_interpolated = phi_interpolated * eHat_relative
        PRV_interpolated = rbk.addPRV(PRV_FMLower, PRV_relative_interpolated)
        dcm_interpolated = rbk.PRV2C(PRV_interpolated)
        gimbal1Angle = np.arctan(dcm_interpolated[1, 2] / dcm_interpolated[1, 1]) / deg2Rad
        gimbal2Angle = np.arctan(dcm_interpolated[2, 0] / dcm_interpolated[0, 0]) / deg2Rad
    else:
        gimbal1Angle = lowerGimbal1Angle
        gimbal2Angle = lowerGimbal2Angle

    return gimbal1Angle, gimbal2Angle


def plotMotorData(timespan, motor1AngleData, motor1AngleRateData, motor1AngularAccelData, motor2AngleData,
                                                                                          motor2AngleRateData,
                                                                                          motor2AngularAccelData,
                                                                                          motor1RefAngle,
                                                                                          motor2RefAngle):
    # Plot motor angle data
    motor1RefAngleData = np.ones(len(timespan)) * motor1RefAngle
    motor2RefAngleData = np.ones(len(timespan)) * motor2RefAngle
    plt.figure()
    plt.clf()
    plt.plot(timespan, motor1AngleData, label=r'$\theta_{1}$', color='mediumseagreen')
    plt.plot(timespan, motor2AngleData, label=r'$\theta_{2}$', color='mediumslateblue')
    # plt.title('Stepper Motor Angles', fontsize=14)
    plt.ylabel('(deg)', fontsize=16)
    plt.xlabel('Time (Min)', fontsize=16)
    plt.legend(loc='center right', prop={'size': 12})
    plt.grid(True)
    #
    # # Plot motor angle rate data
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan, motor1AngleRateData, label=r'$\dot{\theta}_{1}$')
    # plt.plot(timespan, motor2AngleRateData, label=r'$\dot{\theta}_{2}$')
    # plt.title('Stepper Motor Angle Rates', fontsize=14)
    # plt.ylabel('Angle Rate (deg/s)', fontsize=16)
    # plt.xlabel('Time (Min)', fontsize=16)
    # plt.legend(loc='lower right', prop={'size': 16})
    #
    # # Plot motor acceleration data
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan, motor1AngularAccelData, label=r'$\ddot{\theta}_{1}$')
    # plt.plot(timespan, motor2AngularAccelData, label=r'$\ddot{\theta}_{2}$')
    # plt.title('Stepper Motor Angular Accelerations', fontsize=14)
    # plt.ylabel('Angle Acceleration (deg/s$^2$)', fontsize=16)
    # plt.xlabel('Time (Min)', fontsize=16)
    # plt.legend(loc='lower right', prop={'size': 16})

    # Plot motor states on a single figure
    fig, plts = plt.subplots(3, 1, sharex=True)
    l1, = plts[0].plot(timespan, motor1AngleData, color='mediumseagreen')
    l2, = plts[0].plot(timespan, motor2AngleData, color='mediumslateblue')
    l3, = plts[1].plot(timespan, motor1AngleRateData, color='mediumseagreen')
    l4, = plts[1].plot(timespan, motor2AngleRateData, color='mediumslateblue')
    l5, = plts[2].plot(timespan, motor1AngularAccelData, color='mediumseagreen')
    l6, = plts[2].plot(timespan, motor2AngularAccelData, color='mediumslateblue')
    plts[0].set_ylabel("(deg)", fontsize=16)
    plts[1].set_ylabel("(deg/s)", fontsize=16)
    plts[2].set_ylabel("(deg/s$^2$)", fontsize=16)
    plts[2].set_xlabel("Time (Min)", fontsize=16)
    plts[0].legend((l1, l2), (r'$\theta_{1}$', r'$\theta_{2}$'), loc='center right', prop={'size': 12})
    plts[1].legend((l3, l4), (r'$\dot{\theta}_{1}$', r'$\dot{\theta}_{2}$',), loc='center right', prop={'size': 12})
    plts[2].legend((l5, l6), (r'$\ddot{\theta}_{1}$', r'$\ddot{\theta}_{2}$',), loc='center right', prop={'size': 12})
    # plts[0].set_title(r'Stepper Motor States')
    plts[0].grid(True)
    plts[1].grid(True)
    plts[2].grid(True)

def plotGimbalData(timespan, gimbal1AngleData, gimbal1AngleRateData, gimbal1AngularAccelData, gimbal2AngleData, gimbal2AngleRateData, gimbal2AngularAccelData):
    # # Plot gimbal angles
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan, gimbal1AngleData, label=r'$\psi_{1}$')
    # plt.plot(timespan, gimbal2AngleData, label=r'$\phi_{2}$')
    # plt.title('Interpolated Gimbal Angles', fontsize=14)
    # plt.ylabel('Angle (deg)', fontsize=16)
    # plt.xlabel('Time (Min)', fontsize=16)
    # plt.legend(loc='center right', prop={'size': 16})
    #

    # Plot gimbal states on a single figure
    fig, plts = plt.subplots(3, 1, sharex=True)
    l1, = plts[0].plot(timespan, gimbal1AngleData, color='darkturquoise')
    l2, = plts[0].plot(timespan, gimbal2AngleData, color='orchid')
    l3, = plts[1].plot(timespan, gimbal1AngleRateData, color='darkturquoise')
    l4, = plts[1].plot(timespan, gimbal2AngleRateData, color='orchid')
    l5, = plts[2].plot(timespan, gimbal1AngularAccelData, color='darkturquoise')
    l6, = plts[2].plot(timespan, gimbal2AngularAccelData, color='orchid')
    plts.flat[0].set(ylabel='(deg)')
    plts.flat[1].set(ylabel='(deg/s)')
    plts.flat[2].set(xlabel='Time (Min)', ylabel=r'(deg/s$^2$)')
    plts[0].legend((l1, l2), (r'$\psi$', r'$\phi$'), loc='upper right', prop={'size': 12})
    plts[1].legend((l3, l4), (r'$\dot{\psi}$', r'$\dot{\phi}$',), loc='upper right', prop={'size': 12})
    plts[2].legend((l5, l6), (r'$\ddot{\psi}$', r'$\ddot{\phi}$',), loc='upper right', prop={'size': 12})
    # plts[0].set_title('Gimbal Actuator States')
    plts[0].grid(True)
    plts[1].grid(True)
    plts[2].grid(True)

def plotPrescribedGimbalStates(timespan, sigma_FMData, omega_FM_FData, omegaPrime_FM_FData):
    # Plot gimbal attitude
    plt.figure()
    plt.clf()
    plt.plot(timespan, sigma_FMData[:, 0], label=r'$\sigma_{1}$', color='darkturquoise')
    plt.plot(timespan, sigma_FMData[:, 1], label=r'$\sigma_{2}$', color='orchid')
    plt.plot(timespan, sigma_FMData[:, 2], label=r'$\sigma_{3}$', color='mediumslateblue')
    #plt.title(r'Gimbal Attitude, $\sigma_{\mathcal{F} / \mathcal{M}}$', fontsize=14)
    plt.ylabel('', fontsize=16)
    plt.xlabel('Time (Min)', fontsize=16)
    plt.legend(loc='upper right', prop={'size': 16})
    plt.grid(True)

    # Plot gimbal angle velocity
    plt.figure()
    plt.clf()
    plt.plot(timespan, omega_FM_FData[:, 0], label=r'$\omega_{1}$', color='darkturquoise')
    plt.plot(timespan, omega_FM_FData[:, 1], label=r'$\omega_{2}$', color='orchid')
    plt.plot(timespan, omega_FM_FData[:, 2], label=r'$\omega_{3}$', color='mediumslateblue')
    #plt.title(r'Gimbal Angular Velocity ${}^{\mathcal{F}} \omega_{\mathcal{F} / \mathcal{M}}$', fontsize=14)
    plt.ylabel('(deg/s)', fontsize=16)
    plt.xlabel('Time (Min)', fontsize=16)
    plt.legend(loc='upper right', prop={'size': 16})
    plt.grid(True)

    # Plot gimbal angular acceleration
    plt.figure()
    plt.clf()
    plt.plot(timespan, omegaPrime_FM_FData[:, 0], label=r'$\alpha_{1}$', color='darkturquoise')
    plt.plot(timespan, omegaPrime_FM_FData[:, 1], label=r'$\alpha_{2}$', color='orchid')
    plt.plot(timespan, omegaPrime_FM_FData[:, 2], label=r'$\alpha_{3}$', color='mediumslateblue')
    #plt.title(r'Gimbal Angular Acceleration ${}^{\mathcal{F}} \alpha_{\mathcal{F} / \mathcal{M}}$', fontsize=14)
    plt.ylabel('(deg/s$^2$)', fontsize=16)
    plt.xlabel('Time (Min)', fontsize=16)
    plt.legend(loc='upper right', prop={'size': 16})
    plt.grid(True)

    # Plot prescribed gimbal states on a single figure
    fig, plts = plt.subplots(3, 1, sharex=True)
    l1, = plts[0].plot(timespan, sigma_FMData[:, 0], color='darkturquoise')
    l2, = plts[0].plot(timespan, sigma_FMData[:, 1], color='orchid')
    l3, = plts[0].plot(timespan, sigma_FMData[:, 2], color='mediumslateblue')
    l4, = plts[1].plot(timespan, omega_FM_FData[:, 0], color='darkturquoise')
    l5, = plts[1].plot(timespan, omega_FM_FData[:, 1], color='orchid')
    l6, = plts[1].plot(timespan, omega_FM_FData[:, 2], color='mediumslateblue')
    l7, = plts[2].plot(timespan, omegaPrime_FM_FData[:, 0], color='darkturquoise')
    l8, = plts[2].plot(timespan, omegaPrime_FM_FData[:, 1], color='orchid')
    l9, = plts[2].plot(timespan, omegaPrime_FM_FData[:, 2], color='mediumslateblue')
    plts[0].set_ylabel(r"$\sigma_{\mathcal{F} / \mathcal{M}}$", fontsize=16)
    plts[1].set_ylabel(r"${}^{\mathcal{F}} \omega_{\mathcal{F} / \mathcal{M}}$ (deg/s)", fontsize=16)
    plts[2].set_ylabel(r"${}^{\mathcal{F}} \alpha_{\mathcal{F} / \mathcal{M}}$ (deg/s$^2$)", fontsize=16)
    plts[2].set_xlabel(r"Time (Min)", fontsize=16)
    plts[0].legend((l1, l2, l3), (r'$\sigma_{1}$', r'$\sigma_{2}$', r'$\sigma_{3}$',), loc='upper right', prop={'size': 12})
    plts[1].legend((l4, l5, l6), (r'$\omega_{1}$', r'$\omega_{2}$', r'$\omega_{3}$',), loc='upper right', prop={'size': 12})
    plts[2].legend((l7, l8, l9), (r'$\alpha_{1}$', r'$\alpha_{2}$', r'$\alpha_{3}$',), loc='upper right', prop={'size': 12})
    # plts[0].set_title('Prescribed Gimbal States')
    plts[0].grid(True)
    plts[1].grid(True)
    plts[2].grid(True)

def plot2DGimbalMotion(timespan, gimbal1AngleData, gimbal2AngleData):
    # Plot 2D Diamond and the corresponding gimbal trajectory
    scaleG1 = 25 / 18.342
    scaleG2 = 30 / 27.43
    scaleMotor = 128 / 160
    line1 = [(18.342 * scaleG1, 0.0 * scaleG2), (0.676 * scaleG1, 27.43 * scaleG2)]
    line2 = [(-18.764 * scaleG1, 0.0 * scaleG2), (0.676 * scaleG1, 27.43 * scaleG2)]
    line3 = [(-18.764 * scaleG1, 0.0 * scaleG2), (0.676 * scaleG1, -27.43 * scaleG2)]
    line4 = [(18.342 * scaleG1, 0.0 * scaleG2), (0.676 * scaleG1, -27.43 * scaleG2)]
    lines = [line1, line2, line3, line4]
    lc = mc.LineCollection(lines, colors="crimson", linewidths=2, linestyles="dashed")
    plt.figure()
    plt.clf()
    ax = plt.axes()
    ax.add_collection(lc)
    plt.plot(gimbal1AngleData, gimbal2AngleData, linewidth=1, color='black')
    ax.scatter(gimbal1AngleData[0], gimbal2AngleData[0], marker='.', linewidth=4, color='springgreen', label='Initial')
    ax.scatter(gimbal1AngleData[-1], gimbal2AngleData[-1], marker='*', linewidth=4, color='magenta', label='Final')
    # plt.title('Gimbal Sequential Trajectory', fontsize=14)
    plt.ylabel(r'$\psi$ (deg)', fontsize=16)
    plt.xlabel(r'$\phi$ (deg)', fontsize=16)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)
    plt.axis("equal")

def plotThrustAxis(timespan, thrustAxis):
    # # Plot thrust axis
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan[1:], thrustAxis[1:, 0], label='1')
    # plt.plot(timespan[1:], thrustAxis[1:, 1], label='2')
    # plt.plot(timespan[1:], thrustAxis[1:, 2], label='3')
    # plt.title('Thrust Axis', fontsize=14)
    # plt.ylabel('', fontsize=16)
    # plt.xlabel('Time (Min)', fontsize=16)
    # plt.legend(loc='lower right', prop={'size': 16})

    # Plot 3d thrust direction
    plt.figure()
    plt.clf()
    ax = plt.axes(projection='3d')
    u = np.linspace(0, np.pi, 30)
    v = np.linspace(0, 2 * np.pi, 30)
    x = np.outer(np.sin(u), np.sin(v))
    y = np.outer(np.sin(u), np.cos(v))
    z = np.outer(np.cos(u), np.ones_like(v))
    ax.plot_wireframe(x, y, z, linewidth=0.2, linestyle="dashed", edgecolor="grey")
    ax.plot3D(thrustAxis[1:, 0], thrustAxis[1:, 1], thrustAxis[1:, 2], color='black')
    ax.scatter(thrustAxis[1, 0], thrustAxis[1, 1], thrustAxis[1, 2], marker='.', linewidth=2, color='springgreen', label='Initial Thrust Direction')
    ax.scatter(thrustAxis[-1, 0], thrustAxis[-1, 1], thrustAxis[-1, 2], marker='*', linewidth=2, color='magenta', label='Final Thrust Direction')
    ax.quiver(0, 0, 0, 1, 0, 0, length=0.75, linewidth=0.75, color="black")
    ax.quiver(0, 0, 0, 0, 1, 0, length=0.75, linewidth=0.75, color="black")
    ax.quiver(0, 0, 0, 0, 0, 1, length=0.75, linewidth=0.75, color="black")
    # plt.title('Thruster Boresight', fontsize=14)
    ax.set_zlabel('$\hat{m}_3$', fontsize=14)
    ax.set_ylabel('$\hat{m}_2$', fontsize=14)
    ax.set_xlabel(r'$\hat{m}_1$', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})


if __name__ == "__main__":
    run()
    plt.show()
    plt.close("all")
