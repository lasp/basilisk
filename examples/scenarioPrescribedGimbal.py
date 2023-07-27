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
# Creation Date:  July 13, 2023
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


def run():
    deg2Rad = np.pi / 180

    path_to_file = "/Users/leahkiner/Repositories/MAX_Basilisk/examples/gimbal_data.csv"
    gimbal_data = pd.read_csv(path_to_file)

    # Define stepper motor fixed parameters
    motorStepAngle = 1  # [deg]
    tableStepAngle = 2.5  # [deg]

    # Define gimbal fixed parameters
    rotAxis1 = np.array([1.0, 0.0, 0.0])
    rotAxis2 = np.array([0.0, 1.0, 0.0])

    # Choose which motor is used
    motor1 = True
    motor2 = False

    # Choose method of determining gimbal attitude
    oneTwo = True
    twoOne = False

    # Define initial motor states
    motor1InitAngle = 0
    motor2InitAngle = 0
    gimbal1InitAngle = 18.342
    gimbal2InitAngle = 0.0
    thetaDotInit = 0.0
    thetaDotRef = 0.0

    # Define number of steps each motor takes
    numSteps1 = 160  # Note that numSteps * motorStepAngle must be in increments of 2.5 degrees
    numSteps2 = 160

    # Check to make sure that the final motor 1 angle exists in a row of the given motor/gimbal table data
    if ((numSteps1 * motorStepAngle) % 2.5 != 0):
        print("ERROR: Motor 1 Must End At An Increment of 2.5 Degrees")

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
    gimbal1AngleRateData = np.array(0.0)
    gimbal1AngularAccelData = np.array(0.0)
    gimbal2AngleData = np.array(gimbal2InitAngle)
    gimbal2AngleRateData = np.array(0.0)
    gimbal2AngularAccelData = np.array(0.0)
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
    dt = 0.5  # [s]
    t = 0.0
    motorMaxAccel = ((0.5 * np.abs(motorStepAngle)) * 8) / 100
    stepSimTime = np.sqrt(((0.5 * np.abs(motorStepAngle)) * 8) / motorMaxAccel)  # [s]
    n_s = int(stepSimTime / dt)
    timespan = np.array(t)

    # MOVE MOTOR 1
    intermediateInitialAngle = motor1InitAngle
    for idx1 in range(numSteps1):
        motor1FinalAngle = motor1InitAngle + ((idx1 + 1) * motorStepAngle)
        tInit = t

        # Find switch time and final time
        ts = tInit + (0.5 * stepSimTime)
        tf = tInit + stepSimTime

        # Find parabolic constants
        a = (0.5 * motorStepAngle) / ((ts - tInit) * (ts - tInit))
        b = (-0.5 * motorStepAngle) / ((ts - tf) * (ts - tf))

        # Calculate the current motor angles
        motor2Angle = motor2InitAngle  # [CONSTANT]
        motor2AngleRate = 0.0  # [CONSTANT]
        motor2AngularAccel = 0.0  # [CONSTANT]

        for idx2 in range(n_s):
            # Update current time
            t = t + dt

            if ((t < ts or t == ts) and tf - tInit != 0):
                motor1AngularAccel = motorMaxAccel
                motor1AngleRate = motor1AngularAccel * (t - tInit) + thetaDotInit
                motor1Angle = a * (t - tInit) * (t - tInit) + intermediateInitialAngle

            elif ( t > ts and t <= tf and tf - tInit != 0):
                motor1AngularAccel = -1 * motorMaxAccel
                motor1AngleRate = motor1AngularAccel * (t - tInit) + thetaDotInit - motor1AngularAccel * (tf - tInit)
                motor1Angle = b * (t - tf) * (t - tf) + motor1FinalAngle
            else:
                motor1AngularAccel = 0.0
                motor1AngleRate = thetaDotRef
                motor1Angle = motor1FinalAngle

            # Interpolate to find gimbal angles
            gimbal1Angle, gimbal2Angle, gimbal1AngleRate, gimbal2AngleRate, gimbal1AngularAccel, gimbal2AngularAccel = motorToGimbal(gimbal_data,
                                                                                                                                     tableStepAngle,
                                                                                                                                     motor1,
                                                                                                                                     motor2,
                                                                                                                                     motor1Angle,
                                                                                                                                     motor1AngleRate,
                                                                                                                                     motor1AngularAccel,
                                                                                                                                     motor2Angle,
                                                                                                                                     motor2AngleRate,
                                                                                                                                     motor2AngularAccel)

            # Store time current motor and gimbal states into the data arrays
            timespan = np.append(timespan, t)
            motor1AngleData = np.append(motor1AngleData, motor1Angle)
            motor1AngleRateData = np.append(motor1AngleRateData, motor1AngleRate)
            motor1AngularAccelData = np.append(motor1AngularAccelData, motor1AngularAccel)
            motor2AngleData = np.append(motor2AngleData, motor2Angle)
            motor2AngleRateData = np.append(motor2AngleRateData, 0.0)
            motor2AngularAccelData = np.append(motor2AngularAccelData, 0.0)
            gimbal1AngleData = np.append(gimbal1AngleData, gimbal1Angle)
            gimbal1AngleRateData = np.append(gimbal1AngleRateData, gimbal1AngleRate)
            gimbal1AngularAccelData = np.append(gimbal1AngularAccelData, gimbal1AngularAccel)
            gimbal2AngleData = np.append(gimbal2AngleData, gimbal2Angle)
            gimbal2AngleRateData = np.append(gimbal2AngleRateData, gimbal2AngleRate)
            gimbal2AngularAccelData = np.append(gimbal2AngularAccelData, gimbal2AngularAccel)

            dcm1 = np.array([[1,  0, 0],
                             [0, np.cos(deg2Rad * gimbal1Angle), np.sin(deg2Rad * gimbal1Angle)],
                             [0, -np.sin(deg2Rad * gimbal1Angle), np.cos(deg2Rad * gimbal1Angle)]])
            dcm2 = np.array([[np.cos(deg2Rad * gimbal2Angle),  0, -np.sin(deg2Rad * gimbal2Angle)],
                             [0, 1, 0],
                             [np.sin(deg2Rad * gimbal2Angle),  0, np.cos(deg2Rad * gimbal2Angle)]])

            if oneTwo:
                dcm_FM = np.matmul(dcm2, dcm1)
            else:
                dcm_FM = np.matmul(dcm1, dcm2)

            thrustAx = dcm_FM[2, :]
            thrustAxis = np.vstack((thrustAxis, thrustAx))
            sigma_FM = rbk.C2MRP(dcm_FM)
            sigma_FMData = np.vstack((sigma_FMData, np.array(sigma_FM)))

            omega_FM_F = np.matmul(dcm_FM, np.transpose(gimbal1AngleRate * rotAxis1)) + (gimbal2AngleRate * rotAxis2)
            omega_FM_FData = np.vstack((omega_FM_FData, omega_FM_F))

            omegaPrime_FM_F = np.matmul(dcm_FM, np.transpose(gimbal1AngularAccel * rotAxis1)) + (gimbal1AngularAccel * rotAxis2)
            omegaPrime_FM_FData = np.vstack((omegaPrime_FM_FData, omegaPrime_FM_F))

        # Update motor states after a step has been completed
        intermediateInitialAngle = motor1FinalAngle

    # MOVE MOTOR 2
    motor1 = False
    motor2 = True
    intermediateInitialAngle = motor2InitAngle
    for idx3 in range(numSteps2):
        motor2FinalAngle = motor2InitAngle + ((idx3 + 1) * motorStepAngle)
        tInit = t

        # Find switch time and final time
        ts = tInit + (0.5 * stepSimTime)
        tf = tInit + stepSimTime

        # Find parabolic constants
        a = (0.5 * motorStepAngle) / ((ts - tInit) * (ts - tInit))
        b = (-0.5 * motorStepAngle) / ((ts - tf) * (ts - tf))

        # Calculate the current motor angles
        motor1Angle = motor1FinalAngle  # [CONSTANT]
        motor1AngleRate = 0.0
        motor1AngularAccel = 0.0  # [CONSTANT]

        for idx4 in range(n_s):
            # Update current time
            t = t + dt

            if ((t < ts or t == ts) and tf - tInit != 0):
                motor2AngularAccel = motorMaxAccel
                motor2AngleRate = motor2AngularAccel * (t - tInit) + thetaDotInit
                motor2Angle = a * (t - tInit) * (t - tInit) + intermediateInitialAngle

            elif ( t > ts and t <= tf and tf - tInit != 0):
                motor2AngularAccel = -1 * motorMaxAccel
                motor2AngleRate = motor2AngularAccel * (t - tInit) + thetaDotInit - motor2AngularAccel * (tf - tInit)
                motor2Angle = b * (t - tf) * (t - tf) + motor2FinalAngle
            else:
                motor2AngularAccel = 0.0
                motor2AngleRate = thetaDotRef
                motor2Angle = motor2FinalAngle

            # Interpolate to find gimbal angles
            gimbal1Angle, gimbal2Angle, gimbal1AngleRate, gimbal2AngleRate, gimbal1AngularAccel, gimbal2AngularAccel = motorToGimbal(gimbal_data,
                                                                                                                                     tableStepAngle,
                                                                                                                                     motor1,
                                                                                                                                     motor2,
                                                                                                                                     motor1Angle,
                                                                                                                                     motor1AngleRate,
                                                                                                                                     motor1AngularAccel,
                                                                                                                                     motor2Angle,
                                                                                                                                     motor2AngleRate,
                                                                                                                                     motor2AngularAccel)

            # Store time current motor and gimbal states into the data arrays
            timespan = np.append(timespan, t)
            motor2AngleData = np.append(motor2AngleData, motor2Angle)
            motor2AngleRateData = np.append(motor2AngleRateData, motor2AngleRate)
            motor2AngularAccelData = np.append(motor2AngularAccelData, motor2AngularAccel)
            motor1AngleData = np.append(motor1AngleData, motor1Angle)
            motor1AngleRateData = np.append(motor1AngleRateData, 0.0)
            motor1AngularAccelData = np.append(motor1AngularAccelData, 0.0)
            gimbal1AngleData = np.append(gimbal1AngleData, gimbal1Angle)
            gimbal1AngleRateData = np.append(gimbal1AngleRateData, gimbal1AngleRate)
            gimbal1AngularAccelData = np.append(gimbal1AngularAccelData, gimbal1AngularAccel)
            gimbal2AngleData = np.append(gimbal2AngleData, gimbal2Angle)
            gimbal2AngleRateData = np.append(gimbal2AngleRateData, gimbal2AngleRate)
            gimbal2AngularAccelData = np.append(gimbal2AngularAccelData, gimbal2AngularAccel)

            dcm1 = np.array([[1,  0, 0],
                             [0, np.cos(deg2Rad * gimbal1Angle), np.sin(deg2Rad * gimbal1Angle)],
                             [0, -np.sin(deg2Rad * gimbal1Angle), np.cos(deg2Rad * gimbal1Angle)]])
            dcm2 = np.array([[np.cos(deg2Rad * gimbal2Angle),  0, -np.sin(deg2Rad * gimbal2Angle)],
                             [0, 1, 0],
                             [np.sin(deg2Rad * gimbal2Angle),  0, np.cos(deg2Rad * gimbal2Angle)]])
            if oneTwo:
                dcm_FM = np.matmul(dcm2, dcm1)
            else:
                dcm_FM = np.matmul(dcm1, dcm2)

            thrustAx = dcm_FM[2, :]
            thrustAxis = np.vstack((thrustAxis, thrustAx))
            sigma_FM = rbk.C2MRP(dcm_FM)
            sigma_FMData = np.vstack((sigma_FMData, np.array(sigma_FM)))

            omega_FM_F = np.matmul(dcm_FM, np.transpose(gimbal1AngleRate * rotAxis1)) + (gimbal2AngleRate * rotAxis2)
            omega_FM_FData = np.vstack((omega_FM_FData, omega_FM_F))

            omegaPrime_FM_F = np.matmul(dcm_FM, np.transpose(gimbal1AngularAccel * rotAxis1)) + (gimbal1AngularAccel * rotAxis2)
            omegaPrime_FM_FData = np.vstack((omegaPrime_FM_FData, omegaPrime_FM_F))

        # Update motor states after a step has been completed
        intermediateInitialAngle = motor2FinalAngle

    # Call functions to plot results
    plotMotorData(timespan, motor1AngleData, motor1AngleRateData, motor1AngularAccelData, motor2AngleData,
                                                                                          motor2AngleRateData,
                                                                                          motor2AngularAccelData,
                                                                                          motor1RefAngle,
                                                                                          motor2RefAngle)
    plotGimbalData(timespan, gimbal1AngleData, gimbal1AngleRateData, gimbal1AngularAccelData, gimbal2AngleData,
                                                                                              gimbal2AngleRateData,
                                                                                              gimbal2AngularAccelData)
    plotPrescribedGimbalStates(timespan, sigma_FMData, omega_FM_FData, omegaPrime_FM_FData)

    plot2DGimbalMotion(gimbal1AngleData, gimbal2AngleData)

    plotThrustAxis(timespan, thrustAxis)

    # Write attitude data to a text file for 3d concept figure plotting
    sigmaFM_data_file = open(r"/Users/leahkiner/Desktop/sigma_FMData.txt", "w+")
    np.savetxt(sigmaFM_data_file, sigma_FMData)
    sigmaFM_data_file.close()


def motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, motor1Angle, motor1AngleRate, motor1AngularAccel,
                  motor2Angle,
                  motor2AngleRate,
                  motor2AngularAccel):

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

    # Interpolate data to get estimated gimbal angles
    if motor1:
        if upperMotorAngle != lowerMotorAngle:
            gimbal1Angle = ((lowerGimbal1Angle * (upperMotorAngle - motor1Angle)) + (upperGimbal1Angle * (motor1Angle - lowerMotorAngle))) / (upperMotorAngle - lowerMotorAngle)
            gimbal2Angle = ((lowerGimbal2Angle * (upperMotorAngle - motor1Angle)) + (upperGimbal2Angle * (motor1Angle - lowerMotorAngle))) / (upperMotorAngle - lowerMotorAngle)
            gimbal1AngleRate = motor1AngleRate * ((upperGimbal1Angle - lowerGimbal1Angle) / (upperMotorAngle - lowerMotorAngle))
            gimbal2AngleRate = motor1AngleRate * ((upperGimbal2Angle - lowerGimbal2Angle) / (upperMotorAngle - lowerMotorAngle))
            gimbal1AngularAccel = motor1AngularAccel * ((upperGimbal1Angle - lowerGimbal1Angle) / (upperMotorAngle - lowerMotorAngle))
            gimbal2AngularAccel = motor1AngularAccel * ((upperGimbal2Angle - lowerGimbal2Angle) / (upperMotorAngle - lowerMotorAngle))
        else:
            gimbal1Angle = lowerGimbal1Angle
            gimbal2Angle = lowerGimbal2Angle
            gimbal1AngleRate = (gimbal1Angle / motor1Angle) * motor1AngleRate
            gimbal2AngleRate = (gimbal2Angle / motor1Angle) * motor1AngleRate
            gimbal1AngularAccel = (gimbal1Angle / motor1Angle) * motor1AngularAccel
            gimbal2AngularAccel = (gimbal2Angle / motor1Angle) * motor1AngularAccel
    else:
        if upperMotorAngle != lowerMotorAngle:
            gimbal1Angle = ((lowerGimbal1Angle * (upperMotorAngle - motor2Angle)) + (upperGimbal1Angle * (motor2Angle - lowerMotorAngle))) / (upperMotorAngle - lowerMotorAngle)
            gimbal2Angle = ((lowerGimbal2Angle * (upperMotorAngle - motor2Angle)) + (upperGimbal2Angle * (motor2Angle - lowerMotorAngle))) / (upperMotorAngle - lowerMotorAngle)
            gimbal1AngleRate = motor2AngleRate * ((upperGimbal1Angle - lowerGimbal1Angle) / (upperMotorAngle - lowerMotorAngle))
            gimbal2AngleRate = motor2AngleRate * ((upperGimbal2Angle - lowerGimbal2Angle) / (upperMotorAngle - lowerMotorAngle))
            gimbal1AngularAccel = motor2AngularAccel * ((upperGimbal1Angle - lowerGimbal1Angle) / (upperMotorAngle - lowerMotorAngle))
            gimbal2AngularAccel = motor2AngularAccel * ((upperGimbal2Angle - lowerGimbal2Angle) / (upperMotorAngle - lowerMotorAngle))
        else:
            gimbal1Angle = lowerGimbal1Angle
            gimbal2Angle = lowerGimbal2Angle
            gimbal1AngleRate = (gimbal1Angle / motor2Angle) * motor2AngleRate
            gimbal2AngleRate = (gimbal2Angle / motor2Angle) * motor2AngleRate
            gimbal1AngularAccel = (gimbal1Angle / motor2Angle) * motor2AngularAccel
            gimbal2AngularAccel = (gimbal2Angle / motor2Angle) * motor2AngularAccel

    return gimbal1Angle, gimbal2Angle, gimbal1AngleRate, gimbal2AngleRate, gimbal1AngularAccel, gimbal2AngularAccel


def plotMotorData(timespan, motor1AngleData, motor1AngleRateData, motor1AngularAccelData, motor2AngleData,
                                                                                          motor2AngleRateData,
                                                                                          motor2AngularAccelData,
                                                                                          motor1RefAngle,
                                                                                          motor2RefAngle):
    # # Plot motor angle data
    # motor1RefAngleData = np.ones(len(timespan)) * motor1RefAngle
    # motor2RefAngleData = np.ones(len(timespan)) * motor2RefAngle
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan, motor1AngleData, label=r'$\theta_{1}$')
    # plt.plot(timespan, motor2AngleData, label=r'$\theta_{2}$')
    # plt.plot(timespan, motor1RefAngleData, '--', label=r'$\theta_{Ref1}$')
    # plt.plot(timespan, motor2RefAngleData, '--', label=r'$\theta_{Ref2}$')
    # plt.title('Stepper Motor Angles', fontsize=14)
    # plt.ylabel('Angle (deg)', fontsize=16)
    # plt.xlabel('Time (s)', fontsize=16)
    # plt.legend(loc='lower right', prop={'size': 16})
    #
    # # Plot motor angle rate data
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan, motor1AngleRateData, label=r'$\dot{\theta}_{1}$')
    # plt.plot(timespan, motor2AngleRateData, label=r'$\dot{\theta}_{2}$')
    # plt.title('Stepper Motor Angle Rates', fontsize=14)
    # plt.ylabel('Angle Rate (deg/s)', fontsize=16)
    # plt.xlabel('Time (s)', fontsize=16)
    # plt.legend(loc='lower right', prop={'size': 16})
    #
    # # Plot motor acceleration data
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan, motor1AngularAccelData, label=r'$\ddot{\theta}_{1}$')
    # plt.plot(timespan, motor2AngularAccelData, label=r'$\ddot{\theta}_{2}$')
    # plt.title('Stepper Motor Angular Accelerations', fontsize=14)
    # plt.ylabel('Angle Acceleration (deg/s$^2$)', fontsize=16)
    # plt.xlabel('Time (s)', fontsize=16)
    # plt.legend(loc='lower right', prop={'size': 16})

    # Plot motor states on a single figure
    fig, plts = plt.subplots(3, 1, sharex=True)
    l1, = plts[0].plot(timespan, motor1AngleData, color='mediumseagreen')
    l2, = plts[0].plot(timespan, motor2AngleData, color='mediumslateblue')
    l3, = plts[1].plot(timespan, motor1AngleRateData, color='mediumseagreen')
    l4, = plts[1].plot(timespan, motor2AngleRateData, color='mediumslateblue')
    l5, = plts[2].plot(timespan, motor1AngularAccelData, color='mediumseagreen')
    l6, = plts[2].plot(timespan, motor2AngularAccelData, color='mediumslateblue')
    plts.flat[0].set(ylabel='(deg)')
    plts.flat[1].set(ylabel='(deg/s)')
    plts.flat[2].set(xlabel='Time (s)', ylabel='(deg/s$^2$)')
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
    # plt.xlabel('Time (s)', fontsize=16)
    # plt.legend(loc='center right', prop={'size': 16})
    #
    # # Plot gimbal angle rates
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan, gimbal1AngleRateData, label=r'$\dot{\psi}_{1}$')
    # plt.plot(timespan, gimbal2AngleRateData, label=r'$\dot{\phi}_{2}$')
    # plt.title('Interpolated Gimbal Angle Rates', fontsize=14)
    # plt.ylabel('Angle Rate (deg/s)', fontsize=16)
    # plt.xlabel('Time (s)', fontsize=16)
    # plt.legend(loc='center right', prop={'size': 16})
    #
    # # Plot gimbal angular accelerations
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan, gimbal1AngularAccelData, label=r'$\ddot{\psi}_{1}$')
    # plt.plot(timespan, gimbal2AngularAccelData, label=r'$\ddot{\phi}_{2}$')
    # plt.title('Interpolated Gimbal Angular Accelerations', fontsize=14)
    # plt.ylabel('Angle Acceleration (deg/s$^2$)', fontsize=16)
    # plt.xlabel('Time (s)', fontsize=16)
    # plt.legend(loc='center right', prop={'size': 16})

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
    plts.flat[2].set(xlabel='Time (s)', ylabel='(deg/s$^2$)')
    plts[0].legend((l1, l2), (r'$\psi$', r'$\phi$'), loc='center right', prop={'size': 12})
    plts[1].legend((l3, l4), (r'$\dot{\psi}$', r'$\dot{\phi}$',), loc='center right', prop={'size': 12})
    plts[2].legend((l5, l6), (r'$\ddot{\psi}$', r'$\ddot{\phi}$',), loc='center right', prop={'size': 12})
    # plts[0].set_title('Gimbal Actuator States')
    plts[0].grid(True)
    plts[1].grid(True)
    plts[2].grid(True)

def plotPrescribedGimbalStates(timespan, sigma_FMData, omega_FM_FData, omegaPrime_FM_FData):
    # # Plot gimbal attitude
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan, sigma_FMData[:, 0], label=r'$\sigma_{1}$')
    # plt.plot(timespan, sigma_FMData[:, 1], label=r'$\sigma_{2}$')
    # plt.plot(timespan, sigma_FMData[:, 2], label=r'$\sigma_{3}$')
    # plt.title(r'Gimbal Attitude, $\sigma_{\mathcal{F} / \mathcal{M}}$', fontsize=14)
    # plt.ylabel('', fontsize=16)
    # plt.xlabel('Time (s)', fontsize=16)
    # plt.legend(loc='center right', prop={'size': 16})
    #
    # # Plot gimbal angle velocity
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan, omega_FM_FData[:, 0], label=r'$\omega_{1}$')
    # plt.plot(timespan, omega_FM_FData[:, 1], label=r'$\omega_{2}$')
    # plt.plot(timespan, omega_FM_FData[:, 2], label=r'$\omega_{3}$')
    # plt.title(r'Gimbal Angular Velocity ${}^{\mathcal{F}} \omega_{\mathcal{F} / \mathcal{M}}$', fontsize=14)
    # plt.ylabel('Angular Rate (deg/s)', fontsize=16)
    # plt.xlabel('Time (s)', fontsize=16)
    # plt.legend(loc='center right', prop={'size': 16})
    #
    # # Plot gimbal angular acceleration
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan, omegaPrime_FM_FData[:, 0], label=r'$\alpha_{1}$')
    # plt.plot(timespan, omegaPrime_FM_FData[:, 1], label=r'$\alpha_{2}$')
    # plt.plot(timespan, omegaPrime_FM_FData[:, 2], label=r'$\alpha_{3}$')
    # plt.title(r'Gimbal Angular Acceleration ${}^{\mathcal{F}} \alpha_{\mathcal{F} / \mathcal{M}}$', fontsize=14)
    # plt.ylabel('Angular Acceleration (deg/s$^2$)', fontsize=16)
    # plt.xlabel('Time (s)', fontsize=16)
    # plt.legend(loc='center right', prop={'size': 16})


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
    plts.flat[0].set(ylabel=r'$\sigma_{\mathcal{F} / \mathcal{M}}$')
    plts.flat[1].set(ylabel=r'${}^{\mathcal{F}} \omega_{\mathcal{F} / \mathcal{M}}$ (deg/s)')
    plts.flat[2].set(xlabel='Time (s)', ylabel=r'${}^{\mathcal{F}} \alpha_{\mathcal{F} / \mathcal{M}}$ (deg/s$^2$)')
    plts[0].legend((l1, l2, l3), (r'$\sigma_{1}$', r'$\sigma_{2}$', r'$\sigma_{3}$',), loc='center right', prop={'size': 12})
    plts[1].legend((l4, l5, l6), (r'$\omega_{1}$', r'$\omega_{2}$', r'$\omega_{3}$',), loc='center right', prop={'size': 12})
    plts[2].legend((l7, l8, l9), (r'$\alpha_{1}$', r'$\alpha_{2}$', r'$\alpha_{3}$',), loc='center right', prop={'size': 12})
    # plts[0].set_title('Prescribed Gimbal States')
    plts[0].grid(True)
    plts[1].grid(True)
    plts[2].grid(True)

def plot2DGimbalMotion(gimbal1AngleData, gimbal2AngleData):
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
    # plt.xlabel('Time (s)', fontsize=16)
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
