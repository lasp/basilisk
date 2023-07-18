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
from Basilisk.utilities import RigidBodyKinematics as rbk


def run():

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

    # Define initial motor states
    motor1InitAngle = 0
    motor2InitAngle = 0
    thetaDotInit = 0.0
    thetaDotRef = 0.0

    # Define number of steps each motor takes
    numSteps1 = 10  # Note that numSteps * motorStepAngle must be in increments of 2.5 degrees
    numSteps2 = 5

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
    gimbal1AngleData = np.array(0.0)
    gimbal1AngleRateData = np.array(0.0)
    gimbal1AngularAccelData = np.array(0.0)
    gimbal2AngleData = np.array(0.0)
    gimbal2AngleRateData = np.array(0.0)
    gimbal2AngularAccelData = np.array(0.0)


    dcm1 = np.array([[1,  0, 0],
                    [0, np.cos(motor1InitAngle), np.sin(motor1InitAngle)],
                    [0, -np.sin(motor1InitAngle), np.cos(motor1InitAngle)]])
    dcm2 = np.array([[np.cos(motor2InitAngle),  0, -np.sin(motor2InitAngle)],
                     [0, 1, 0],
                     [np.sin(motor2InitAngle),  0, np.cos(motor2InitAngle)]])
    dcm_FMInit = np.matmul(dcm2, dcm1)
    sigma_FMInit = rbk.C2MRP(dcm_FMInit)
    sigma_FMData = np.array([sigma_FMInit[0], sigma_FMInit[1], sigma_FMInit[2]])
    omega_FM_FData = np.array([0.0, 0.0, 0.0])
    omegaPrime_FM_FData = np.array([0.0, 0.0, 0.0])

    # Define temporal parameters
    dt = 0.1  # [s]
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

            dcm_FM = np.array([[1,  0, 0],
                               [0, np.cos(gimbal1Angle), np.sin(gimbal1Angle)],
                               [0, -np.sin(gimbal1Angle), np.cos(gimbal1Angle)]])
            sigma_FM = rbk.C2MRP(dcm_FM)
            sigma_fm = np.array(sigma_FM)
            sigma_FMData = np.vstack((sigma_FMData, sigma_fm))
            omega_FM_FData = np.vstack((omega_FM_FData, gimbal1AngleRate * rotAxis1))
            omegaPrime_FM_FData = np.vstack((omegaPrime_FM_FData, gimbal1AngularAccel * rotAxis1))

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


            dcm_FM = np.array([[np.cos(gimbal2Angle),  0, -np.sin(gimbal2Angle)],
                            [0, 1, 0],
                            [np.sin(gimbal2Angle),  0, np.cos(gimbal2Angle)]])
            sigma_FM = rbk.C2MRP(dcm_FM)
            sigma_fm = np.array(sigma_FM)
            sigma_FMData = np.vstack((sigma_FMData, sigma_fm))
            omega_FM_FData = np.vstack((omega_FM_FData, gimbal2AngleRate * rotAxis2))
            omegaPrime_FM_FData = np.vstack((omegaPrime_FM_FData, gimbal2AngularAccel * rotAxis2))

        # Update motor states after a step has been completed
        intermediateInitialAngle = motor2FinalAngle

    print(sigma_FMData)
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

    plt.show()
    plt.close("all")


def motorToGimbal(gimbal_data, tableStepAngle, motor1, motor2, motor1Angle, motor1AngleRate, motor1AngularAccel,
                  motor2Angle,
                  motor2AngleRate,
                  motor2AngularAccel):

    # Find closest match to motor angles
    if motor1:
        lowerMotorAngle = tableStepAngle * math.floor(motor1Angle / tableStepAngle)
        upperMotorAngle = tableStepAngle * math.ceil(motor1Angle / tableStepAngle)
    else:
        lowerMotorAngle = tableStepAngle * math.floor(motor2Angle / tableStepAngle)
        upperMotorAngle = tableStepAngle * math.ceil(motor2Angle / tableStepAngle)

    # # Check correct upper and lower angles were found
    # print("Closest Lower Angle:")
    # print(lowerMotorAngle)
    # print("Closest Upper Angle:")
    # print(upperMotorAngle)

    # Extract the correct data for interpolation
    numRows = gimbal_data.shape[0]
    numCols = gimbal_data.shape[1]
    lowerGimbal1Angle = 360
    lowerGimbal2Angle = 360
    upperGimbal1Angle = 360
    uppperGimbal2Angle = 360

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

    # Adjust upper and lower angles based on which is truly larger than the other
    if lowerGimbal1Angle > upperGimbal1Angle:
        tempAngle = lowerGimbal1Angle
        lowerGimbal1Angle = upperGimbal1Angle
        upperGimbal1Angle = tempAngle

    if lowerGimbal2Angle > upperGimbal2Angle:
        tempAngle = lowerGimbal2Angle
        lowerGimbal2Angle = upperGimbal2Angle
        upperGimbal2Angle = tempAngle

    # # Check correct gimbal angles were found
    # print("\n\nClosest G1 Lower Angle:")
    # print(lowerGimbal1Angle)
    # print("Closest G1 Upper Angle:")
    # print(upperGimbal1Angle)
    # print("\nClosest G2 Lower Angle:")
    # print(lowerGimbal2Angle)
    # print("Closest G2 Upper Angle:")
    # print(upperGimbal2Angle)

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
            gimbal1AngleRate = 0.0 #motor1AngleRate
            gimbal2AngleRate = 0.0  #motor1AngleRate
            gimbal1AngularAccel = 0.0  #motor1AngularAccel
            gimbal2AngularAccel = 0.0  #motor1AngularAccel
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
            gimbal1AngleRate = 0.0 #motor2AngleRate
            gimbal2AngleRate = 0.0  #motor2AngleRate
            gimbal1AngularAccel = 0.0  #motor2AngularAccel
            gimbal2AngularAccel = 0.0  #motor2AngularAccel

    # # Print interpolated results
    # print("\n\n ** ** ** INTERPOLATED RESULTS ** ** **")
    # print("GIMBAL 1 ANGLE:")
    # print(gimbal1Angle)
    # print("GIMBAL 2 ANGLE:")
    # print(gimbal2Angle)

    return gimbal1Angle, gimbal2Angle, gimbal1AngleRate, gimbal2AngleRate, gimbal1AngularAccel, gimbal2AngularAccel


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
    plt.plot(timespan, motor1AngleData, label=r'$\theta_{1}$')
    plt.plot(timespan, motor2AngleData, label=r'$\theta_{2}$')
    plt.plot(timespan, motor1RefAngleData, '--', label=r'$\theta_{Ref1}$')
    plt.plot(timespan, motor2RefAngleData, '--', label=r'$\theta_{Ref2}$')
    plt.title('Stepper Motor Angles', fontsize=14)
    plt.ylabel('Angle (deg)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='lower right', prop={'size': 16})

    # Plot motor angle rate data
    plt.figure()
    plt.clf()
    plt.plot(timespan, motor1AngleRateData, label=r'$\dot{\theta}_{1}$')
    plt.plot(timespan, motor2AngleRateData, label=r'$\dot{\theta}_{2}$')
    plt.title('Stepper Motor Angle Rates', fontsize=14)
    plt.ylabel('Angle Rate (deg/s)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='lower right', prop={'size': 16})

    # Plot motor acceleration data
    plt.figure()
    plt.clf()
    plt.plot(timespan, motor1AngularAccelData, label=r'$\ddot{\theta}_{1}$')
    plt.plot(timespan, motor2AngularAccelData, label=r'$\ddot{\theta}_{2}$')
    plt.title('Stepper Motor Angular Accelerations', fontsize=14)
    plt.ylabel('Angle Acceleration (deg/s$^2$)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='lower right', prop={'size': 16})

def plotGimbalData(timespan, gimbal1AngleData, gimbal1AngleRateData, gimbal1AngularAccelData, gimbal2AngleData, gimbal2AngleRateData, gimbal2AngularAccelData):

    # Plot gimbal angles
    plt.figure()
    plt.clf()
    plt.plot(timespan, gimbal1AngleData, label=r'$\psi_{1}$')
    plt.plot(timespan, gimbal2AngleData, label=r'$\phi_{2}$')
    plt.title('Interpolated Gimbal Angles', fontsize=14)
    plt.ylabel('Angle (deg)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='center right', prop={'size': 16})

    # Plot gimbal angle rates
    plt.figure()
    plt.clf()
    plt.plot(timespan, gimbal1AngleRateData, label=r'$\dot{\psi}_{1}$')
    plt.plot(timespan, gimbal2AngleRateData, label=r'$\dot{\phi}_{2}$')
    plt.title('Interpolated Gimbal Angle Rates', fontsize=14)
    plt.ylabel('Angle Rate (deg/s)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='center right', prop={'size': 16})

    # Plot gimbal anglular accelerations
    plt.figure()
    plt.clf()
    plt.plot(timespan, gimbal1AngularAccelData, label=r'$\ddot{\psi}_{1}$')
    plt.plot(timespan, gimbal2AngularAccelData, label=r'$\ddot{\phi}_{2}$')
    plt.title('Interpolated Gimbal Angular Accelerations', fontsize=14)
    plt.ylabel('Angle Acceleration (deg/s$^2$)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='center right', prop={'size': 16})


def plotPrescribedGimbalStates(timespan, sigma_FMData, omega_FM_FData, omegaPrime_FM_FData):

    # Plot gimbal attitude
    plt.figure()
    plt.clf()
    plt.plot(timespan, sigma_FMData[:, 0], label=r'$\sigma_{1}$')
    plt.plot(timespan, sigma_FMData[:, 1], label=r'$\sigma_{2}$')
    plt.plot(timespan, sigma_FMData[:, 2], label=r'$\sigma_{3}$')
    plt.title(r'Gimbal Attitude, $\sigma_{\mathcal{F} / \mathcal{M}}$', fontsize=14)
    plt.ylabel('', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='center right', prop={'size': 16})

    # Plot gimbal angle velocity
    plt.figure()
    plt.clf()
    plt.plot(timespan, omega_FM_FData[:, 0], label=r'$\omega_{1}$')
    plt.plot(timespan, omega_FM_FData[:, 1], label=r'$\omega_{2}$')
    plt.plot(timespan, omega_FM_FData[:, 2], label=r'$\omega_{3}$')
    plt.title(r'Gimbal Angular Velocity ${}^{\mathcal{F}} \omega_{\mathcal{F} / \mathcal{M}}$', fontsize=14)
    plt.ylabel('Angular Rate (deg/s)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='center right', prop={'size': 16})

    # Plot gimbal angular acceleration
    plt.figure()
    plt.clf()
    plt.plot(timespan, omegaPrime_FM_FData[:, 0], label=r'$\alpha_{1}$')
    plt.plot(timespan, omegaPrime_FM_FData[:, 1], label=r'$\alpha_{2}$')
    plt.plot(timespan, omegaPrime_FM_FData[:, 2], label=r'$\alpha_{3}$')
    plt.title(r'Gimbal Angular Acceleration ${}^{\mathcal{F}} \alpha_{\mathcal{F} / \mathcal{M}}$', fontsize=14)
    plt.ylabel('Angular Acceleration (deg/s$^2$)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='center right', prop={'size': 16})


if __name__ == "__main__":
    run()
