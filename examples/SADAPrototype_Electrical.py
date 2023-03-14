#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
# SADA Prototyping
#
# Purpose: This script prototypes the Solar Array Drive Assembly (SADA) model for the MAX mission.
#
#          This simulation sets up a rigid spacecraft hub and a solar panel, both of which are constrained to rotate
#          about the first axis of the spacecraft hub B frame. This results in a 2 DOF rotational simulation. The
#          SADA is modeled with a simple feedback control law with proportional and derivative gains K and C applied
#          to the panel error states.
#
# Author:  Leah Kiner
# Creation Date:  February 27, 2023
#

import os
import matplotlib.pyplot as plt
import numpy as np

# This function defines the system equations of motion
def EOM(t, X, Ih, Ip, panelThetaRef, panelThetaDotRef, V_a, V_b):
    # print(X)
    # Store state variables
    hubAngleRate = X[1]
    panelAngle = X[2]
    panelAngleRate = X[3]
    current_a = X[4]
    current_b = X[5]
    # print("Ia", current_a)
    # print("Ib", current_b)

    # SADA Electrical Parameters
    resistance = 13.0  # [Ohms]
    inductance = 0.002  # [Henrys]
    backEMFAmplitude = 0.53  # [Vs]
    friction = 8e-2 # [N m / Hz]
    numTeeth = 5  # Number of rotor teeth
    stepAngleRad = 18 * np.pi / 180  # [rad]
    numPhases = 4  # Number of stator phases

    # Define back EMFs
    if (current_a == 0.0):
        backEMF_a = 0.0
        backEMF_b = -panelAngleRate * numTeeth * backEMFAmplitude * (panelThetaRef - panelAngle)
        # print("backEMF", backEMF_b)
    else:
        backEMF_a = panelAngleRate * numTeeth * backEMFAmplitude * (panelThetaRef - panelAngle)
        backEMF_b = 0.0

    # Define electromagnetic torque
    T_elec = (current_a * backEMFAmplitude * numTeeth * (panelThetaRef - panelAngle)) + (current_b * backEMFAmplitude * numTeeth * (panelThetaRef - panelAngle))
    # print("Telec:", T_elec)

    XDot = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
    XDot[0,0] = hubAngleRate
    XDot[1,0] = -(T_elec - (friction * (panelAngleRate))) / Ih
    XDot[2,0] = panelAngleRate
    XDot[3,0] = (T_elec - (friction * (panelAngleRate))) * ((1 / Ip) + (1 / Ih))
    XDot[4,0] = (V_a - (resistance * current_a) + backEMF_a) / inductance # IDot_a
    XDot[5,0] = (V_b - (resistance * current_b) + backEMF_b) / inductance # IDot_b
    # print(XDot)

    return XDot, T_elec

# Call this function to run the simulation
def run(show_plots):
    # Define the mass properties of the rigid spacecraft hub
    mHub = 1.0  # [kg]
    Ih = 0.5  # [kg m^2]

    # Define the mass properties of the panel
    mPanel = 0.75  # [kg]
    Ip = 3e-3  # [kg m^2]

    # SADA Electrical Parameters
    resistance = 13.0  # [Ohms]
    inductance = 0.002  # [Henrys]
    backEMFAmplitude = 0.53  # [Vs]
    friction = 8e-4  # [N m / Hz]
    voltageMag = 12  # [Volts]
    numTeeth = 5  # Number of rotor teeth
    stepAngleRad = 18 * np.pi / 180  # [rad]
    numPhases = 4  # Number of stator phases

    if ( (2 * np.pi / (stepAngleRad * numPhases)) != numTeeth):
        print("Unable to configure stepper motor rotor and stator")

    # Define the panel angular reference values
    panelThetaRef = 18 * np.pi / 180  # [rad]
    panelThetaDotRef = 0.0  # [rad/s]

    # Create and initialize the T_elec array
    T_elec = np.array(0.0)

    # Create and initialize the voltage arrays
    V_a = np.array(0.0)
    V_b = np.array(12.0)

    # Define initial states
    hubThetaInit = 0.0 * np.pi / 180.0  # [rad]
    hubThetaDotInit = 0.0 * np.pi / 180.0  # [rad/s]
    panelThetaInit = 0.0 * np.pi / 180.0  # [rad]
    panelThetaDotInit = 0.0 * np.pi / 180.0  # [rad/s]
    current_a_Init = V_a / resistance  # [Amperes]
    current_b_Init = V_b / resistance  # [Amperes]
    X = np.array([[hubThetaInit], [hubThetaDotInit], [panelThetaInit], [panelThetaDotInit], [current_a_Init], [current_b_Init]])

    step = 1

    # Create and initialize the timespan array
    t = 0.0
    timespan = np.array(t)

    # Define quantities for the integration loop
    timeStep = 0.0001 # [s]
    t = 0.0
    i = 0

    while(i < 10000):
    #while (np.abs(X[2,i] - panelThetaRef) > 1e-1 or np.abs(X[3,i] - panelThetaDotRef) > 1e-1):
    #while (np.abs(X[2,i] - panelThetaRef) > 1e-8 or np.abs(X[3,i] - panelThetaDotRef) > 1e-8):
        # Update the current time
        t = t + timeStep
        timespan = np.append(timespan, t)

        # Store the current state
        XCurrent = np.reshape(X[:,i], (6,1))

        if (i != 0):
            Va = V_a[i]
            Vb = V_b[i]
        else:
            Va = V_a
            Vb = V_b

        # Numerically integrate the system EOM using RK4 algorithm
        K1, Telec = EOM(t, XCurrent, Ih, Ip, panelThetaRef, panelThetaDotRef, Va, Vb)
        K2, Telec = EOM(t + (timeStep/2), XCurrent + ((timeStep * K1) / 2), Ih, Ip, panelThetaRef, panelThetaDotRef, Va, Vb)
        K3, Telec = EOM(t + (timeStep/2), XCurrent + ((timeStep * K2) / 2), Ih, Ip, panelThetaRef, panelThetaDotRef, Va, Vb)
        K4, Telec = EOM(t + timeStep, XCurrent + timeStep * K3, Ih, Ip, panelThetaRef, panelThetaDotRef, Va, Vb)
        X = np.append(X, XCurrent + (timeStep / 6) * (K1 + 2 * K2 + 2 * K3 + K4), axis=1)
        T_elec = np.append(T_elec, Telec)
        i = i + 1

        # # Define the voltage for each phase
        # if (panelThetaRef - X[2,i] > 0):
        #     if (step == 1):
        #         V_a = np.append(V_a, 0.0)
        #         V_b = np.append(V_b, 12.0)
        #         X[4,i] = 0.0
        #     elif (step == 2):
        #         V_a = np.append(V_a, -12.0)
        #         V_b = np.append(V_b, 0.0)
        #         X[5,i] = 0.0
        #     elif (step == 3):
        #         V_a = np.append(V_a, 0.0)
        #         V_b = np.append(V_b, -12.0)
        #         X[4,i] = 0.0
        #     else:
        #         V_a = np.append(V_a, 12.0)
        #         V_b = np.append(V_b, 0.0)
        #         X[5,i] = 0.0
        # elif (panelThetaRef - XCurrent[2] < 0):
        #     if (step == 1):
        #         V_a = np.append(V_a, 0.0)
        #         V_b = np.append(V_b, -12.0)
        #         X[4,i] = 0.0
        #     elif (step == 2):
        #         V_a = np.append(V_a, -12.0)
        #         V_b = np.append(V_b, 0.0)
        #         X[5,i] = 0.0
        #     elif (step == 3):
        #         V_a = np.append(V_a, 0.0)
        #         V_b = np.append(V_b, 12.0)
        #         X[4,i] = 0.0
        #     else:
        #         V_a = np.append(V_a, 12.0)
        #         V_b = np.append(V_b, 0.0)
        #         X[5,i] = 0.0

        V_a = np.append(V_a, Va)
        V_b = np.append(V_b, Vb)
        X[4,i] = X[4,i-1]
        X[5,i] = X[5,i-1]

    # Plot the results
    plt.close("all")  # clears out plots from earlier test runs

    panelThetaData = (180 / np.pi) * X[2,:]
    panelThetaDotData = (180 / np.pi) * X[3,:]

    # Plot the panel angle and angle rate
    fig, plts = plt.subplots(2, 1, sharex=True)
    l1, = plts[0].plot(timespan, panelThetaData)
    l2, = plts[0].plot(timespan, np.ones(len(timespan)) * (panelThetaRef * 180 / np.pi) , '--')
    plts[0].set_title(r'Panel Angle, $\theta$')
    l3, = plts[1].plot(timespan, panelThetaDotData)
    l4, = plts[1].plot(timespan, np.ones(len(timespan)) * (panelThetaDotRef * 180 / np.pi), '--')
    plts[1].set_title(r'Panel Angle Rate, $\dot{\theta}$')
    plts.flat[0].set(ylabel='[deg]')
    plts.flat[1].set(xlabel='Time [sec]', ylabel='[deg/s]')
    plts[0].legend((l1, l2), (r'$\theta$', r'$\theta_{Ref}$', ), loc='lower right', prop={'size': 14})
    plts[1].legend((l3, l4), (r'$\dot{\theta}$', r'$\dot{\theta}_{Ref}$', ), loc='upper right', prop={'size': 14})

    # Plot the voltage, current, and torque
    fig, plts = plt.subplots(3, 1, sharex=True)
    l1, = plts[0].plot(timespan, V_a)
    l2, = plts[0].plot(timespan, V_b)
    plts[0].set_title(r'Phase A and B Voltages')
    l3, = plts[1].plot(timespan, X[4,:])
    l4, = plts[1].plot(timespan, X[5,:])
    plts[1].set_title(r'Phase A and B Currents')
    l5, = plts[2].plot(timespan, T_elec)
    plts[2].set_title(r'$T_{elec}$')
    plts.flat[0].set(ylabel='Voltage [v]')
    plts.flat[1].set(ylabel='Current [A]')
    plts.flat[2].set(xlabel='Time [sec]', ylabel='Torque [Nm]')
    plts[0].legend((l1, l2), (r'$V_a$', r'$V_b$', ), loc='lower right', prop={'size': 14})
    plts[1].legend((l3, l4), (r'$I_a$', r'$I_b$', ), loc='center right', prop={'size': 14})

    if show_plots:
        plt.show()

    plt.close("all")


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True  # show_plots
    )
