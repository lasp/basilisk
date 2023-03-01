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
def EOM(t, X, Ih, Ip, k, c, panelThetaRef, panelThetaDotRef):
    # Define the motor control torque for the panel angle and angle rate reference tracking
    u = - k * (X[2] - panelThetaRef) - c * (X[3] - panelThetaDotRef)

    XDot = np.array([[0.0], [0.0], [0.0], [0.0]])
    XDot[0,0] = X[1]
    XDot[1,0] = -u / Ih
    XDot[2,0] = X[3]
    XDot[3,0] = u * ((1 / Ip) + (1 / Ih))

    return XDot

# Call this function to run the simulation
def run(show_plots):
    # Define the mass properties of the rigid spacecraft hub
    mHub = 800.0  # [kg]
    IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 800.0]]  # [kg m^2]
    Ih = IHubPntBc_B[0][0]

    # Define the mass properties of the panel
    mPanel = 100.0  # [kg]
    IPanelPntFc_F = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]  # [kg m^2]
    Ip = IPanelPntFc_F[0][0]
    k = 200.0  # [N-m/rad] Torsional spring constant of hinge
    c = 5.0  # [N-m-s/rad] Rotational damping coefficient for the hinge

    # Define initial states
    hubThetaInit = 0.0 * np.pi / 180.0  # [rad]
    hubThetaDotInit = 0.0 * np.pi / 180.0  # [rad/s]
    panelThetaInit = 0.0 * np.pi / 180.0  # [rad]
    panelThetaDotInit = 0.0 * np.pi / 180.0  # [rad/s]
    X = np.array([[hubThetaInit], [hubThetaDotInit], [panelThetaInit], [panelThetaDotInit]])

    # Define the panel angular reference values
    panelThetaRef = 5.0 * np.pi / 180  # [rad]
    panelThetaDotRef = 0.0  # [rad/s]

    # Create and initialize the total system angular momentum about point B array
    H_sc_B = np.array((Ih + Ip) * hubThetaDotInit + Ip * panelThetaDotInit)

    # Create and initialize the total system energy array
    totEnergy_sc = np.array( (0.5 * Ih * hubThetaDotInit * hubThetaDotInit) + (0.5 * Ip * (panelThetaDotInit + hubThetaDotInit) * (panelThetaDotInit + hubThetaDotInit)) + (0.5 * k * panelThetaInit * panelThetaInit) )

    # Create and initialize the timespan array
    t = 0.0
    timespan = np.array(t)

    # Define quantities for the integration loop
    timeStep = 0.1 # [s]
    t = 0.0
    i = 0

    while (np.abs(X[2,i] - panelThetaRef) > 1e-8 or np.abs(X[3,i] - panelThetaDotRef) > 1e-8):
        # Update the current time
        t = t + timeStep
        timespan = np.append(timespan, t)

        # Store the current state
        XCurrent = np.reshape(X[:,i], (4,1))

        # Numerically integrate the system EOM using RK4 algorithm
        K1 = EOM(t, XCurrent, Ih, Ip, k, c, panelThetaRef, panelThetaDotRef)
        K2 = EOM(t + (timeStep/2), XCurrent + ((timeStep * K1) / 2), Ih, Ip, k, c, panelThetaRef, panelThetaDotRef)
        K3 = EOM(t + (timeStep/2), XCurrent + ((timeStep * K2) / 2), Ih, Ip, k, c, panelThetaRef, panelThetaDotRef)
        K4 = EOM(t + timeStep, XCurrent + timeStep * K3, Ih, Ip, k, c, panelThetaRef, panelThetaDotRef)
        X = np.append(X, XCurrent + (timeStep / 6) * (K1 + 2 * K2 + 2 * K3 + K4), axis=1)

        # Add the current system angular momentum about point B to the storage array
        H_scCurrent_B = np.array((Ih + Ip) * X[1,i+1] + Ip * X[3,i+1])
        H_sc_B = np.append(H_sc_B, H_scCurrent_B)

        # Add the current system energy to the storage array
        totEnergy_scCurrent = np.array( (0.5 * Ih * X[1,i+1] * X[1,i+1]) + (0.5 * Ip * (X[3,i+1] + X[1,i+1]) * (X[3,i+1] + X[1,i+1])) + (0.5 * k * X[2,i+1] * X[2,i+1]) )
        totEnergy_sc = np.append(totEnergy_sc, totEnergy_scCurrent)

        i = i + 1


    # Plot the results
    plt.close("all")  # clears out plots from earlier test runs

    panelThetaData = (180 / np.pi) * X[2,:]
    panelThetaDotData = (180 / np.pi) * X[3,:]
    timespan = (1/60) * timespan

    # Plot the panel angle and angle rate
    fig, plts = plt.subplots(2, 1, sharex=True)
    l1, = plts[0].plot(timespan, panelThetaData)
    l2, = plts[0].plot(timespan, np.ones(len(timespan)) * (panelThetaRef * 180 / np.pi) , '--')
    plts[0].set_title(r'Panel Angle, $\theta$')
    l3, = plts[1].plot(timespan, panelThetaDotData)
    l4, = plts[1].plot(timespan, np.ones(len(timespan)) * (panelThetaDotRef * 180 / np.pi), '--')
    plts[1].set_title(r'Panel Angle Rate, $\dot{\theta}$')
    plts.flat[0].set(ylabel='[deg]')
    plts.flat[1].set(xlabel='Time [min]', ylabel='[deg/s]')
    plts[0].legend((l1, l2), (r'$\theta$', r'$\theta_{Ref}$', ), loc='upper right', prop={'size': 16})
    plts[1].legend((l3, l4), (r'$\dot{\theta}$', r'$\dot{\theta}_{Ref}$', ), loc='upper right', prop={'size': 16})

    # Plot angular momentum about point B difference
    plt.figure()
    plt.clf()
    plt.plot(timespan, H_sc_B - H_sc_B[0])
    plt.title(r'Spacecraft Angular Momentum Difference about Point B, ${}^\mathcal{B} H_{sc, B}(t) - {}^\mathcal{B} H_{sc, B}(t_0)$', fontsize=14)
    plt.ylabel('[Nms]', fontsize=16)
    plt.xlabel('Time [min]', fontsize=16)

    # Plot the system kinetic energy difference
    plt.figure()
    plt.clf()
    plt.plot(timespan, totEnergy_sc - totEnergy_sc[0])
    plt.title(r'Spacecraft Kinetic Energy Difference, $T_{sc}(t) - T_{sc}(t_0)$', fontsize=14)
    plt.ylabel('[Nm]', fontsize=16)
    plt.xlabel('Time [min]', fontsize=16)

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
