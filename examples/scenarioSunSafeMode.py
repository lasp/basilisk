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
# EMA Sun Safe Mode Analysis
#
# Purpose:  The purpose of this script is to analyse the EMA spacecraft dynamics under the influence of
#           solar radiation pressure while in the Sun-safe mode configuration. In Sun-safe mode, the
#           spacecraft has both solar arrays facing directly towards the Sun. The spacecraft may rotate
#           about the Sun-line in this configuration.
#
# Authors:  Leah Kiner
# Creation Date:    March 14, 2023
#

import os
import numpy as np
import matplotlib.pyplot as plt
from Basilisk.simulation import (spacecraft, extForceTorque, simpleNav, reactionWheelStateEffector, thrusterDynamicEffector,
                                coarseSunSensor, facetSRPDynamicEffector, spinningBodyOneDOFStateEffector, extForceTorque)
from Basilisk.fswAlgorithms import sunSafePoint, mrpPD, cssWlsEst, thrFiringSchmitt, thrForceMapping, hingedRigidBodyPIDMotor
from Basilisk.architecture import messaging
from Basilisk.utilities import (macros, SimulationBaseClass, unitTestSupport, orbitalMotion, simIncludeGravBody,
                                simIncludeRW, simIncludeThruster, vizSupport)
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])
np.set_printoptions(precision=16)

def run(show_plots):
    # Decide if thrusters should be used for the sim or the extForceTorque module
    useThrusters = True
    numTh = 8

    # Decide if SRP should be added to the sim
    useSRP = True

    # Decide if solar arrays should be added to the sim
    useSolarArrays = True

    # Large 180 initial attitude error
    att180Error = True

    # Spin rate about Sun-line
    spinRate = True

    # Choose the settling time and damping ratios
    settlingTime = 150 * 1  # [s]
    dampingRatios = np.array([1.0, 0.9, 0.85, 0.8])

    # Define and initialize the gains for the MRP PD controller
    PGain = 0.0  # derivative gain on omega_BR
    KGains = np.empty(len(dampingRatios))  # proportional gain on sigma_BR

    if useSolarArrays:
        Pxx = 2028.7401077353311
        Pyy = 5829.0209582175321
        Pzz = 4807.992104939683
    else:
        Pxx = 4995.119267
        Pyy = 8794.038297
        Pzz = 4809.587466

    # Find the best gain P
    P1 = 2 * Pxx / settlingTime
    P2 = 2 * Pyy / settlingTime
    P3 = 2 * Pzz / settlingTime
    T11 = 2 * Pxx / P1
    T12 = 2 * Pyy / P1
    T13 = 2 * Pzz / P1
    T21 = 2 * Pxx / P2
    T22 = 2 * Pyy / P2
    T23 = 2 * Pzz / P2
    T31 = 2 * Pxx / P3
    T32 = 2 * Pyy / P3
    T33 = 2 * Pzz / P3

    if (T11 <= settlingTime and T12 <= settlingTime and T13 <= settlingTime):
        PGain = P1
    elif (T21 <= settlingTime and T22 <= settlingTime and T23 <= settlingTime):
        PGain = P2
    elif (T31 <= settlingTime and T32 <= settlingTime and T33 <= settlingTime):
        PGain = P3
    else:
        print("\nNo optimal gain P could be found")

    # Print Information:
    print("\nSettling Time:")
    print(settlingTime)
    print("Associated P Gain:")
    print(PGain)

    # Find the best gain K for the given damping ratios
    for i in range(len(dampingRatios)):
        dampingRatio = dampingRatios[i]
        K1 = (PGain * PGain) / (dampingRatio * dampingRatio * Pxx)
        K2 = (PGain * PGain) / (dampingRatio * dampingRatio * Pyy)
        K3 = (PGain * PGain) / (dampingRatio * dampingRatio * Pzz)
        damping11 = PGain / np.sqrt(K1 * Pxx)
        damping12 = PGain / np.sqrt(K1 * Pyy)
        damping13 = PGain / np.sqrt(K1 * Pzz)
        damping21 = PGain / np.sqrt(K2 * Pxx)
        damping22 = PGain / np.sqrt(K2 * Pyy)
        damping23 = PGain / np.sqrt(K2 * Pzz)
        damping31 = PGain / np.sqrt(K3 * Pxx)
        damping32 = PGain / np.sqrt(K3 * Pyy)
        damping33 = PGain / np.sqrt(K3 * Pzz)

        if (damping11 <= dampingRatio and damping12 <= dampingRatio and damping13 <= dampingRatio):
            KGains[i] = K1
        elif (damping21 <= dampingRatio and damping22 <= dampingRatio and damping23 <= dampingRatio):
            KGains[i] = K2
        elif (damping31 <= dampingRatio and damping32 <= dampingRatio and damping33 <= dampingRatio):
            KGains[i] = K3
        else:
            print("\nNo optimal gain K could be found")

        # Print Information:
        print("\nDamping Ratio:")
        print(dampingRatio)
        print("Associated K Gain:")
        print(KGains[i])

    timespan = []
    r_bn_n = []
    sigma_bn = []
    rHat_sb_b = []
    sigma_br = []
    omega_br = []
    thrusterForce = []
    thrusterOnTimeReq = []
    pointingErrorAng = []
    theta_bs1 = []
    theta_bs2 = []
    SRPDataForce_b = []
    SRPDataTorque_b = []

    for i in range(len(dampingRatios)):
        timespan, r_bn_n, sigma_bn, sigma_br, omega_br, rHat_sb_b, thrusterForce, thrusterOnTimeReq, pointingErrorAng, theta_bs1, theta_bs2, SRPDataForce_b, SRPDataTorque_b = collectData(useThrusters, useSolarArrays, useSRP, att180Error, spinRate, PGain, KGains[i])
        if i != 0:
            r_bn_n = np.array(r_bn_n)
            sigma_bn = np.array(sigma_bn)
            rHat_sb_b = np.array(rHat_sb_b)
            sigma_br = np.array(sigma_br)
            omega_br = np.array(omega_br)
            thrusterForce = np.array(thrusterForce)
            thrusterOnTimeReq = np.array(thrusterOnTimeReq)
            pointingErrorAng = np.array(pointingErrorAng)
            theta_bs1 = np.array(theta_bs1)
            theta_bs2 = np.array(theta_bs2)
            SRPDataForce_b = np.array(SRPDataForce_b)
            SRPDataTorque_b = np.array(SRPDataTorque_b)

            r_BN_N = np.append(r_BN_N, r_bn_n, axis=1)
            sigma_BN = np.append(sigma_BN, sigma_bn, axis=1)
            rHat_SB_B = np.append(rHat_SB_B, rHat_sb_b, axis=1)
            sigma_BR = np.append(sigma_BR, sigma_br, axis=1)
            omega_BR = np.append(omega_BR, omega_br, axis=1)
            thrForce = np.append(thrForce, thrusterForce, axis=1)
            thrOnTimeRequest = np.append(thrOnTimeRequest, thrusterOnTimeReq, axis=1)
            pointingErrorAngle = np.append(pointingErrorAngle, pointingErrorAng)
            theta_BS1 = np.append(theta_BS1, theta_bs1)
            theta_BS2 = np.append(theta_BS2, theta_bs2)
            SRPDataForce_B = np.append(SRPDataForce_B, SRPDataForce_b, axis=1)
            SRPDataTorque_B = np.append(SRPDataTorque_B, SRPDataTorque_b, axis=1)
        else:
            r_BN_N = np.array(r_bn_n)
            sigma_BN = np.array(sigma_bn)
            rHat_SB_B = np.array(rHat_sb_b)
            sigma_BR = np.array(sigma_br)
            omega_BR = np.array(omega_br)
            thrForce = np.array(thrusterForce)
            thrOnTimeRequest = np.array(thrusterOnTimeReq)
            pointingErrorAngle = np.array(pointingErrorAng)
            theta_BS1 = np.array(theta_bs1)
            theta_BS2 = np.array(theta_bs2)
            SRPDataForce_B = np.array(SRPDataForce_b)
            SRPDataTorque_B = np.array(SRPDataTorque_b)


    # PLOTTING
    scenarioToPlot = 1
    ee = 2.718281828459045  # value of e

    # Remove initial values for plotting clarity
    r_BN_N = r_BN_N[1:]
    sigma_BN = sigma_BN[1:]
    rHat_SB_B = rHat_SB_B[1:]
    sigma_BR = sigma_BR[1:]
    omega_BR = omega_BR[1:]
    thrForce = thrForce[1:]
    thrOnTimeRequest = thrOnTimeRequest[1:]
    pointingErrorAngle = pointingErrorAngle[1:]
    theta_BS1 = theta_BS1[1:]
    theta_BS2 = theta_BS2[1:]
    SRPDataForce_B = SRPDataForce_B[1:]
    SRPDataTorque_B = SRPDataTorque_B[1:]
    timespan = np.array(timespan[1:])

    # Plot the new state error quantity but using magnitudes
    plt.figure()
    plt.clf()
    for idx in range(len(dampingRatios)):
        error = []
        for i in range(len(timespan)):
            e1 = np.sqrt(sigma_BR[i, 3 * idx] * sigma_BR[i, 3 * idx] + omega_BR[i, 3 * idx] * omega_BR[i, 3 * idx])
            e2 = np.sqrt(sigma_BR[i, 3 * idx + 1] * sigma_BR[i, 3 * idx + 1] + omega_BR[i, 3 * idx + 1] * omega_BR[i, 3 * idx + 1])
            e3 = np.sqrt(sigma_BR[i, 3 * idx + 2] * sigma_BR[i, 3 * idx + 2] + omega_BR[i, 3 * idx + 2] * omega_BR[i, 3 * idx + 2])
            err = np.sqrt(e1 * e1 + e2 * e2 + e3 * e3)
            if i != 0:
                error = np.append(error, err)
            else:
                error = np.array(err)
        plt.plot((1/60) * timespan, error, label="P = %f" % PGain + ", K = %f" % KGains[idx] + r", $\zeta$ = %f" % dampingRatios[idx])
        plt.hlines(y=(1 / ee) * error[4], xmin=0, xmax=settlingTime / 60, linestyles='dashed')
        plt.vlines(x=settlingTime / 60, ymin=0, ymax=(1 / ee) * error[4], linestyles='dashed')
        plt.legend(loc='center right')
        plt.title(r'State Error $\epsilon = \sqrt{\epsilon_1^2 + \epsilon_2^2 + \epsilon_3^2}$, T = %i' % settlingTime + " Seconds")
        plt.ylabel(r'$\epsilon$')
        plt.xlabel('Time (min)')
        plt.grid(True)

    # Plot the angles requested to converge such that the spacecraft is in a Sun-facing orientation
    if useSolarArrays:
        fig, plts = plt.subplots(2, 1, sharex=True)
        for idx in range(len(dampingRatios)):
            plts[0].plot((1/60) * timespan, theta_BS1[idx * len(timespan) : idx * len(timespan) + len(timespan)] * (180 / np.pi), label="P = %f" % PGain + ", K = %f" % KGains[idx] + r", $\zeta$ = %f" % dampingRatios[idx])
            plts[1].plot((1/60) * timespan, theta_BS2[idx * len(timespan) : idx * len(timespan) + len(timespan)] * (180 / np.pi), label="P = %f" % PGain + ", K = %f" % KGains[idx] + r", $\zeta$ = %f" % dampingRatios[idx])
        plts[0].legend(loc='center right')
        plts[1].legend(loc='center right')
        plts[0].set_title(r'True Solar Array 1 Sun Incidence Angle')
        plts[1].set_title(r'True Solar Array 2 Sun Incidence Angle')
        plts.flat[0].set(ylabel=r'Angle (deg)')
        plts.flat[1].set(xlabel=r'Time (min)', ylabel=r'Angle (deg)')
        plts[0].grid(True)
        plts[1].grid(True)
    else:
        plt.figure()
        plt.clf()
        for idx in range(len(dampingRatios)):
            plt.plot((1/60) * timespan, pointingErrorAngle[idx * len(timespan) : idx * len(timespan) + len(timespan)] * (180 / np.pi), label="P = %f" % PGain + ", K = %f" % KGains[idx] + r", $\zeta$ = %f" % dampingRatios[idx])
        plt.xlabel(r'Time (min)')
        plt.ylabel(r'Angle (deg)')
        plt.title(r'True Pointing Angle Error')
        plt.legend(loc='center right')
        plt.grid(True)

    # Plot the attitude error
    ee = 2.718281828459045
    fig, plts = plt.subplots(3, 1, sharex=True)
    for idx1 in range(3):
        for idx2 in range(len(dampingRatios)):
            if idx1 == 0:
                plts[idx1].plot((1/60) * timespan, sigma_BR[:, 3 * idx2 + idx1], label="P = %f" % PGain + ", K = %f" % KGains[idx2] + r", $\zeta$ = %f" % dampingRatios[idx2])
                plts[idx1].legend(loc='upper right')
            else:
                plts[idx1].plot((1/60) * timespan, sigma_BR[:, 3 * idx2 + idx1])
            # plts[idx1].hlines(y=(1 / ee) * sigma_BR[4, 3 * idx2 + idx1], xmin=0, xmax=settlingTime / 60, linestyles='dashed')
            # plts[idx1].vlines(x=settlingTime / 60, ymin=min(sigma_BR[:, 3 * idx2 + idx1]), ymax=(1 / ee) * sigma_BR[4, 3 * idx2 + idx1], linestyles='dashed')
    plts[0].set_title(r'Estimated Attitude Error $\sigma_{\mathcal{B} / \mathcal{R}}$, T = %i' % settlingTime + " Seconds")
    plts.flat[0].set(ylabel=r'$\sigma_1$')
    plts.flat[1].set(ylabel=r'$\sigma_2$')
    plts.flat[2].set(xlabel=r'Time (min)', ylabel=r'$\sigma_3$')
    plts[0].grid(True)
    plts[1].grid(True)
    plts[2].grid(True)

    # Plot the angular velocity tracking errors
    fig, plts = plt.subplots(3, 1, sharex=True)
    for idx1 in range(3):
        for idx2 in range(len(dampingRatios)):
            if idx1 == 0:
                plts[idx1].plot((1/60) * timespan, omega_BR[:, 3 * idx2 + idx1], label="P = %f" % PGain + ", K = %f" % KGains[idx2] + r", $\zeta$ = %f" % dampingRatios[idx2])
                plts[idx1].legend(loc='lower right')
            else:
                plts[idx1].plot((1/60) * timespan, omega_BR[:, 3 * idx2 + idx1])
            # plts[idx1].hlines(y=(1 / ee) * omega_BR[4, 3 * idx2 + idx1], xmin=0, xmax=settlingTime / 60, linestyles='dashed')
            # plts[idx1].vlines(x=settlingTime / 60, ymin=min(omega_BR[:, 3 * idx2 + idx1]), ymax=(1 / ee) * omega_BR[4, 3 * idx2 + idx1], linestyles='dashed')
    plts[0].set_title(r'Estimated Angular Velocity Error $\omega_{\mathcal{B} / \mathcal{R}}$, T = %i' % settlingTime + " Seconds")
    plts.flat[0].set(ylabel=r'$\delta \omega_1$')
    plts.flat[1].set(ylabel=r'$\delta \omega_2$')
    plts.flat[2].set(xlabel=r'Time (min)', ylabel=r'$\delta \omega_3$')
    plts[0].grid(True)
    plts[1].grid(True)
    plts[2].grid(True)

    if useThrusters:
        # Plot the thruster force values
        plt.figure()
        plt.clf()
        for idx in range(numTh):
            plt.plot((1/60) * timespan, thrForce[:, idx + 3 * (scenarioToPlot - 1)], label='%d' % idx)
        plt.title('Thruster Force Requested')
        plt.xlabel('Time (min)')
        plt.ylabel('Force (N)')
        plt.legend(loc='lower right', prop={'size': 8})
        plt.grid(True)

        # Plot the thruster on time requests
        plt.figure()
        plt.clf()
        for idx in range(numTh):
            plt.plot((1/60) * timespan, thrOnTimeRequest[:, idx + 3 * (scenarioToPlot - 1)], label="%d" % idx)
        plt.title('Thruster On-Time Requests')
        plt.xlabel('Time (min)')
        plt.ylabel('Time (sec)')
        plt.legend(loc='lower right', prop={'size': 8})
        plt.grid(True)

    # Plot the SRP force and torque acting on the spacecraft
    if useSRP:
        plt.figure()
        plt.clf()
        plt.plot((1/60) * timespan, SRPDataForce_B[:, 3 * (scenarioToPlot - 1)], label=r'$F \cdot \hat{b}_1$')
        plt.plot((1/60) * timespan, SRPDataForce_B[:, 1 + 3 * (scenarioToPlot - 1)], label=r'$F \cdot \hat{b}_2$')
        plt.plot((1/60) * timespan, SRPDataForce_B[:, 2 + 3 * (scenarioToPlot - 1)], label=r'$F \cdot \hat{b}_3$')
        plt.title(r'SRP Force ${}^\mathcal{B} F_{SRP, B}$')
        plt.xlabel(r'Time (min)')
        plt.ylabel(r'Force (N)')
        plt.legend(loc='center right', prop={'size': 12})
        plt.grid(True)

        plt.figure()
        plt.clf()
        plt.plot((1/60) * timespan, SRPDataTorque_B[:, 3 * (scenarioToPlot - 1)], label=r'$L \cdot \hat{b}_1$')
        plt.plot((1/60) * timespan, SRPDataTorque_B[:, 1 + 3 * (scenarioToPlot - 1)], label=r'$L \cdot \hat{b}_2$')
        plt.plot((1/60) * timespan, SRPDataTorque_B[:, 2 + 3 * (scenarioToPlot - 1)], label=r'$L \cdot \hat{b}_3$')
        plt.title(r'SRP Torque ${}^\mathcal{B} L_{SRP, B}$')
        plt.xlabel(r'Time (min)')
        plt.ylabel(r'Torque (Nm)')
        plt.legend(loc='center right', prop={'size': 12})
        plt.grid(True)

    # Plot the spacecraft inertial position
    plt.figure()
    plt.clf()
    plt.plot((1/60) * timespan, r_BN_N[:, 3 * (scenarioToPlot - 1)], label=r'$r_{B/N} \cdot \hat{n}_1$')
    plt.plot((1/60) * timespan, r_BN_N[:, 1 + 3 * (scenarioToPlot - 1)], label=r'$r_{B/N} \cdot \hat{n}_2$')
    plt.plot((1/60) * timespan, r_BN_N[:, 2 + 3 * (scenarioToPlot - 1)], label=r'$r_{B/N} \cdot \hat{n}_3$')
    plt.title(r'Spacecraft Inertial Position ${}^\mathcal{N} r_{B/N}$')
    plt.xlabel(r'Time (min)')
    plt.ylabel(r'Position (m)')
    plt.legend(loc='center right', prop={'size': 12})
    plt.grid(True)

    # Plot the spacecraft attitude
    plt.figure()
    plt.clf()
    plt.plot((1/60) * timespan, sigma_BN[:, 3 * (scenarioToPlot - 1)], label=r'$\sigma_1$')
    plt.plot((1/60) * timespan, sigma_BN[:, 1 + 3 * (scenarioToPlot - 1)], label=r'$\sigma_2$')
    plt.plot((1/60) * timespan, sigma_BN[:, 2 + 3 * (scenarioToPlot - 1)], label=r'$\sigma_3$')
    plt.title(r'Spacecraft Inertial Attitude $\sigma_{\mathcal{B} / \mathcal{N}}$')
    plt.xlabel(r'Time (min)')
    plt.ylabel(r'MRP Components')
    plt.legend(loc='center right', prop={'size': 12})
    plt.grid(True)

    # Plot the Sun direction in body-frame coordinates
    commandedVec = [0.0, np.cos(20 * np.pi / 180), -np.sin(20 * np.pi / 180)]
    pos1Ref = np.ones(len(timespan)) * commandedVec[0]
    pos2Ref = np.ones(len(timespan)) * commandedVec[1]
    pos3Ref = np.ones(len(timespan)) * commandedVec[2]

    plt.figure()
    plt.clf()
    plt.plot((1/60) * timespan, rHat_SB_B[:, 3 * (scenarioToPlot - 1)], label=r'$r_{S/B} \cdot \hat{b}_1$', color='red')
    plt.plot((1/60) * timespan, pos1Ref, '--', color='red')
    plt.plot((1/60) * timespan, rHat_SB_B[:, 1 + 3 * (scenarioToPlot - 1)], label=r'$r_{S/B} \cdot \hat{b}_2$', color='blue')
    plt.plot((1/60) * timespan, pos2Ref, '--', color='blue')
    plt.plot((1/60) * timespan, rHat_SB_B[:, 2 + 3 * (scenarioToPlot - 1)], label=r'$r_{S/B} \cdot \hat{b}_3$', color='green')
    plt.plot((1/60) * timespan, pos3Ref, '--', color='green')
    plt.title(r'True Sun Direction ${}^\mathcal{B} r_{S/B}$ (m)')
    plt.xlabel(r'Time (min)')
    plt.ylabel(r'$\mathcal{B}$-Frame Components')
    plt.legend(loc='center right', prop={'size': 12})
    plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")


def collectData(useThrusters, useSolarArrays, useSRP, att180Error, spinRate, PGain, KGain):
    # Create the simulation variable names
    fswTask = "fswTask"
    dynTask = "dynTask"
    simProcessName = "simProcess"

    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    scSim.SetProgressBar(True)

    # Create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # Create the dynamics task and specify the simulation time and integration update time
    simulationTime = macros.min2nano(30)
    simulationTimeStepFsw = macros.sec2nano(0.1)
    simulationTimeStepDyn = macros.sec2nano(0.01)
    dynProcess.addTask(scSim.CreateNewTask(fswTask, simulationTimeStepFsw))
    dynProcess.addTask(scSim.CreateNewTask(dynTask, simulationTimeStepDyn))

    # Initialize the spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "Max-SC"
    scSim.AddModelToTask(dynTask, scObject, None, 1)

    # Set up the simulation gravitational bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(['sun'])
    sun = gravBodies['sun']
    sun.isCentralBody = True
    mu = sun.mu
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Create and configure the SPICE support module
    timeInitString = "2022 JUNE 27 00:00:00.0"
    gravFactory.createSpiceInterface(bskPath +'/supportData/EphemerisData/',
                                     timeInitString,
                                     epochInMsg=True)

    # Set the Sun as the zero base
    gravFactory.spiceObject.zeroBase = 'Sun'

    # Add the SPICE object to the simulation task list.
    scSim.AddModelToTask(fswTask, gravFactory.spiceObject, None, 2)

    # Connect the gravitational bodies to the spacecraft object
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Set up the spacecraft orbit
    oe = orbitalMotion.ClassicElements()
    AU2Meters = 1.4959787e11
    case1 = 0.7 * AU2Meters
    case2 = 1 * AU2Meters
    case3 = 2.3 * AU2Meters
    oe.a = case1  # meters
    oe.e = 0.001
    oe.i = 0.0 * macros.D2R
    oe.Omega = 0.0 * macros.D2R
    oe.omega = 0.0 * macros.D2R
    if att180Error == True:
        oe.f = 30.0 * macros.D2R
    else:
        oe.f = -83.0 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # [m] - r_BN_N
    scObject.hub.v_CN_NInit = vN  # [m/s] - v_BN_N
    scObject.hub.sigma_BNInit = [0.0, 0.0, 0.0]  # initial MRP inertial attitude
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # [rad/s] - omega_CN_B

    # Define the MAX spacecraft model
    scMass = 2215.09816  # [kg]
    massSolarArray = 82.79  # [kg]

    if useSolarArrays:
        scObject.hub.mHub = scMass - 2 * massSolarArray
        r_BcB_B = [[7.84713085978795 / 1000], [-9.966952366 / 1000], [1214.854881 / 1000]]  # [m] - position vector of hub CM relative to the body-fixed point B
        Pxx = 1531.396396995112  # [kg m^2]
        Pyy = 2610.4239755468516  # [kg m^2]
        Pzz = 1998.446530952034  # [kg m^2]
        Pxy = -5.085218765292817  # [kg m^2]
        Pxz = 7.866648194001868  # [kg m^2]
        Pyz = 79.00993258422233  # [kg m^2]
    else:
        scObject.hub.mHub = scMass
        r_BcB_B = [0.007260552824003,  0.0258361327545087, 1.1569339845301208]  # [m] - position vector of hub CM relative to the body-fixed point B
        Pxx = 4995.119267  # [kg m^2]
        Pyy = 8794.038297  # [kg m^2]
        Pzz = 4809.587466  # [kg m^2]
        Pxy = -4.924921893  # [kg m^2]
        Pxz = -33.08314604  # [kg m^2]
        Pyz = 69.65723097  # [kg m^2]

    IHubPntBc_B = [Pxx,  Pxy,  Pxz,
                   Pxy,  Pyy,  Pyz,
                   Pxz,  Pyz,  Pzz]

    scObject.hub.r_BcB_B = r_BcB_B
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(IHubPntBc_B)

    inch2Meter = 0.0254  # [meters in an inch]

    if useSolarArrays:
        # Create the solar arrays
        solarArrayList = []

        # Define the first solar array
        solarArrayList.append(spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector())
        solarArrayList[0].r_SB_B = [0.5 * 1.53, 0.0, 0.44]
        solarArrayList[0].r_ScS_S = [-0.036, 2.827, -0.469]
        solarArrayList[0].sHat_S = [0, 1, 0]
        solarArrayList[0].dcm_S0B = [[0, 0, -1], [1, 0, 0], [0, -1, 0]]
        solarArrayList[0].IPntSc_S = [[319.0, 0.0, 0.0],
                                      [0.0, 185.0, 0.0],
                                      [0.0, 0.0, 495.0]]
        solarArrayList[0].mass = massSolarArray
        solarArrayList[0].k = 100
        solarArrayList[0].c = 100
        solarArrayList[0].thetaInit = -20 * np.pi / 180  # [rad]
        solarArrayList[0].thetaDotInit = 0
        solarArrayList[0].ModelTag = "solarArray0"
        scObject.addStateEffector(solarArrayList[0])
        scSim.AddModelToTask(dynTask, solarArrayList[0])

        # Define the second solar array
        solarArrayList.append(spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector())
        solarArrayList[1].r_SB_B = [-0.5 * 1.53, 0.0, 0.44]
        solarArrayList[1].r_ScS_S = [-0.036, 2.827, -0.469]
        solarArrayList[1].sHat_S = [0, 1, 0]
        solarArrayList[1].dcm_S0B = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        solarArrayList[1].IPntSc_S = [[319.0, 0.0, 0.0],
                                      [0.0, 185.0, 0.0],
                                      [0.0, 0.0, 495.0]]
        solarArrayList[1].mass = massSolarArray
        solarArrayList[1].k = 100
        solarArrayList[1].c = 100
        solarArrayList[1].thetaInit = 20 * np.pi / 180  # [rad]
        solarArrayList[1].thetaDotInit = 0
        solarArrayList[1].ModelTag = "solarArray1"
        scObject.addStateEffector(solarArrayList[1])
        scSim.AddModelToTask(dynTask, solarArrayList[1])

        # Create the locking message
        lockArray = messaging.ArrayEffectorLockMsgPayload()
        lockArray.effectorLockFlag = [1]
        lockMsg = messaging.ArrayEffectorLockMsg().write(lockArray)
        solarArrayList[0].motorLockInMsg.subscribeTo(lockMsg)
        solarArrayList[1].motorLockInMsg.subscribeTo(lockMsg)

        # Create the solar array reference message
        solarArrayMsgData1 = messaging.HingedRigidBodyMsgPayload()
        solarArrayMsgData1.theta = -20 * np.pi / 180
        solarArrayMsgData1.thetaDot = 0.0
        solarArrayMessage1 = messaging.HingedRigidBodyMsg().write(solarArrayMsgData1)

        solarArrayMsgData2 = messaging.HingedRigidBodyMsgPayload()
        solarArrayMsgData2.theta = 20 * np.pi / 180
        solarArrayMsgData2.thetaDot = 0.0
        solarArrayMessage2 = messaging.HingedRigidBodyMsg().write(solarArrayMsgData2)

        # Create the hinged rigid body motor control modules
        # Control module for array 1
        motorConfig1 = hingedRigidBodyPIDMotor.hingedRigidBodyPIDMotorConfig()
        motorWrap1 = scSim.setModelDataWrap(motorConfig1)
        motorWrap1.ModelTag = "hingedRigidBodyPIDMotor1"
        motorConfig1.K = 5.0
        motorConfig1.P = 5
        motorConfig1.I = 5
        scSim.AddModelToTask(fswTask, motorWrap1, motorConfig1)

        # Control module for array 2
        motorConfig2 = hingedRigidBodyPIDMotor.hingedRigidBodyPIDMotorConfig()
        motorWrap2 = scSim.setModelDataWrap(motorConfig2)
        motorWrap2.ModelTag = "hingedRigidBodyPIDMotor2"
        motorConfig2.K = 5.0
        motorConfig2.P = 5
        motorConfig2.I = 5
        scSim.AddModelToTask(fswTask, motorWrap2, motorConfig2)

    if useThrusters:
        # Create the thrusters
        # Create the set of thrusters in the dynamics task
        thrusterSet = thrusterDynamicEffector.ThrusterDynamicEffector()
        scSim.AddModelToTask(dynTask, thrusterSet)

        # Make a fresh thruster factory instance, this is critical to run multiple times
        thFactory = simIncludeThruster.thrusterFactory()

        # Create arrays for thruster locations and directions
        a = 38.480 * inch2Meter
        b = 35.781 * inch2Meter
        c = 35.531 * inch2Meter
        d = 78.119 * inch2Meter
        e = 7.681 * inch2Meter
        location = [
            [ a,  b,  d],
            [ a,  c,  e],
            [-a,  b,  d],
            [-a,  c,  e],
            [-a, -b,  d],
            [-a, -c,  e],
            [ a, -b,  d],
            [ a, -c,  e]
        ]
        direction = [
            [0,   0.707,  -0.707],
            [0,   0.707,   0.707],
            [0,   0.707,  -0.707],
            [0,   0.707,   0.707],
            [0,  -0.707,  -0.707],
            [0,  -0.707,   0.707],
            [0,  -0.707,  -0.707],
            [0,  -0.707,   0.707]
        ]

        # Create the thruster devices by specifying the thruster type and its location and direction
        BOLNominalThrust = 4.893  # [N]
        MOLNominalThrust = 3.571  # [N]
        EOLNominalThrust = 1.756  # [N]
        for pos_B, dir_B in zip(location, direction):
            thFactory.create('MOOG_Monarc_5', pos_B, dir_B, MaxThrust=BOLNominalThrust)

        # Get the number of thruster devices
        numTh = thFactory.getNumOfDevices()

        # Create the thruster object container and tie to the spacecraft object
        thrModelTag = "ACSThrusterDynamics"
        thFactory.addToSpacecraft(thrModelTag, thrusterSet, scObject)

        # Set up the FSW thruster configuration message
        fswThrConfigMsg = thFactory.getConfigMessage()

    # Add the simple Navigation sensor module
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(dynTask, sNavObject)

    if useSRP:
        # Create an instance of the facetSRPDynamicEffector module
        newSRP = facetSRPDynamicEffector.FacetSRPDynamicEffector()
        newSRP.ModelTag = "FacetSRP"
        scObject.addDynamicEffector(newSRP)
        scSim.AddModelToTask(dynTask, newSRP)

        # Define the spacecraft geometry for populating the FacetedSRPSpacecraftGeometryData structure in the SRP module
        # Define the facet surface areas
        lenXHub = 1.53  # [m]
        lenYHub = 1.8  # [m]
        lenZHub = 2.86  # [m]
        area2 = np.pi*(0.5 * 7.262)*(0.5 * 7.262)  # [m^2]
        facetAreas = [lenYHub * lenZHub, lenXHub * lenZHub, lenYHub * lenZHub, lenXHub * lenZHub, lenXHub * lenYHub, lenXHub * lenYHub, area2, area2, area2, area2]

        # Define the facet normals in B frame components
        facetNormal1 = np.array([1.0, 0.0, 0.0])
        facetNormal2 = np.array([0.0, 1.0, 0.0])
        facetNormal3 = np.array([-1.0, 0.0, 0.0])
        facetNormal4 = np.array([0.0, -1.0, 0.0])
        facetNormal5 = np.array([0.0, 0.0, 1.0])
        facetNormal6 = np.array([0.0, 0.0, -1.0])
        facetNormal7 = np.array([0.0, 1.0, 0.0])
        facetNormal8 = np.array([0.0, -1.0, 0.0])
        facetNormal9 = np.array([0.0, 1.0, 0.0])
        facetNormal10 = np.array([0.0, -1.0, 0.0])
        normals_B = [facetNormal1, facetNormal2, facetNormal3, facetNormal4, facetNormal5, facetNormal6, facetNormal7, facetNormal8, facetNormal9, facetNormal10]

        # Define the facet center of pressure locations with respect to point B in B frame components
        facetLoc1 = np.array([0.5 * lenXHub, 0.0, 0.5 * lenZHub])  # [m]
        facetLoc2 = np.array([0.0, 0.5 * lenYHub, 0.5 * lenZHub])  # [m]
        facetLoc3 = np.array([-0.5 * lenXHub, 0.0, 0.5 * lenZHub])  # [m]
        facetLoc4 = np.array([0.0, -0.5 * lenYHub, 0.5 * lenZHub])  # [m]
        facetLoc5 = np.array([0.0, 0.0, lenZHub])  # [m]
        facetLoc6 = np.array([0.0, 0.0, 0.0])  # [m]
        facetLoc7 = np.array([3.75 + 0.5 * lenXHub, 0.544, 0.44])  # [m]
        facetLoc8 = np.array([3.75 + 0.5 * lenXHub, 0.544, 0.44])  # [m]
        facetLoc9 = np.array([-(3.75 + 0.5 * lenXHub), 0.544, 0.44])  # [m]
        facetLoc10 = np.array([-(3.75 + 0.5 * lenXHub), 0.544, 0.44])  # [m]
        locationsPntB_B = [facetLoc1, facetLoc2, facetLoc3, facetLoc4, facetLoc5, facetLoc6, facetLoc7, facetLoc8, facetLoc9, facetLoc10]

        # Define the facet optical coefficients
        specCoeff = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        diffCoeff = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

        # Populate the FacetedSRPSpacecraftGeometryData structure with the facet information
        for i in range(len(facetAreas)):
            newSRP.addFacet(facetAreas[i], specCoeff[i], diffCoeff[i], normals_B[i], locationsPntB_B[i])

    # Create the Course Sun Sensors
    # Create arrays for CSS' locations and directions
    a = 1
    b = 1.28
    c = 2**(-0.5)
    location = [
        [-a, -a, -b],
        [ a, -a,  b],
        [ a, -a, -b],
        [ a,  a,  b],
        [ a,  a, -b],
        [-a,  a,  b],
        [-a,  a, -b],
        [-a, -a,  b],
    ]
    direction = [
        [0.70710678118654746, -0.5, 0.5],
        [0.70710678118654746, -0.5, -0.5],
        [0.70710678118654746, 0.5, -0.5],
        [0.70710678118654746, 0.5, 0.5],
        [-0.70710678118654746, 0, 0.70710678118654757],
        [-0.70710678118654746, 0.70710678118654757, 0.0],
        [-0.70710678118654746, 0, -0.70710678118654757],
        [-0.70710678118654746, -0.70710678118654757, 0.0]
    ]

    # Instantiate coarse sun sensors
    cssList = []
    ii = 0
    for pos_B, dir_B in zip(location, direction):
        cssList.append(coarseSunSensor.CoarseSunSensor())
        cssList[ii].ModelTag = "CSS" + str(ii+1)
        cssList[ii].fov = 80. * macros.D2R
        # cssList[ii].scaleFactor = 2.0
        # cssList[ii].maxOutput = 2.0
        # cssList[ii].minOutput = 0.5
        cssList[ii].r_B = pos_B
        cssList[ii].nHat_B = dir_B
        cssList[ii].sunInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[0])
        cssList[ii].stateInMsg.subscribeTo(scObject.scStateOutMsg)
        ii += 1

    cssArray = coarseSunSensor.CSSConstellation()
    cssArray.ModelTag = "css_array"
    cssArray.sensorList = coarseSunSensor.CSSVector(cssList)
    scSim.AddModelToTask(dynTask, cssArray)

    # Create the CSS config msg
    cssConfig = messaging.CSSConfigMsgPayload()
    totalCSSList = []
    for CSSHat in direction:
        newCSS = messaging.CSSUnitConfigMsgPayload()
        newCSS.nHat_B = CSSHat
        newCSS.CBias = 1.0
        totalCSSList.append(newCSS)
    cssConfig.nCSS = len(direction)
    cssConfig.cssVals = totalCSSList

    cssConstMsg = messaging.CSSConfigMsg().write(cssConfig)

    # Set up least-squares estimator for CSS measurements
    cssLSConfig = cssWlsEst.CSSWLSConfig()
    cssLSWrap = scSim.setModelDataWrap(cssLSConfig)
    cssLSWrap.ModelTag = "css_ls"
    scSim.AddModelToTask(fswTask, cssLSWrap, cssLSConfig)

    # Create an instance of the sunSafePoint module
    sunPointConfig = sunSafePoint.sunSafePointConfig()
    sunPointConfigWrap = scSim.setModelDataWrap(sunPointConfig)
    sunPointConfigWrap.ModelTag = "sunSafePoint"
    scSim.AddModelToTask(fswTask, sunPointConfigWrap, sunPointConfig)
    sunPointConfig.minUnitMag = 0.0
    sunPointConfig.sunAngleErr = 0.0  # [rad]
    sunPointConfig.smallAngle = 0.0  # [rad]
    # sunPointConfig.eHat180_B = np.array([1.0, 0.0, 0.0])
    # sunPointConfig.sunMnvrVec = np.array([0.0, 0.0, 0.0])
    sunPointConfig.sHatBdyCmd = np.array([0.0, np.cos(20 * np.pi / 180), -np.sin(20 * np.pi / 180)]) # np.array([0.0, 1.0, 0.0])
    sunPointConfig.omega_RN_B = np.array([0.0, 0.0, 0.0])
    if spinRate == True:
        sunPointConfig.sunAxisSpinRate = 0.01  # [rad/s]
    else:
        sunPointConfig.sunAxisSpinRate = 0.0  # [rad/s]

    # Create the vehicle configuration message for the MRP PD module
    vehConfigData = messaging.VehicleConfigMsgPayload()
    vehConfigData.ISCPntB_B = IHubPntBc_B
    vcMsg = messaging.VehicleConfigMsg().write(vehConfigData)

    # Create an instance of the mrpPD module
    mrpControlConfig = mrpPD.MrpPDConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "mrpPD"
    scSim.AddModelToTask(fswTask, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.K = KGain
    mrpControlConfig.P = PGain
    mrpControlConfig.knownTorquePntB_B = np.array([0.0, 0.0, 0.0])

    if useThrusters:
        # Set up the thruster force mapping module
        thrForceMappingConfig = thrForceMapping.thrForceMappingConfig()
        thrForceMappingWrap = scSim.setModelDataWrap(thrForceMappingConfig)
        thrForceMappingWrap.ModelTag = "thrForceMapping"
        scSim.AddModelToTask(fswTask, thrForceMappingWrap, thrForceMappingConfig)
        controlAxes_B = [1, 0, 0,
                         0, 1, 0,
                         0, 0, 1]
        thrForceMappingConfig.thrForceSign = +1
        thrForceMappingConfig.controlAxes_B = controlAxes_B

        # Set up the Schmitt trigger thruster firing logic module
        thrFiringSchmittConfig = thrFiringSchmitt.thrFiringSchmittConfig()
        thrFiringSchmittWrap = scSim.setModelDataWrap(thrFiringSchmittConfig)
        thrFiringSchmittWrap.ModelTag = "thrFiringSchmitt"
        scSim.AddModelToTask(fswTask, thrFiringSchmittWrap, thrFiringSchmittConfig)
        thrFiringSchmittConfig.thrMinFireTime = 0.0001
        thrFiringSchmittConfig.level_on = .75
        thrFiringSchmittConfig.level_off = .25
        thrusterSet.cmdsInMsg.subscribeTo(thrFiringSchmittConfig.onTimeOutMsg)
        thrForceMappingConfig.cmdTorqueInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)
        thrForceMappingConfig.thrConfigInMsg.subscribeTo(fswThrConfigMsg)
        thrForceMappingConfig.vehConfigInMsg.subscribeTo(vcMsg)
        thrFiringSchmittConfig.thrConfInMsg.subscribeTo(fswThrConfigMsg)
        thrFiringSchmittConfig.thrForceInMsg.subscribeTo(thrForceMappingConfig.thrForceCmdOutMsg)
    else:
        # setup extForceTorque module
        extFTObject = extForceTorque.ExtForceTorque()
        extFTObject.ModelTag = "externalDisturbance"
        scObject.addDynamicEffector(extFTObject)
        scSim.AddModelToTask(dynTask, extFTObject)
        extFTObject.cmdTorqueInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)

    # Link all messages
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    cssLSConfig.cssDataInMsg.subscribeTo(cssArray.constellationOutMsg)
    cssLSConfig.cssConfigInMsg.subscribeTo(cssConstMsg)
    sunPointConfig.sunDirectionInMsg.subscribeTo(cssLSConfig.navStateOutMsg)
    sunPointConfig.imuInMsg.subscribeTo(sNavObject.attOutMsg)
    mrpControlConfig.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControlConfig.guidInMsg.subscribeTo(sunPointConfig.attGuidanceOutMsg)

    if useSolarArrays:
        solarArrayList[0].motorTorqueInMsg.subscribeTo(motorConfig1.motorTorqueOutMsg)
        solarArrayList[1].motorTorqueInMsg.subscribeTo(motorConfig2.motorTorqueOutMsg)
        motorConfig1.hingedRigidBodyInMsg.subscribeTo(solarArrayList[0].spinningBodyOutMsg)
        motorConfig1.hingedRigidBodyRefInMsg.subscribeTo(solarArrayMessage1)
        motorConfig2.hingedRigidBodyInMsg.subscribeTo(solarArrayList[1].spinningBodyOutMsg)
        motorConfig2.hingedRigidBodyRefInMsg.subscribeTo(solarArrayMessage2)
    if useSRP:
        newSRP.sunInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[0])

    # Set up data logging before the simulation is initialized
    numDataPoints = 10000
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStepDyn, numDataPoints)
    scPosDataLog = scObject.scStateOutMsg.recorder(samplingTime)
    sunDataLog = gravFactory.spiceObject.planetStateOutMsgs[0].recorder(samplingTime)
    sNavRec = sNavObject.attOutMsg.recorder(samplingTime)
    dataRec = scObject.scStateOutMsg.recorder(samplingTime)
    scMassProps = scObject.scMassOutMsg.recorder(samplingTime)
    sunLineLog = cssLSConfig.navStateOutMsg.recorder(samplingTime)
    attErrorLog = sunPointConfig.attGuidanceOutMsg.recorder(samplingTime)
    torqueRequest = mrpControlConfig.cmdTorqueOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, scPosDataLog)
    scSim.AddModelToTask(dynTask, sunDataLog)
    scSim.AddModelToTask(dynTask, sNavRec)
    scSim.AddModelToTask(dynTask, dataRec)
    scSim.AddModelToTask(dynTask, scMassProps)
    scSim.AddModelToTask(dynTask, sunLineLog)
    scSim.AddModelToTask(dynTask, attErrorLog)
    scSim.AddModelToTask(dynTask, torqueRequest)

    if useSolarArrays:
        solarArray1Log = solarArrayList[0].spinningBodyConfigLogOutMsg.recorder(samplingTime)
        solarArray2Log = solarArrayList[1].spinningBodyConfigLogOutMsg.recorder(samplingTime)
        scSim.AddModelToTask(dynTask, solarArray1Log)
        scSim.AddModelToTask(dynTask, solarArray2Log)
    if useSRP:
        scSim.AddVariableForLogging(newSRP.ModelTag + ".forceExternal_B", samplingTime, 0, 2, 'double')
        scSim.AddVariableForLogging(newSRP.ModelTag + ".torqueExternalPntB_B", samplingTime, 0, 2, 'double')
    if useThrusters:
        thrMapLog = thrForceMappingConfig.thrForceCmdOutMsg.recorder(samplingTime)
        thrTrigLog = thrFiringSchmittConfig.onTimeOutMsg.recorder(samplingTime)
        scSim.AddModelToTask(dynTask, thrMapLog)
        scSim.AddModelToTask(dynTask, thrTrigLog)

    # Add optional Vizard visualization
    vizSupport.enableUnityVisualization(scSim, dynTask, scObject
                                        #, saveFile=fileName
                                        )

    # Run the simulation
    scSim.InitializeSimulation()

    # Configure the simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Retrieve the logged data
    timespan = scPosDataLog.times() * 1e-9
    r_BN_N = scPosDataLog.r_BN_N
    r_SN_N = sunDataLog.PositionVector
    sigma_BN = sNavRec.sigma_BN
    rHatEst_SB_B = sunLineLog.vehSunPntBdy
    sigma_BR = attErrorLog.sigma_BR
    omega_BR = attErrorLog.omega_BR_B
    ISC_PntB_B = scMassProps.ISC_PntB_B

    if useSolarArrays:
        sigma_S1N = solarArray1Log.sigma_BN
        sigma_S2N = solarArray2Log.sigma_BN

    if useSRP:
        SRPDataForce_B = scSim.GetLogVariableData(newSRP.ModelTag + ".forceExternal_B")
        SRPDataTorque_B = scSim.GetLogVariableData(newSRP.ModelTag + ".torqueExternalPntB_B")
        SRPDataForce_B = SRPDataForce_B
        SRPDataForce_B = np.delete(SRPDataForce_B, 0, axis=1)
        SRPDataTorque_B = SRPDataTorque_B
        SRPDataTorque_B = np.delete(SRPDataTorque_B, 0, axis=1)
    else:
        SRPDataForce_B = np.zeros((len(timespan), 3))
        SRPDataTorque_B = np.zeros((len(timespan), 3))

    if useThrusters:
        thrForce = thrMapLog.thrForce
        thrForce = thrForce[:, 0:8]
        thrOnTimeRequest = thrTrigLog.OnTimeRequest
        thrOnTimeRequest = thrOnTimeRequest[:, 0:8]
    else:
        thrForce = np.zeros((len(timespan), 8))
        thrOnTimeRequest = np.zeros((len(timespan), 8))

    # Calculate data for plotting
    pointingErrorAngle = []
    rHatTrue_SB_B = []

    for i in range(len(timespan)):
        # Compute DCMs
        dcm_BN = rbk.MRP2C(sigma_BN[i])

        # Calculate the unit vector pointing from the B frame origin to the Sun, rHatTrue_SB_B
        r_BN_B = np.matmul(dcm_BN, r_BN_N[i])
        r_SN_B = np.matmul(dcm_BN, r_SN_N[i])
        r_SB_B = r_SN_B - r_BN_B
        rHatTrue_sb_b = r_SB_B / np.linalg.norm(r_SB_B)

        # Calculate the angle between the Sun-line and the body-frame vector commanded to point towards the sun
        pointErrorAngle = np.arccos( np.dot(sunPointConfig.sHatBdyCmd, rHatTrue_sb_b) / (np.linalg.norm(sunPointConfig.sHatBdyCmd) * np.linalg.norm(rHatTrue_sb_b)) )

        # Store the data into arrays
        pointingErrorAngle.append(pointErrorAngle)
        rHatTrue_SB_B.append(rHatTrue_sb_b)

    rHatTrue_SB_B = np.array(rHatTrue_SB_B)
    pointingErrorAngle = np.array(pointingErrorAngle)

    if useSolarArrays:
        theta_BS1 = []
        theta_BS2 = []

        for i in range(len(timespan)):
            # Compute DCMs
            dcm_BN = rbk.MRP2C(sigma_BN[i])
            dcm_S1N = rbk.MRP2C(sigma_S1N[i])
            dcm_S2N = rbk.MRP2C(sigma_S2N[i])
            dcm_BS1 = np.matmul(dcm_BN, np.transpose(dcm_S1N))
            dcm_BS2 = np.matmul(dcm_BN, np.transpose(dcm_S2N))

            # Calculate the unit vector pointing from the B frame origin to the Sun, rHatTrue_SB_B
            r_BN_B = np.matmul(dcm_BN, r_BN_N[i])
            r_SN_B = np.matmul(dcm_BN, r_SN_N[i])
            r_SB_B = r_SN_B - r_BN_B
            rHatTrue_sb_b = r_SB_B / np.linalg.norm(r_SB_B)

            # Calculate the angle between the array normal vectors and the body-frame vector bHat2 commanded to point
            # towards the sun
            rHat_SB_S1 = np.matmul(rHatTrue_sb_b, dcm_BS1)
            rHat_SB_S2 = np.matmul(rHatTrue_sb_b, dcm_BS2)
            theta_bs1 = np.arccos(np.dot([0, 0, -1], rHat_SB_S1))
            theta_bs2 = np.arccos(np.dot([0, 0, -1], rHat_SB_S2))

            # Store the data into arrays
            theta_BS1.append(theta_bs1)
            theta_BS2.append(theta_bs2)

        theta_BS1 = np.array(theta_BS1)
        theta_BS2 = np.array(theta_BS2)
    else:
        theta_BS1 = np.zeros(len(timespan))
        theta_BS2 = np.zeros(len(timespan))

    return timespan, r_BN_N, sigma_BN, sigma_BR, omega_BR, rHatTrue_SB_B, thrForce, thrOnTimeRequest, pointingErrorAngle, theta_BS1, theta_BS2, SRPDataForce_B, SRPDataTorque_B


if __name__ == "__main__":
    run(
        True,  # show_plots
    )
