# ISC License
#
# Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import inspect
import os
import pytest
import numpy as np
import matplotlib.pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros, RigidBodyKinematics as rbk, pythonVariableLogger
from Basilisk.simulation import spacecraft, stateSpaceStateEffector


@pytest.mark.parametrize("frequency", [1e3, 1e4, 1e5])
def test_flexiblePaneTestFunction(show_plots, frequency):
    dynTaskName = "dynTask"
    dynProcessName = "dynProcess"

    scSim = SimulationBaseClass.SimBaseClass()
    simulationTime = macros.min2nano(0.1)
    dynProcess = scSim.CreateNewProcess(dynProcessName)

    simTimeStep = macros.sec2nano(0.0001)
    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, simTimeStep))

    massHub = 720
    r_BcB_B = np.array([0.0, 0.0, 0.0])
    IHubPntBc_B = np.array([[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]])
    mass1 = 50
    mass2 = 30
    IS1PntSc1_F = np.array([[30, 0.0, 0.0],
                            [0.0, 40, 0.0],
                            [0.0, 0.0, 50]])
    IS2PntSc2_F = np.array([[30, 0.0, 0.0],
                            [0.0, 40, 0.0],
                            [0.0, 0.0, 50]])
    r_FB_B = np.array([0., 0.0, 1.0])
    r_Sc1S1_F = np.array([0, 0, 0])
    r_S2S1_F = np.array([0, 0, 0])
    r_Sc2S2_F = np.array([0, 0, 0])
    yHat_F = np.array([0.0, 1.0, 0.0])
    dcm_FB = np.array([[0.0, 1.0, 0.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0]])

    scObject = spacecraft.Spacecraft()
    scObject.hub.mHub = massHub
    scObject.hub.IHubPntBc_B = IHubPntBc_B
    scObject.hub.r_BcB_B = r_BcB_B
    scObject.hub.r_CN_NInit = np.array([0.0, 0.0, 0.0])
    scObject.hub.v_CN_NInit = np.array([0.1, 0.1, 0.1])
    scObject.hub.sigma_BNInit = np.array([0.0, 0.0, 0.0])
    scObject.hub.omega_BN_BInit = np.array([0.001, 0.001, 0.001])

    stateSpace = stateSpaceStateEffector.StateSpaceStateEffector(7)
    stateSpace.ModelTag = "stateSpace"

    mass = mass1 + mass2
    r_Sc2S1_F = r_Sc2S2_F + r_S2S1_F
    r_ScS1_F = (mass1 * r_Sc1S1_F + mass2 * r_Sc2S1_F) / mass
    r_Sc1Sc_F = r_Sc1S1_F - r_ScS1_F
    r_Sc2Sc_F = r_Sc2S2_F + r_S2S1_F - r_ScS1_F
    rTilde_Sc1S1_F = np.array(rbk.v3Tilde(r_Sc1S1_F))
    rTilde_Sc2S2_F = np.array(rbk.v3Tilde(r_Sc2S2_F))
    rTilde_S2S1_F = np.array(rbk.v3Tilde(r_S2S1_F))
    rTilde_Sc2S1_F = np.array(rbk.v3Tilde(r_Sc2S1_F))
    rTilde_ScS1_F = np.array(rbk.v3Tilde(r_ScS1_F))
    rTilde_Sc1Sc_F = np.array(rbk.v3Tilde(r_Sc1Sc_F))
    rTilde_Sc2Sc_F = np.array(rbk.v3Tilde(r_Sc2Sc_F))
    ISPntS1_F = IS1PntSc1_F - mass1 * rTilde_Sc1S1_F @ rTilde_Sc1S1_F + IS2PntSc2_F - mass2 * rTilde_Sc2S1_F @ rTilde_Sc2S1_F
    IS2PntS2_F = IS2PntSc2_F - mass2 * rTilde_Sc2S2_F @ rTilde_Sc2S2_F
    ISPntSc_F = IS1PntSc1_F - mass1 * rTilde_Sc1Sc_F @ rTilde_Sc1Sc_F + IS2PntSc2_F - mass2 * rTilde_Sc2Sc_F @ rTilde_Sc2Sc_F

    M = np.zeros((7, 7))
    M[0:3, 0:3] = mass * np.identity(3)
    M[0:3, 3:6] = - mass * rTilde_ScS1_F
    M[0:3, 6] = - mass2 * rTilde_Sc2S2_F @ yHat_F
    M[3:6, 0:3] = mass * rTilde_ScS1_F
    M[3:6, 3:6] = ISPntS1_F
    M[3:6, 6] = (IS2PntS2_F - mass2 * rTilde_S2S1_F @ rTilde_Sc2S2_F) @ yHat_F
    M[6, 0:3] = mass2 * yHat_F.transpose() @ rTilde_Sc2S2_F
    M[6, 3:6] = yHat_F.transpose() @ (IS2PntS2_F + mass2 * rTilde_Sc2S2_F @ rTilde_S2S1_F)
    M[6, 6] = yHat_F.transpose() @ IS2PntS2_F @ yHat_F

    stateSpace.mass = mass
    stateSpace.M = M
    stateSpace.ISPntSc_F = ISPntSc_F
    stateSpace.r_FB_B = r_FB_B
    stateSpace.r_ScS_F = r_ScS1_F
    stateSpace.dcm_FB = dcm_FB

    springFrequency = frequency
    dampingFrequency = 0.0
    stateSpace.K = [[springFrequency, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 1.3 * springFrequency, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.9 * springFrequency, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.2 * springFrequency, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.3 * springFrequency, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.7 * springFrequency, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.4 * springFrequency]]
    stateSpace.C = [[0.1 * dampingFrequency, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.3 * dampingFrequency, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.1 * dampingFrequency, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.2 * dampingFrequency, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.7 * dampingFrequency, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.5 * dampingFrequency, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, dampingFrequency]]

    stateSpace.XInit = [0.0,
                        0.0,
                        0.0,
                        0.0 * macros.D2R,
                        0.0 * macros.D2R,
                        0.0 * macros.D2R,
                        0.0 * macros.D2R]
    stateSpace.XDotInit = [0.0,
                           0.0,
                           0.0,
                           0.0 * macros.D2R,
                           0.0 * macros.D2R,
                           0.0 * macros.D2R,
                           0.0 * macros.D2R]

    scObject.addStateEffector(stateSpace)
    scSim.AddModelToTask(dynTaskName, stateSpace)
    scSim.AddModelToTask(dynTaskName, scObject)

    mUnified = scObject.hub.mHub + mass
    rUnified_BcB_B = (scObject.hub.mHub * r_BcB_B + mass * (dcm_FB.transpose() @ r_ScS1_F + r_FB_B)) / mUnified
    IUnifiedPntBc_B = (IHubPntBc_B + dcm_FB.transpose() @ ISPntSc_F @ dcm_FB
                       - massHub * np.array(rbk.v3Tilde(rUnified_BcB_B)) @ np.array(rbk.v3Tilde(rUnified_BcB_B))
                       - mass * np.array(rbk.v3Tilde(dcm_FB.transpose() @ r_ScS1_F + r_FB_B - rUnified_BcB_B))
                            @ np.array(rbk.v3Tilde(dcm_FB.transpose() @ r_ScS1_F + r_FB_B - rUnified_BcB_B)))

    scObjectUnified = spacecraft.Spacecraft()
    scObjectUnified.ModelTag = "scObjectUnified"
    scObjectUnified.hub.mHub = mUnified
    scObjectUnified.hub.r_BcB_B = rUnified_BcB_B
    scObjectUnified.hub.IHubPntBc_B = IUnifiedPntBc_B
    scObjectUnified.hub.r_CN_NInit = scObject.hub.r_CN_NInit
    scObjectUnified.hub.v_CN_NInit = scObject.hub.v_CN_NInit
    scObjectUnified.hub.sigma_BNInit = scObject.hub.sigma_BNInit
    scObjectUnified.hub.omega_BN_BInit = scObject.hub.omega_BN_BInit
    scSim.AddModelToTask(dynTaskName, scObjectUnified)

    datLog = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(dynTaskName, datLog)
    energyMomLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    scSim.AddModelToTask(dynTaskName, energyMomLog)
    stateLog = pythonVariableLogger.PythonVariableLogger({"X": lambda _: scObject.dynManager.getStateObject("X1").getState()})
    scSim.AddModelToTask(dynTaskName, stateLog)
    datLog2 = scObjectUnified.scStateOutMsg.recorder()
    scSim.AddModelToTask(dynTaskName, datLog2)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Set up the conservation quantities
    orbEnergy = energyMomLog.totOrbEnergy
    initialOrbEnergy = orbEnergy[0]
    rotEnergy = energyMomLog.totRotEnergy
    initialRotEnergy = rotEnergy[0]

    plt.close("all")
    timeSecDyn = datLog.times() * macros.NANO2MIN

    plt.figure(1)
    for idx in range(3):
        plt.plot(timeSecDyn, datLog.sigma_BN[:, idx] - datLog2.sigma_BN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude $\sigma_{B/N}$')

    plt.figure(2)
    for idx in range(3):
        plt.plot(timeSecDyn, datLog.omega_BN_B[:, idx] - datLog2.omega_BN_B[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Rate $\omega_{B/N}$')

    plt.figure(3)
    for idx in range(3):
        plt.plot(timeSecDyn, datLog.r_CN_N[:, idx] - datLog2.r_CN_N[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$r_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Position $r_{C/N}$')

    plt.figure(4)
    for idx in range(3):
        plt.plot(timeSecDyn, datLog.v_CN_N[:, idx] - datLog2.v_CN_N[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$v_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Velocity $v_{C/N}$')

    plt.figure(5)
    plt.clf()
    plt.plot(timeSecDyn, (orbEnergy - initialOrbEnergy) / initialOrbEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Energy')

    plt.figure(6)
    plt.clf()
    for idx in range(3):
        plt.plot(timeSecDyn, stateLog.X[:, idx])
    plt.legend([r'$\Delta x$', r'$\Delta y$', r'$\Delta z$'])
    plt.xlabel('time (s)')

    plt.figure(7)
    plt.clf()
    for idx in range(4):
        plt.plot(timeSecDyn, stateLog.X[:, idx+3])
    plt.legend([r'$\theta_1$', r'$\theta_2$', r'$\theta_3$', r'$\beta_1$'])
    plt.xlabel('time (s)')

    plt.figure(8)
    plt.clf()
    plt.plot(timeSecDyn, (rotEnergy - initialRotEnergy) / initialRotEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Energy')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    for idx in range(3):
        np.testing.assert_allclose(datLog.omega_BN_B[:, idx], datLog2.omega_BN_B[:, idx], atol=1e-6/np.sqrt(frequency), rtol=1e-3)
        np.testing.assert_allclose(datLog.v_CN_N[:, idx], datLog2.v_CN_N[:, idx], atol=2e-6/np.sqrt(frequency), rtol=1e-3)


if __name__ == "__main__":
    test_flexiblePaneTestFunction(True, 1e6)
