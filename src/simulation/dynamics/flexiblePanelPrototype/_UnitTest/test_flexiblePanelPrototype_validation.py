# ISC License
#
# Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


#
#   Unit Test Script
#   Module Name:        flexiblePanelPrototype
#   Author:             João Vaz Carneiro
#   Creation Date:      May 8, 2024
#

import inspect
import os
import pytest
import numpy as np
import matplotlib.pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import spacecraft, flexiblePanelPrototype, svIntegrators


@pytest.mark.parametrize("frequency", [1e3, 1e4, 1e5])
def test_flexiblePaneTestFunction(show_plots, frequency):
    flexiblePaneTestFunction(show_plots, frequency)


def flexiblePaneTestFunction(show_plots, frequency):
    dynTaskName = "dynTask"
    dynProcessName = "dynProcess"

    scSim = SimulationBaseClass.SimBaseClass()
    simulationTime = macros.min2nano(0.1)
    dynProcess = scSim.CreateNewProcess(dynProcessName)

    simTimeStep = macros.sec2nano(0.0001)
    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, simTimeStep))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "scObject"
    scObject.hub.mHub = 720
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    scObject.hub.v_CN_NInit = [[0.1], [0.1], [0.1]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.001], [0.001], [0.001]]

    flexiblePanel = flexiblePanelPrototype.FlexiblePanelPrototype()
    flexiblePanel.ModelTag = "flexiblePanel"

    flexiblePanel.mass1 = 50
    flexiblePanel.mass2 = 30
    flexiblePanel.dcm_FB = [[0.0, 1.0, 0.0],
                            [-1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0]]
    flexiblePanel.IS1PntSc1_F = [[30, 0.0, 0.0],
                                 [0.0, 40, 0.0],
                                 [0.0, 0.0, 50]]
    flexiblePanel.IS2PntSc2_F = [[30, 0.0, 0.0],
                                 [0.0, 40, 0.0],
                                 [0.0, 0.0, 50]]
    flexiblePanel.r_FB_B = [[0.0], [0.0], [1.0]]
    # flexiblePanel.r_Sc1S1_F = [[2], [0], [0]]
    # flexiblePanel.r_S2S1_F = [[4], [0], [0]]
    # flexiblePanel.r_Sc2S2_F = [[2], [0], [0]]

    springFrequency = frequency
    dampingFrequency = 0e1
    flexiblePanel.kX = springFrequency
    flexiblePanel.kY = springFrequency
    flexiblePanel.kZ = springFrequency
    flexiblePanel.kTheta1 = springFrequency
    flexiblePanel.kTheta2 = springFrequency
    flexiblePanel.kTheta3 = springFrequency
    flexiblePanel.kBeta1 = springFrequency
    flexiblePanel.cX = dampingFrequency
    flexiblePanel.cY = dampingFrequency
    flexiblePanel.cZ = dampingFrequency
    flexiblePanel.cTheta1 = dampingFrequency
    flexiblePanel.cTheta2 = dampingFrequency
    flexiblePanel.cTheta3 = dampingFrequency
    flexiblePanel.cBeta1 = dampingFrequency

    flexiblePanel.xInit = 0.0
    flexiblePanel.xDotInit = 0.0
    flexiblePanel.yInit = 0.0
    flexiblePanel.yDotInit = 0.0
    flexiblePanel.zInit = 0.0
    flexiblePanel.zDotInit = 0.0
    flexiblePanel.theta1Init = 0.0 * macros.D2R
    flexiblePanel.theta1DotInit = 0.0
    flexiblePanel.theta2Init = 0.0 * macros.D2R
    flexiblePanel.theta2DotInit = 0.0
    flexiblePanel.theta3Init = 0.0 * macros.D2R
    flexiblePanel.theta3DotInit = 0.0
    flexiblePanel.beta1Init = 0.0 * macros.D2R
    flexiblePanel.beta1DotInit = 0.0

    scObject.addStateEffector(flexiblePanel)
    scSim.AddModelToTask(dynTaskName, flexiblePanel)
    scSim.AddModelToTask(dynTaskName, scObject)

    scObjectUnified = spacecraft.Spacecraft()
    scObjectUnified.ModelTag = "scObjectUnified"
    scObjectUnified.hub.mHub = 800
    scObjectUnified.hub.IHubPntBc_B = [[1052, 0.0, 0.0], [0.0, 932, 0.0], [0.0, 0.0, 700.0]]
    scObjectUnified.hub.r_BcB_B = [[0.0], [0.0], [0.1]]
    scObjectUnified.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    scObjectUnified.hub.v_CN_NInit = [[0.1], [0.1], [0.1]]
    scObjectUnified.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObjectUnified.hub.omega_BN_BInit = [[0.001], [0.001], [0.001]]
    scSim.AddModelToTask(dynTaskName, scObjectUnified)

    datLog = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(dynTaskName, datLog)
    energyMomLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    scSim.AddModelToTask(dynTaskName, energyMomLog)
    stateLog = flexiblePanel.logger(["X"])
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
    plt.plot(timeSecDyn, (rotEnergy - initialRotEnergy) / initialRotEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Energy')

    plt.figure(7)
    plt.clf()
    for idx in range(3):
        plt.plot(timeSecDyn, stateLog.X[:, idx])
    plt.legend([r'$\Delta x$', r'$\Delta y$', r'$\Delta z$'])
    plt.xlabel('time (s)')

    plt.figure(8)
    plt.clf()
    for idx in range(4):
        plt.plot(timeSecDyn, stateLog.X[:, idx+3])
    plt.legend([r'$\theta_1$', r'$\theta_2$', r'$\theta_3$', r'$\beta_1$'])
    plt.xlabel('time (s)')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    for idx in range(3):
        np.testing.assert_allclose(datLog.omega_BN_B[:, idx], datLog2.omega_BN_B[:, idx], atol=1e-6/np.sqrt(frequency), rtol=1e-3)
        np.testing.assert_allclose(datLog.v_CN_N[:, idx], datLog2.v_CN_N[:, idx], atol=2e-6/np.sqrt(frequency), rtol=1e-3)


if __name__ == "__main__":
    test_flexiblePaneTestFunction(True, 1e3)
