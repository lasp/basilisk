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


def flexiblePanelSim(show_plots):
    dynTaskName = "dynTask"
    dynProcessName = "dynProcess"

    scSim = SimulationBaseClass.SimBaseClass()
    simulationTime = macros.min2nano(0.5)
    dynProcess = scSim.CreateNewProcess(dynProcessName)

    simTimeStep = macros.sec2nano(0.0001)
    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, simTimeStep))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "scObject"
    # integratorObject = svIntegrators.svIntegratorRKF45(scObject)
    # scObject.setIntegrator(integratorObject)
    massSC = 1000
    lengthSC = 5
    widthSC = 5
    heightSC = 10
    scObject.hub.mHub = massSC
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[massSC / 12 * (lengthSC ** 2 + heightSC ** 2), 0.0, 0.0],
                                [0.0, massSC / 12 * (widthSC ** 2 + heightSC ** 2), 0.0],
                                [0.0, 0.0, massSC / 12 * (lengthSC ** 2 + widthSC ** 2)]]

    scObject.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    scObject.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    flexiblePanel = flexiblePanelPrototype.FlexiblePanelPrototype()
    massPanel = 100
    lengthPanel = 10.0
    widthPanel = 5.0
    thicknessPanel = 0.1
    flexiblePanel.mass = massPanel / 2
    flexiblePanel.J = massPanel / 2 / 12 * ((lengthPanel / 2) ** 2 + thicknessPanel ** 2)
    flexiblePanel.l = lengthPanel / 2
    flexiblePanel.kX = (2 * np.pi * 1) ** 2 * massPanel
    flexiblePanel.kY = (2 * np.pi * 0.5) ** 2 * massPanel
    flexiblePanel.kTheta1 = (2 * np.pi * 0.1) ** 2 * massPanel / 12 * (4 * lengthPanel ** 2 + thicknessPanel ** 2)
    flexiblePanel.kTheta2 = (2 * np.pi * 1.0) ** 2 * 0.5 * massPanel / 12 * (4 * (lengthPanel / 2) ** 2 + thicknessPanel ** 2)
    flexiblePanel.cX = 2 * 1 / 60 * (2 * np.pi * 1.0) * massPanel
    flexiblePanel.cY = 2 * 1 / 60 * (2 * np.pi * 0.5) * massPanel
    flexiblePanel.cTheta1 = 2 * 1 / 60 * (2 * np.pi * 0.1) * massPanel / 12 * (4 * lengthPanel ** 2 + thicknessPanel ** 2)
    flexiblePanel.cTheta2 = 2 * 1 / 60 * (2 * np.pi * 1.0) * 0.5 * massPanel / 12 * (4 * (lengthPanel / 2) ** 2 + thicknessPanel ** 2)
    flexiblePanel.dcm_FB = [[0.0, 1.0, 0.0],
                            [0.0, 0.0, 1.0],
                            [1.0, 0.0, 0.0]]
    flexiblePanel.IPntFc_F = [[flexiblePanel.mass / 12 * (widthPanel ** 2 + thicknessPanel ** 2), 0.0, 0.0],
                              [0.0, flexiblePanel.mass / 12 * (widthPanel ** 2 + lengthPanel ** 2), 0.0],
                              [0.0, 0.0, flexiblePanel.mass / 12 * (thicknessPanel ** 2 + lengthPanel ** 2)]]
    flexiblePanel.r_FB_B = [[0.0], [lengthSC / 2], [heightSC / 2 - thicknessPanel / 2]]
    flexiblePanel.xInit = 0.01
    flexiblePanel.xDotInit = 0.0
    flexiblePanel.yInit = 0.01
    flexiblePanel.yDotInit = 0.0
    flexiblePanel.theta1Init = 0.01 * macros.D2R
    flexiblePanel.theta1DotInit = 0.0
    flexiblePanel.theta2Init = -0.01 * macros.D2R
    flexiblePanel.theta2DotInit = 0.0
    scObject.addStateEffector(flexiblePanel)

    scSim.AddModelToTask(dynTaskName, flexiblePanel)
    scSim.AddModelToTask(dynTaskName, scObject)

    datLog = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(dynTaskName, datLog)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    plt.close("all")
    timeSecDyn = datLog.times() * macros.NANO2MIN

    plt.figure(1)
    for idx in range(3):
        plt.plot(timeSecDyn, datLog.sigma_BN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude $\sigma_{B/N}$')

    plt.figure(2)
    for idx in range(3):
        plt.plot(timeSecDyn, datLog.omega_BN_B[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Rate $\omega_{B/N}$')

    plt.figure(3)
    for idx in range(3):
        plt.plot(timeSecDyn, datLog.r_CN_N[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$r_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Position $r_{C/N}$')

    plt.figure(4)
    for idx in range(3):
        plt.plot(timeSecDyn, datLog.v_CN_N[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$v_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Velocity $v_{C/N}$')

    if show_plots:
        plt.show()

    plt.close("all")


if __name__ == "__main__":
    flexiblePanelSim(True)
