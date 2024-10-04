#
#  ISC License
#
#  Copyright (c) 2024, Laboratory of Atmospheric and Space Physics, University of Colorado at Boulder
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

import pytest
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass, macros, fswSetupRW
from Basilisk.fswAlgorithms import thrMomentumManagementCpp
from Basilisk.architecture import messaging


@pytest.mark.parametrize("hsMinCheck", [(0), (1)])
def test_thrMomentumManagement(show_plots, hsMinCheck):
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    module = thrMomentumManagementCpp.ThrMomentumManagementCpp()
    module.ModelTag = "thrMomentumManagement"
    unitTestSim.AddModelToTask(unitTaskName, module)

    if hsMinCheck:
        module.hs_min = 1000. / 6000. * 100.  # Nms
    else:
        module.hs_min = 100. / 6000. * 100.  # Nms

    # Define instantaneous wheel speed information to run the simulation
    rwSpeedMessage = messaging.RWSpeedMsgPayload()
    rwSpeedMessage.wheelSpeeds = [10.0, -25.0, 50.0, 100.]
    rwSpeedInMsg = messaging.RWSpeedMsg().write(rwSpeedMessage)
    fswSetupRW.clearSetup()
    Js = 0.1
    fswSetupRW.create([1.0, 0.0, 0.0], Js)
    fswSetupRW.create([0.0, 1.0, 0.0], Js)
    fswSetupRW.create([0.0, 0.0, 1.0], Js)
    fswSetupRW.create([0.5773502691896258, 0.5773502691896258, 0.5773502691896258], Js)
    rwConfigInMsg = fswSetupRW.writeConfigMessage()
    module.rwSpeedsInMsg.subscribeTo(rwSpeedInMsg)
    module.rwConfigDataInMsg.subscribeTo(rwConfigInMsg)

    dataLog = module.deltaHOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))
    unitTestSim.InitializeSimulation()
    unitTestSim.ExecuteSimulation()

    # Define expected output depending on the type of check
    if hsMinCheck == 1:
        trueVector = np.array([[0.0, 0.0, 0.0]] * 2)
    else:
        trueVector = np.array([[-5.914369484146579, -2.858300248464629, -9.407020039211664],
                               [0.0, 0.0, 0.0]])
    np.testing.assert_allclose(trueVector, dataLog.torqueRequestBody, rtol=1e-12)


def test_momentumBias(show_plots):
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    module = thrMomentumManagementCpp.ThrMomentumManagementCpp()
    module.ModelTag = "thrMomentumManagement"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    hd_B = np.array([1.0, 0.0, 0.0])  # Nms
    module.hd_B = hd_B

    # wheelSpeeds Message
    rwSpeedMessage = messaging.RWSpeedMsgPayload()
    rwSpeedMessage.wheelSpeeds = [10.0, -25.0, 50.0, 100.]
    rwSpeedInMsg = messaging.RWSpeedMsg().write(rwSpeedMessage)

    # wheelConfigData Message
    fswSetupRW.clearSetup()
    Js = 0.1
    Gs = np.array([[1.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0],
                   [0.0, 0.0, 1.0],
                   [0.5773502691896258, 0.5773502691896258, 0.5773502691896258]])
    fswSetupRW.create(Gs[0, :], Js)
    fswSetupRW.create(Gs[1, :], Js)
    fswSetupRW.create(Gs[2, :], Js)
    fswSetupRW.create(Gs[3, :], Js)
    rwConfigInMsg = fswSetupRW.writeConfigMessage()

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.deltaHOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # setup message connections
    module.rwSpeedsInMsg.subscribeTo(rwSpeedInMsg)
    module.rwConfigDataInMsg.subscribeTo(rwConfigInMsg)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))  # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # Compute the wheel momentum vector
    hs_B = np.zeros(3)
    for i in range(4):
        hs_B += Js * rwSpeedMessage.wheelSpeeds[i] * Gs[i, :]

    # set the filtered output truth states
    trueVector = hd_B - hs_B

    # compare the module results to the truth values
    np.testing.assert_allclose(trueVector, dataLog.torqueRequestBody[0, :], rtol=1e-12)


if __name__ == "__main__":
    test_thrMomentumManagement(
                 True,
                 0            # hsMinCheck
               )
