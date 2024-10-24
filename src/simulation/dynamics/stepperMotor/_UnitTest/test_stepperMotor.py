#
#  ISC License
#
#  Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
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
#   Unit Test Script
#   Module Name:        stepperMotor
#   Author:             Leah Kiner
#   Creation Date:      March 25, 2024
#

import inspect
import os
import matplotlib.pyplot as plt
import numpy as np
import pytest
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.simulation import stepperMotor
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

@pytest.mark.parametrize("initialMotorAngle", [0.0 * (np.pi / 180), 10.0 * (np.pi / 180), -5.0 * (np.pi / 180)])
@pytest.mark.parametrize("stepsCommanded", [0, 5, -5])
@pytest.mark.parametrize("stepAngle", [0.01 * (np.pi / 180), 0.5 * (np.pi / 180), 1.0 * (np.pi / 180)])
@pytest.mark.parametrize("stepTime", [0.1, 0.5, 1.0])
@pytest.mark.parametrize("accuracy", [1e-12])
def test_stepperMotor(show_plots, initialMotorAngle, stepsCommanded, stepAngle, stepTime, accuracy):
    r"""
    **Validation Test Description**

    This unit test ensures that the stepper motor profiler module correctly actuates the stepper motor from an initial
    angle to a final reference angle, given an input integer number of commanded steps contained in the
    ``motorStepCommand`` input message. The initial motor angle and number of commanded steps are varied so that the
    module is shown to work for both positive and negative steps. The motor states are profiled for each step using
    a bang-bang constant positive, constant negative acceleration profile. The motor acceleration is determined from
    the given constant step angle and constant step time.

    **Test Parameters**

    Args:
        initialMotorAngle (float): [rad] Initial stepper motor angle
        stepsCommanded (int): [steps] Number of steps commanded to the stepper motor
        stepAngle (float): [rad] Angle the stepper motor moves through for a single step (constant)
        stepTime (float): [sec] Time required for a single motor step (constant)
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    This unit test checks that the final motor angle matches the reference motor angele. The reference motor angle is
    determined from the initial motor angle and number of commanded steps. The test also checks that the module
    keeps track of the motor step count correctly by comparing the final motor step count with the number of steps
    commanded from the module ``motorStepCommand`` input message.

    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create the test thread
    testProcessRate = macros.sec2nano(0.1)     # Set process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the stepperMotor module to be tested
    StepperMotor = stepperMotor.StepperMotor()
    StepperMotor.ModelTag = "StepperMotor"
    StepperMotor.setThetaInit(initialMotorAngle)
    StepperMotor.setStepAngle(stepAngle)
    StepperMotor.setStepTime(stepTime)
    StepperMotor.setThetaDDotMax(stepAngle / (0.25 * stepTime * stepTime))

    # Add the test module to the runtime call list
    unitTestSim.AddModelToTask(unitTaskName, StepperMotor)

    # Create the StepperMotor input message
    MotorStepCommandMessageData = messaging.MotorStepCommandMsgPayload()
    MotorStepCommandMessageData.stepsCommanded = stepsCommanded
    MotorStepCommandMessage = messaging.MotorStepCommandMsg().write(MotorStepCommandMessageData)
    StepperMotor.motorStepCommandInMsg.subscribeTo(MotorStepCommandMessage)

    # Log the test module output message for data comparison
    stepperMotorDataLog = StepperMotor.stepperMotorOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, stepperMotorDataLog)

    # Initialize the simulation, set the sim run time, and execute the simulation
    unitTestSim.InitializeSimulation()
    actuateTime = stepTime * np.abs(stepsCommanded)  # [sec] Time for the motor to actuate to the desired angle
    holdTime = 5  # [sec] Time the simulation will continue while holding the final angle
    unitTestSim.ConfigureStopTime(macros.sec2nano(actuateTime + holdTime))
    unitTestSim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    timespan = stepperMotorDataLog.times()
    theta = macros.R2D * stepperMotorDataLog.theta
    thetaDot = macros.R2D * stepperMotorDataLog.thetaDot
    thetaDDot = macros.R2D * stepperMotorDataLog.thetaDDot
    motorStepCount = stepperMotorDataLog.stepCount
    motorCommandedSteps = stepperMotorDataLog.stepsCommanded

    # Only show plots if the motor actuates
    if (stepsCommanded == 0):
        show_plots = False

    # Plot motor angle
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, theta, label=r"$\theta$")
    plt.title(r'Stepper Motor Angle $\theta_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot motor thetaDot
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, thetaDot, label=r"$\dot{\theta}$")
    plt.title(r'Stepper Motor Angle Rate $\dot{\theta}_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
    plt.ylabel('(deg/s)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot motor thetaDDot
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, thetaDDot, label=r"$\ddot{\theta}$")
    plt.title(r'Stepper Motor Angular Acceleration $\ddot{\theta}_{\mathcal{F}/\mathcal{M}}$ ', fontsize=14)
    plt.ylabel('(deg/s$^2$)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot steps commanded and motor steps taken
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, motorStepCount)
    plt.plot(timespan * macros.NANO2SEC, motorCommandedSteps, '--', label='Commanded')
    plt.title(r'Motor Step History', fontsize=14)
    plt.ylabel('Steps', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")

    # Check to ensure the angle converges to the reference angle
    desiredMotorAngleTrue = initialMotorAngle + (stepsCommanded * stepAngle)
    np.testing.assert_allclose(theta[-1],
                               macros.R2D * desiredMotorAngleTrue,
                               atol=accuracy,
                               verbose=True)

    # Check to ensure angle rate converges to zero
    np.testing.assert_allclose(thetaDot[-1],
                               0.0,
                               atol=accuracy,
                               verbose=True)

    # Check the motor achieved the commanded steps
    np.testing.assert_allclose(motorStepCount[-1],
                               stepsCommanded,
                               atol=accuracy,
                               verbose=True)

if __name__ == "__main__":
    test_stepperMotor(
                 True,
                 0.0,                       # initialMotorAngle
                 10,                        # stepsCommanded
                 1.0 * (np.pi / 180),       # stepAngle
                 1.0,                       # stepTime
                 1e-12                      # accuracy
               )
