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
# Author:  Leah Kiner
# Creation Date:  February 23, 2023
#

import os
import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__
from Basilisk.simulation import spacecraft, hingedRigidBodyStateEffector
from Basilisk.utilities import SimulationBaseClass, orbitalMotion, simIncludeGravBody, macros, unitTestSupport, vizSupport
from Basilisk.architecture import messaging

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

def run(show_plots):
    """Call this routine directly to run the unit test.

    Args:
        show_plots (bool): Determines if the script should display plots
    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    # Create a sim motorControlModule as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # Create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # Create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(0.1)  # [ns]
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Create the spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"

    # Set up the gravitational bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    mu = earth.mu

    # Attach the gravity model to the spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Define the mass properties of the rigid spacecraft hub
    scObject.hub.mHub = 800.0  # [kg]
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # [m]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 800.0]]  # [kg m^2]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # [rad/s]

    # Create a panel to add to the spacecraft
    panel = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    panel.mass = 100.0  # [kg]
    panel.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]  # [kg m^2]
    panel.d = 0  # [m] Distance from hinge point H to the panel center of mass S
    panel.k = 200.0  # [N-m/rad] Torsional spring constant of hinge
    panel.c = 5.0  # [N-m-s/rad] Rotational damping coefficient for the hinge
    panel.r_HB_B = [[0.0], [0.0], [0.0]]  # [m]
    panel.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    thetaInit = 0 * np.pi / 180.0  # [rad]
    panel.thetaInit = thetaInit  # [rad]
    panel.thetaDotInit = 0.0 * np.pi / 180.0  # [rad/s]

    # Define the angular reference values
    thetaRef = 5 * np.pi / 180  # [rad]
    thetaDotRef = 0.0  # [rad/s]

    # Add the effector module to the spacecraft
    scObject.addStateEffector(panel)

    # Add the modules to the simulation task
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, panel)

    # Set up the spacecraft orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000  # [m]
    oe.a = rLEO  # [m]
    oe.e = 0.0001
    oe.i = 0.0 * macros.D2R  # [rad]
    oe.Omega = 48.2 * macros.D2R  # [rad]
    oe.omega = 347.8 * macros.D2R  # [rad]
    oe.f = 85.3 * macros.D2R  # [rad]
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # [m]
    scObject.hub.v_CN_NInit = vN  # [m/s]

    # Define the stepper motor information
    angleStep = 5 * np.pi / 180  # [rad]
    numSteps = int((thetaRef - thetaInit) / angleStep)
    thetaRefIntermediate = thetaInit

    # If this scenario is to interface with the BSK Viz, uncomment the following lines
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              # , saveFile=fileName
                                              )

    # Set up data logging before the simulation is initialized
    scLog = scObject.scStateOutMsg.recorder()
    panelLog = panel.hingedRigidBodyOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, scLog)
    scSim.AddModelToTask(simTaskName, panelLog)

    # Define temporal information
    P = 2. * np.pi * np.sqrt(oe.a * oe.a * oe.a / mu)  # Orbit period [s]
    simulationTimeFactor = 1
    simulationTime = 0.0

    # Add the panel to the scSim to use with the event handler
    scSim.panel = panel

    for step in range(numSteps):
        # Define the intermediate reference angle
        thetaRefIntermediate = thetaRefIntermediate + angleStep

        # Create the reference message
        hingedBodyStateReference = messaging.HingedRigidBodyMsgPayload()
        hingedBodyStateReference.theta = thetaRefIntermediate  # [rad]
        hingedBodyStateReference.thetaDot = thetaDotRef  # [rad/s]
        hingedBodyStateReferenceInMsg = messaging.HingedRigidBodyMsg().write(hingedBodyStateReference)

        # Connect all required messages
        panel.hingedRigidBodyRefMsg.subscribeTo(hingedBodyStateReferenceInMsg)

        # Set the simulation time
        simulationTime = simulationTime + macros.sec2nano(simulationTimeFactor * P) # [ns]

        # Event to terminate the simulation when the panel angle has converged to the reference
        scSim.createNewEvent("AngleConvergence" + str(step+1), simulationTimeStep, True,
                             [f"np.abs(self.panel.hingedRigidBodyOutMsg.read().theta - {thetaRefIntermediate}) < 1e-8 and np.abs(self.panel.hingedRigidBodyOutMsg.read().thetaDot - {thetaDotRef}) < 1e-8"],
                             [], terminal=True)

        # Run the simulation
        if step == 0:
            scSim.InitializeSimulation()

        scSim.ConfigureStopTime(simulationTime)
        scSim.ExecuteSimulation()

    # Retrieve the logged data
    panelThetaData = panelLog.theta * (180 / np.pi)
    panelThetaDotData = panelLog.thetaDot * (180 / np.pi)
    timespan = scLog.times()
    np.set_printoptions(precision=16)

    # Plot the results
    plt.close("all")  # clears out plots from earlier test runs

    # Plot the panel angle and angle rate
    fig, plts = plt.subplots(2, 1, sharex=True)
    l1, = plts[0].plot(timespan * macros.NANO2MIN, panelThetaData)
    l2, = plts[0].plot(timespan * macros.NANO2MIN, np.ones(len(timespan)) * (thetaRef * 180 / np.pi) , '--')
    plts[0].set_title(r'Panel Angle, $\theta$')
    l3, = plts[1].plot(timespan * macros.NANO2MIN, panelThetaDotData)
    l4, = plts[1].plot(timespan * macros.NANO2MIN, np.ones(len(timespan)) * (thetaDotRef * 180 / np.pi), '--')
    plts[1].set_title(r'Panel Angle Rate, $\dot{\theta}$')
    plts.flat[0].set(ylabel='[deg]')
    plts.flat[1].set(xlabel='Time [min]', ylabel='[deg/s]')
    plts[0].legend((l1, l2), (r'$\theta$', r'$\theta_{Ref}$', ), loc='upper right', prop={'size': 16})
    plts[1].legend((l3, l4), (r'$\dot{\theta}$', r'$\dot{\theta}_{Ref}$', ), loc='upper right', prop={'size': 16})
    pltName = fileName + "panelThetaAndThetaDot" + str(int(0.))
    figureList = {}
    figureList[pltName] = plt.figure(1)

    if show_plots:
        plt.show()

    plt.close("all")

    return figureList

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True  # show_plots
    )
