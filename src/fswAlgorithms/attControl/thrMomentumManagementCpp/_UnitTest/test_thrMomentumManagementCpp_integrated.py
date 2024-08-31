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

import os
import pytest
import numpy as np
import matplotlib.pyplot as plt

from Basilisk import __path__
from Basilisk.simulation import spacecraft, reactionWheelStateEffector, extForceTorque, simpleNav
from Basilisk.fswAlgorithms import thrMomentumManagementCpp, inertial3D, attTrackingError, mrpFeedback, rwMotorTorque
from Basilisk.utilities import SimulationBaseClass, macros, fswSetupRW, simIncludeRW, unitTestSupport
from Basilisk.architecture import messaging

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def test_momentumBiasIntegrated(show_plots):
    # Create simulation variable names
    fswTask = "fswTask"
    dynTask = "dynTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  Create the simulation process and tasks
    simProcess = scSim.CreateNewProcess(simProcessName)
    fswTimeStep = macros.sec2nano(1)
    dynTimeStep = macros.sec2nano(0.1)
    simProcess.addTask(scSim.CreateNewTask(dynTask, dynTimeStep))
    simProcess.addTask(scSim.CreateNewTask(fswTask, fswTimeStep))

    # Initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "scObject"
    scSim.AddModelToTask(dynTask, scObject)
    I = np.array([[1700, 0., 0.],
                  [0., 1700, 0.],
                  [0., 0., 1800]])
    scObject.hub.mHub = 2500
    scObject.hub.r_BcB_B = np.array([0.0, 0.0, 1.28])
    scObject.hub.IHubPntBc_B = I

    # Define spacecraft initial conditions
    scObject.hub.r_CN_NInit = [0, 0., 0.]
    scObject.hub.v_CN_NInit = [0, 0., 0.]
    scObject.hub.sigma_BNInit = [0, 0., 0.]
    scObject.hub.omega_BN_BInit = [0., 0., 0.]

    # Define the RWs
    rwFactory = simIncludeRW.rwFactory()
    varRWModel = messaging.BalancedWheels
    c = 1 / np.sqrt(2)
    RW1 = rwFactory.create('Honeywell_HR16', [c, 0, c], maxMomentum=100., Omega=4000., RWModel=varRWModel)
    RW2 = rwFactory.create('Honeywell_HR16', [0, c, c], maxMomentum=100., Omega=2000., RWModel=varRWModel)
    RW3 = rwFactory.create('Honeywell_HR16', [-c, 0, c], maxMomentum=100., Omega=-3500., RWModel=varRWModel)
    RW4 = rwFactory.create('Honeywell_HR16', [0, -c, c], maxMomentum=100., Omega=-1500., RWModel=varRWModel)
    numRW = rwFactory.getNumOfDevices()
    RW = [RW1, RW2, RW3, RW4]

    # Create RW object container for attitude control
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "RW_cluster"
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)
    scSim.AddModelToTask(dynTask, rwStateEffector)

    # Set up extForceTorque module for desaturation torque
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(dynTask, extFTObject)

    # Add the navigation module
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(dynTask, sNavObject)

    # Set up the attitude inertial guidance module
    inertial3DObj = inertial3D.Inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    scSim.AddModelToTask(fswTask, inertial3DObj)
    inertial3DObj.sigma_R0N = [0., 0., 0.]

    # Set up the attitude tracking error module
    attError = attTrackingError.AttTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(fswTask, attError)

    # Set up the MRP Feedback control module
    mrpControl = mrpFeedback.MrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(fswTask, mrpControl)
    decayTime = 10.0
    xi = 1.0
    mrpControl.Ki = -1
    mrpControl.P = 3 * np.max(I)/decayTime
    mrpControl.K = (mrpControl.P / xi) * (mrpControl.P / xi) / np.max(I)
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    # Add the module that maps the control torque into the RW motor torques
    rwMotorTorqueObj = rwMotorTorque.RwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(fswTask, rwMotorTorqueObj)
    rwMotorTorqueObj.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    # Construct the desaturation momentum algorithm
    thrDesatControl = thrMomentumManagementCpp.ThrMomentumManagementCpp()
    thrDesatControl.ModelTag = "thrMomentumManagement"
    scSim.AddModelToTask(fswTask, thrDesatControl)
    hd_B = np.array([110, 50, -10])  # Nms
    thrDesatControl.hd_B = hd_B

    # Write config messages
    fswRwParamMsg = rwFactory.getConfigMessage()
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I.flatten()
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # Set up message connections
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControl.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    mrpControl.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)
    thrDesatControl.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    thrDesatControl.rwConfigDataInMsg.subscribeTo(fswRwParamMsg)
    extFTObject.cmdTorqueInMsg.subscribeTo(thrDesatControl.deltaHOutMsg)

    # Log variables
    sNavRec = sNavObject.attOutMsg.recorder(dynTimeStep)
    scSim.AddModelToTask(dynTask, sNavRec)
    dataRec = scObject.scStateOutMsg.recorder(dynTimeStep)
    scSim.AddModelToTask(dynTask, dataRec)
    rwMotorLog = rwMotorTorqueObj.rwMotorTorqueOutMsg.recorder(fswTimeStep)
    scSim.AddModelToTask(dynTask, rwMotorLog)
    attErrorLog = attError.attGuidOutMsg.recorder(fswTimeStep)
    scSim.AddModelToTask(dynTask, attErrorLog)
    deltaHLog = thrDesatControl.deltaHOutMsg.recorder(fswTimeStep)
    scSim.AddModelToTask(dynTask, deltaHLog)
    mrpLog = rwStateEffector.rwSpeedOutMsg.recorder(fswTimeStep)
    scSim.AddModelToTask(dynTask, mrpLog)
    dataLog = thrDesatControl.deltaHOutMsg.recorder()
    scSim.AddModelToTask(fswTask, dataLog)
    rwLogs = []
    for item in range(numRW):
        rwLogs.append(rwStateEffector.rwOutMsgs[item].recorder(dynTimeStep))
        scSim.AddModelToTask(dynTask, rwLogs[item])

    # Run the simulation
    simulationTime = macros.min2nano(10)
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Retrieve logged data
    dataSigmaBR = attErrorLog.sigma_BR
    dataOmegaBR = attErrorLog.omega_BR_B
    dataOmegaRW = mrpLog.wheelSpeeds
    dataDH = deltaHLog.torqueRequestBody
    np.set_printoptions(precision=16)
    timeData = rwMotorLog.times() * macros.NANO2SEC

    # Compute the wheel momentum vector
    rwMomentum = np.zeros((len(timeData), 3))
    for j in range(len(timeData)):
        for idx in range(numRW):
            rwMomentum[j, :] += dataOmegaRW[j, idx] * RW[idx].Js * np.array(RW[idx].gsHat_B).flatten()

    # Plots
    plot_attitude_error(timeData, dataSigmaBR)
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plot_rate_error(timeData, dataOmegaBR)
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plot_rw_momenta(timeData, rwMomentum, hd_B)
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)

    plot_DH(timeData, dataDH)
    pltName = fileName + "4"
    figureList[pltName] = plt.figure(4)

    if show_plots:
        plt.show()

    plt.close("all")

    # Compare the last rw momenta to the desired value
    np.testing.assert_allclose(rwMomentum[-1, :], hd_B, rtol=1e-4)


def plot_attitude_error(timeData, dataSigmaBR):
    """Plot the attitude errors."""
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + r'$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')


def plot_rate_error(timeData, dataOmegaBR):
    """Plot the body angular velocity rate tracking errors."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeData, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx+1) + r'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel('Rate Tracking Error (rad/s) ')


def plot_rw_momenta(timeData, rwMomentum, hd_B):
    """Plot the RW momenta."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeData, rwMomentum[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$H_{' + str(idx+1) + r'}$')
        plt.hlines(y=hd_B[idx], xmin=timeData[0], xmax=timeData[-1],
                    color=unitTestSupport.getLineColor(idx, 3), linestyle='--')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel('RW Momentum (Nms)')


def plot_DH(timeData, dataDH):
    """Plot the body angular velocity rate tracking errors."""
    plt.figure(4)
    for idx in range(3):
        plt.plot(timeData, dataDH[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\Delta H_{' + str(idx+1) + r'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel('Dumped momentum (Nms) ')


def plot_rw_speeds(timeData, dataOmegaRW, numRW):
    """Plot the RW spin rates."""
    plt.figure(5)
    for idx in range(numRW):
        plt.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\Omega_{' + str(idx+1) + r'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel('RW Speed (RPM) ')


if __name__ == "__main__":
    test_momentumBiasIntegrated(True)
