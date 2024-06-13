import pytest
import inspect
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import math  
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import sepGimbalLookUpTable
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Vary the thrust unit vector angle for pytest
#Shamsa: thetaint is gonna excute twice one in 0 and once in 2*np.pi/3

# @pytest.mark.parametrize("desiredAngle", [0, 2*np.pi/3])
@pytest.mark.parametrize("accuracy", [1e-12])
def test_sepGimbalLookUpTableTestFunction(inputTipAngle, inputTiltAngle, accuracy):
    r"""
    **Validation Test Description**

    This unit test ensures that the stepper motor is properly computed, where the unput of deisred angle will give us the right number of motor steps. 
    **Test Parameters**

    Args:
        desiredAngle (float): [rad] desired angle value
        thetaInit (float): [rad] Initial PRV angle of the F frame with respect to the M frame
        thetaRef (float): [rad] Reference PRV angle of the F frame with respect to the M frame
        thetaDDotMax (float): [rad/s^2] Maximum angular acceleration for the attitude maneuver

    **Description of Variables Being Tested**

    This unit test ensures that the profiled 1 DOF rotational attitude maneuver is properly computed for a series of
    initial and reference PRV angles and maximum angular accelerations. The final prescribed angle ``theta_FM_Final``
    and angular velocity magnitude ``thetaDot_Final`` are compared with the reference values ``theta_Ref`` and
    ``thetaDot_Ref``, respectively.
    """
    [testResults, testMessage] = sepGimbalLookUpTableTestFunction(inputTipAngle, inputTiltAngle, accuracy)

#assertion statement checks whether a given condition is True.
    # If the condition is False, an AssertionError is raised, and an optional error message (testMessage) can be
    # provided to explain the reason for the assertion failure.
    assert testResults < 1, testMessage

def sepGimbalLookUpTableTestFunction(tipAngle, tiltAngle, accuracy):
    testFailCount = 0                                        # Zero the unit test result counter
    testMessages = []                                        # Create an empty array to store the test log messages
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create the test thread
    testProcessRate = macros.sec2nano(1)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the sepGimbalLookUpTable module to be tested
    SepGimbalLookUpTable = sepGimbalLookUpTable.SepGimbalLookUpTable()
    SepGimbalLookUpTable.ModelTag = "sepGimbalLookUpTable"

    # Add the sepGimbal test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, SepGimbalLookUpTable, SepGimbalLookUpTable)

    # Create variable for the tip and tilt angles 
    HingedRigidBodyTipAngleData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyTipAngleData.theta = tipAngle

    HingedRigidBodyTiltAngleData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyTiltAngleData.theta = tiltAngle

    # Create the sepGimbalLookUpTable tip and tilt angles input message
    HingedRigidBodyTipAngleMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyTipAngleData)
    SepGimbalLookUpTable.desiredGimbalTipAngleInMsg.subscribeTo(HingedRigidBodyTipAngleMessage)

    HingedRigidBodyTiltAngleMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyTiltAngleData)
    SepGimbalLookUpTable.desiredGimbalTiltAngleInMsg.subscribeTo(HingedRigidBodyTiltAngleMessage)

    #SepGimbalLookUpTable.fileName = "C:\\Users\\ShamsaS\\Documents\\software-projects\\basilisk-lasp\\src\\fswAlgorithms\\effectorInterfaces\sepGimbalLookUpTable\platformToMotor1.h"

    # Log the test module output message for data comparison
    motor1AngleMsgLog = SepGimbalLookUpTable.motor1AngleOutMsg.recorder()
    motor2AngleMsgLog = SepGimbalLookUpTable.motor2AngleOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, motor1AngleMsgLog)
    unitTestSim.AddModelToTask(unitTaskName, motor2AngleMsgLog)

   # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Set the simulation time
    #unitTestSim.ConfigureStopTime(macros.sec2nano(30))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()
    
    #print("")
    #print(type(motor1AngleMsgLog))
    #print(type(motor2AngleMsgLog))


# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
if __name__ == "__main__":
    sepGimbalLookUpTableTestFunction(
                 -11,    # tipAngle1
                 -10.1,   # tiltAngle1
                 1e-12  # accuracy
    )
