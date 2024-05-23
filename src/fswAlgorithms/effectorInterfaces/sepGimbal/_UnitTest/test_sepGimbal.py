import pytest
import inspect
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import math  # Added import for math module
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import sepGimbal 
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
def test_sepGimbalTestFunction(show_plots, desiredThrustUnitVector, accuracy):
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
    [testResults, testMessage] = sepGimbalTestFunction(show_plots, desiredThrustUnitVector, accuracy)

#assertion statement checks whether a given condition is True.
    # If the condition is False, an AssertionError is raised, and an optional error message (testMessage) can be
    # provided to explain the reason for the assertion failure.
    assert testResults < 1, testMessage

def sepGimbalTestFunction(showPlots, desiredThrustUnitVector1, desiredThrustUnitVector2, desiredThrustUnitVector3, accuracy):
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

    # Create an instance of the sepGimbal module to be tested
    SepGimbalConfig = sepGimbal.SepGimbalConfig()
    SepGimbalWrap = unitTestSim.setModelDataWrap(SepGimbalConfig)
    SepGimbalWrap.ModelTag = "sepGimbal"

    # Add the sepGimbal test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, SepGimbalWrap, SepGimbalConfig)

    # Create the sepGimbal input message
    BodyHeadingMessageData = messaging.BodyHeadingMsgPayload()
    BodyHeadingMessageData.rHat_XB_B = desiredThrustUnitVector1         
    BodyHeadingMessage = messaging.BodyHeadingMsg().write(BodyHeadingMessageData)
    SepGimbalConfig.desiredThrustMountFrameInMsg.subscribeTo(BodyHeadingMessage)

    # Log the test module output message for data comparison
    sepGimbalTipMsgLog = SepGimbalConfig.desiredGimbalTipAngleOutMsg.recorder()
    sepGimbalTiltMsgLog = SepGimbalConfig.desiredGimbalTiltAngleOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, sepGimbalTipMsgLog)
    unitTestSim.AddModelToTask(unitTaskName, sepGimbalTiltMsgLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Set the simulation time
    unitTestSim.ConfigureStopTime(macros.sec2nano(30))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()

    # # Create the sepGimbal input message
    # BodyHeadingMessageData = messaging.BodyHeadingMsg_C_zeroMsgPayload()
    # print("")
    # BodyHeadingMessageData.rHat_XB_B = desiredThrustUnitVector2
    # BodyHeadingMessage = messaging.BodyHeadingMsg().write(BodyHeadingMessageData, unitTestSim.TotalSim.CurrentNanos)
    # SepGimbalConfig.desiredThrustMountFrameInMsg.subscribeTo(BodyHeadingMessage)

    # # Set the simulation time
    # unitTestSim.ConfigureStopTime(macros.sec2nano(30) + macros.sec2nano(30))

    # # Begin the simulation
    # unitTestSim.ExecuteSimulation()

    # # Create the sepGimbal input message
    # print("")
    # BodyHeadingMessageData = messaging.BodyHeadingMsg_C_zeroMsgPayload()
    # BodyHeadingMessageData.rHat_XB_B = desiredThrustUnitVector3
    # BodyHeadingMessage = messaging.BodyHeadingMsg().write(BodyHeadingMessageData, unitTestSim.TotalSim.CurrentNanos)
    # SepGimbalConfig.desiredThrustMountFrameInMsg.subscribeTo(BodyHeadingMessage)
    # # Set the simulation time
    # unitTestSim.ConfigureStopTime(macros.sec2nano(30) + macros.sec2nano(30) +  macros.sec2nano(30))

    # # Begin the simulation
    # unitTestSim.ExecuteSimulation()

    # Extract the logged data 
    print("")
   
   # vector in platform frame
    platformVector = [0.0, 0.0, 1.0]

    # Agnles values
    Tip_angle = 35*(math.pi / 180)
    Tilt_angle = 25*(math.pi / 180)

    print(f"\nPython tip angle: {Tip_angle:.2f}\nPython tilt angle: {Tilt_angle:.2f}")

    # Tilt transformation matrix in mount frame
    M1 = [
        [1, 0, 0],
        [0, math.cos(Tilt_angle), math.sin(Tilt_angle)],
        [0, -math.sin(Tilt_angle), math.cos(Tilt_angle)]
    ]

    print("\nTilt transformation matrix:")
    for i in range(3):
        for j in range(3):
            print(M1[i][j], end=" ")
        print()

    # Tip transformation matrix in mount frame
    M2 = [
        [math.cos(Tip_angle), 0, -math.sin(Tip_angle)],
        [0, 1, 0],
        [math.sin(Tip_angle), 0, math.cos(Tip_angle)]
    ]

    print("\nTip transformation matrix:")
    for i in range(3):
        for j in range(3):
            print(M2[i][j], end=" ")
        print()

    # Mount frame to platform frame transformation matrix
    PM = [[0.0 for _ in range(3)] for _ in range(3)]

    for i in range(3):
        for j in range(3):
            for k in range(3):
                PM[i][j] += M2[i][k] * M1[k][j]

    print("\nPM transformation matrix:")
    for i in range(3):
        for j in range(3):
            print(PM[i][j], end=" ")
        print()

    # Transpose PM to get Mount frame to platform frame
    PM_transpose = [[0.0 for _ in range(3)] for _ in range(3)]

    for i in range(3):
        for j in range(3):
            PM_transpose[i][j] = PM[j][i]

    print("\nPM_transpose matrix:")
    for i in range(3):
        for j in range(3):
            print(PM_transpose[i][j], end=" ")
        print()

    # Initialize the resulting 3x1 vector
    desiredVector = [0.0, 0.0, 0.0]

    # Perform the matrix-vector multiplication
    for i in range(3):
        for j in range(3):
            desiredVector[i] += PM_transpose[i][j] * platformVector[j]

    # Print the result
    print(f"python Resulting vector: [{desiredVector[0]:.2f}, {desiredVector[1]:.2f}, {desiredVector[2]:.2f}]")


# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
if __name__ == "__main__":
    sepGimbalTestFunction(
                 True,
                 [0.71, -0.50, 0.50],     # desiredThrustUnitVector1
                 [0, 0, 0],     # desiredThrustUnitVector2
                 [0, 0, 0],     # desiredThrustUnitVector3
                 1e-12   # accuracy
    )
