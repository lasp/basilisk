import pytest
import inspect
import matplotlib.pyplot as plt
import numpy as np
import os
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import sepGimbalLookUpTable
from Basilisk.fswAlgorithms import stepperMotorController
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Vary the thrust unit vector angle for pytest
# Shamsa: thetaint is gonna execute twice, once in 0 and once in 2*np.pi/3

# @pytest.mark.parametrize("desiredAngle", [0, 2*np.pi/3])
@pytest.mark.parametrize("accuracy", [1e-12])
@pytest.mark.parametrize("stepAngle", [1])
@pytest.mark.parametrize("stepTime", [1])
@pytest.mark.parametrize("initialMotor1Angle", [0, 0.01, 0.5])
@pytest.mark.parametrize("inputTipAngle", [17.9, 15, -12])
@pytest.mark.parametrize("initialMotor2Angle", [5])
@pytest.mark.parametrize("inputTiltAngle", [0.4, -3, 0.0])
def test_senarioTestFunction(show_plots, inputTipAngle, inputTiltAngle, stepAngle, stepTime, initialMotor1Angle, initialMotor2Angle, accuracy):
    """
    **Validation Test Description**

    This unit test ensures that the stepper motor is properly computed, where
    the input of desired angle will give us the right number of motor steps.
    **Test Parameters**

    Args:
        desiredAngle (float): [rad] desired angle value
        thetaInit (float): [rad] Initial PRV angle of the F frame with respect
        to the M frame
        thetaRef (float): [rad] Reference PRV angle of the F frame with respect
        to the M frame
        thetaDDotMax (float): [rad/s^2] Maximum angular acceleration for the
        attitude maneuver

    **Description of Variables Being Tested**

    This unit test ensures that the profiled 1 DOF rotational attitude maneuver
    is properly computed for a series of initial and reference PRV angles and
    maximum angular accelerations. The final prescribed angle
    ``theta_FM_Final`` and angular velocity magnitude ``thetaDot_Final`` are
    compared with the reference values ``theta_Ref`` and ``thetaDot_Ref``,
    respectively.
    """
    [testResults, testMessage] = senarioTestFunction(show_plots, inputTipAngle, inputTiltAngle, stepAngle, stepTime, initialMotor1Angle, initialMotor2Angle, accuracy)

    # assertion statement checks whether a given condition is True.
    # If the condition is False, an AssertionError is raised, and an optional
    # error message (testMessage) can be provided to explain the reason for the
    # assertion failure.
    assert testResults < 1, testMessage


def senarioTestFunction(show_plots, desiredTipAngle1, desiredTiltAngle1, stepAngle, stepTime, initialMotor1Angle, initialMotor2Angle, accuracy):
    testFailCount = 0  # Zero the unit test result counter
    testMessages = []  # Create an empty array to store the test log messages
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create the test thread
    testProcessRate = macros.sec2nano(0.1)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the sepGimbalLookUpTable module to be tested
    SepGimbalLookUpTableConfig = sepGimbalLookUpTable.SepGimbalLookUpTableConfig()
    SepGimbalLookUpTableWrap = unitTestSim.setModelDataWrap(SepGimbalLookUpTableConfig)
    SepGimbalLookUpTableWrap.ModelTag = "sepGimbalLookUpTable"

    # Add the sepGimbal test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, SepGimbalLookUpTableWrap, SepGimbalLookUpTableConfig)

    # Create variable for the tip and tilt angles and input message
    HingedRigidBodyTipAngleData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyTipAngleData.theta = desiredTipAngle1
    HingedRigidBodyTipAngleMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyTipAngleData)
    SepGimbalLookUpTableConfig.desiredGimbalTipAngleInMsg.subscribeTo(HingedRigidBodyTipAngleMessage)

    HingedRigidBodyTiltAngleData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyTiltAngleData.theta = desiredTiltAngle1
    HingedRigidBodyTiltAngleMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyTiltAngleData)
    SepGimbalLookUpTableConfig.desiredGimbalTiltAngleInMsg.subscribeTo(HingedRigidBodyTiltAngleMessage)

    # CSV File
    SepGimbalLookUpTableConfig.fileName = ("C:\\Users\\ShamsaS\\Documents\\software-projects\\basilisk-lasp\\src\\fswAlgorithms\\effectorInterfaces\\sepGimbalLookUpTable\\platformAngle_motorAngle.csv")

    # Log the lookup table output messages for data comparison
    motor1AngleMsgLog = SepGimbalLookUpTableConfig.motor1AngleOutMsg.recorder(testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, motor1AngleMsgLog)
    motor2AngleMsgLog = SepGimbalLookUpTableConfig.motor2AngleOutMsg.recorder(testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, motor2AngleMsgLog)

    # Create an instance of the stepperMotorController module to be tested
    StepperMotor1Controller = stepperMotorController.stepperMotorController()
    StepperMotor1Controller.ModelTag = "stepperMotor1Controller"
    StepperMotor1Controller.stepAngle = stepAngle
    StepperMotor1Controller.stepTime = stepTime
    StepperMotor1Controller.initAngle = initialMotor1Angle
    StepperMotor1Controller.currentAngle = initialMotor1Angle

    StepperMotor2Controller = stepperMotorController.stepperMotorController()
    StepperMotor2Controller.ModelTag = "stepperMotor2Controller"
    StepperMotor2Controller.stepAngle = stepAngle
    StepperMotor2Controller.stepTime = stepTime
    StepperMotor2Controller.initAngle = initialMotor2Angle
    StepperMotor2Controller.currentAngle = initialMotor2Angle

    # Create the stepperMotorController input message for motor 2
    HingedRigidBodyMessageData1 = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData1.theta = 11.98
    HingedRigidBodyMessage1 = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData1)
    StepperMotor1Controller.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage1)

    HingedRigidBodyMessageData2 = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData2.theta = 0.34
    HingedRigidBodyMessage2 = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData2)
    StepperMotor2Controller.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage2)

    # Add the stepperMotor test module to the runtime call list
    unitTestSim.AddModelToTask(unitTaskName, StepperMotor1Controller)
    unitTestSim.AddModelToTask(unitTaskName, StepperMotor2Controller)

    # Log the stepper motor output messages for data comparison
    motor1StepCommandLog = StepperMotor1Controller.motorStepCommandOutMsg.recorder(testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, motor1StepCommandLog)
    motor2StepCommandLog = StepperMotor2Controller.motorStepCommandOutMsg.recorder(testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, motor2StepCommandLog)

    # Subscribe the stepper motor controllers to the lookup table outputs
    StepperMotor1Controller.spinningBodyInMsg.subscribeTo(SepGimbalLookUpTableConfig.motor1AngleOutMsg)
    StepperMotor2Controller.spinningBodyInMsg.subscribeTo(SepGimbalLookUpTableConfig.motor2AngleOutMsg)

    # Set the simulation time
    unitTestSim.ConfigureStopTime(macros.sec2nano(30))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()

    # Pull the logged motor step data
    stepsCommanded1 = motor1StepCommandLog.stepsCommanded
    stepsCommanded2 = motor2StepCommandLog.stepsCommanded


    # # Calculate the required number of steps for validation
    # if initialMotor1Angle > 0:
    #     trueNumSteps1 = (desiredMotor1Angle - (np.ceil(initialMotor1Angle / stepAngle) * stepAngle)) / stepAngle
    # else:
    #     trueNumSteps1 = (desiredMotor1Angle - (np.floor(initialMotor1Angle / stepAngle) * stepAngle)) / stepAngle

    # if initialMotor2Angle > 0:
    #     trueNumSteps2 = (desiredMotor2Angle - (np.ceil(initialMotor2Angle / stepAngle) * stepAngle)) / stepAngle
    # else:
    #     trueNumSteps2 = (desiredMotor2Angle - (np.floor(initialMotor2Angle / stepAngle) * stepAngle)) / stepAngle

    print("Initial Parameters:")
    print(f"  Step Angle: {stepAngle}")
    print(f"  Step Time: {stepTime}")
    print(f"  Initial Motor 1 Angle: {initialMotor1Angle}")
    print(f"  Initial Motor 2 Angle: {initialMotor2Angle}")
    # print(f"  Accuracy: {accuracy}")
    # print("\\ Messages and Subscriptions:")
    # print(f"  HingedRigidBodyTipAngleMessage: {HingedRigidBodyTipAngleMessage}")
    # print(f"  HingedRigidBodyTiltAngleMessage: {HingedRigidBodyTiltAngleMessage}")
    # print(f"  SepGimbalLookUpTableConfig.fileName: {SepGimbalLookUpTableConfig.fileName}")
    # print(f"  StepperMotor1Controller.spinningBodyInMsg.subscriptions: {StepperMotor1Controller.spinningBodyInMsg}")
    # print(f"  StepperMotor2Controller.spinningBodyInMsg.subscriptions: {StepperMotor2Controller.spinningBodyInMsg}")
    print("\\ Motor Commands:")
    print(f"  Commanded steps for Motor 1: {stepsCommanded1}")
    print(f"  Commanded steps for Motor 2: {stepsCommanded2}")
    print("")


    # Return the logs
    return [testFailCount, ''.join(testMessages)]


# This statement below ensures that the unitTestScript can be run as a
# stand-alone python script
if __name__ == "__main__":
    test_senarioTestFunction(
        True,
        17.9,  # desiredTipAngle1
        0.4,  # desiredTiltAngle1
        1,  # step angle
        1,  # step time
        10,  # initialMotor1Angle
        5,  # initialMotor2Angle
        1e-12  # accuracy
    )
