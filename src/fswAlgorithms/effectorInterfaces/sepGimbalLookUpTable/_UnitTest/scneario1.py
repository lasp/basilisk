import pytest
import inspect
import os
import numpy as np
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import sepGimbalLookUpTable
from Basilisk.fswAlgorithms import stepperMotorController
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'

@pytest.mark.parametrize("accuracy", [1e-12])
def test_combinedScenario(accuracy):
    combinedScenario(accuracy)

def combinedScenario(accuracy):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the sepGimbalLookUpTable module algorithm
    SepGimbalLookUpTableConfig = sepGimbalLookUpTable.SepGimbalLookUpTableConfig()
    SepGimbalLookUpTableWrap = unitTestSim.setModelDataWrap(SepGimbalLookUpTableConfig)
    SepGimbalLookUpTableWrap.ModelTag = "sepGimbalLookUpTable"
    unitTestSim.AddModelToTask(unitTaskName, SepGimbalLookUpTableWrap, SepGimbalLookUpTableConfig)

    # Create an instance of the stepperMotorController module algorithm
    motor1ControllerConfig = stepperMotorController.StepperMotorControllerConfig()
    motor1ControllerWrap = unitTestSim.setModelDataWrap(motor1ControllerConfig)
    motor1ControllerWrap.ModelTag = "stepperMotor1Controller"
    unitTestSim.AddModelToTask(unitTaskName, motor1ControllerWrap, motor1ControllerConfig)

    motor2ControllerConfig = stepperMotorController.StepperMotorControllerConfig()
    motor2ControllerWrap = unitTestSim.setModelDataWrap(motor2ControllerConfig)
    motor2ControllerWrap.ModelTag = "stepperMotor2Controller"
    unitTestSim.AddModelToTask(unitTaskName, motor2ControllerWrap, motor2ControllerConfig)

    # Log the test module output message for data comparison
    motor1AngleMsgLog = SepGimbalLookUpTableConfig.motor1AngleOutMsg.recorder()
    motor2AngleMsgLog = SepGimbalLookUpTableConfig.motor2AngleOutMsg.recorder()
    motor1StepCommandLog = motor1ControllerConfig.motorStepCommandOutMsg.recorder()
    motor2StepCommandLog = motor2ControllerConfig.motorStepCommandOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, motor1AngleMsgLog)
    unitTestSim.AddModelToTask(unitTaskName, motor2AngleMsgLog)
    unitTestSim.AddModelToTask(unitTaskName, motor1StepCommandLog)
    unitTestSim.AddModelToTask(unitTaskName, motor2StepCommandLog)

    # Initialize simulation
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(30))
    unitTestSim.ExecuteSimulation()

    # Retrieve the step angle and step time from the stepperMotorController configuration
    stepAngle = motor1ControllerConfig.stepAngle
    stepTime = motor1ControllerConfig.stepTime

    # Retrieve the final motor angles
    finalMotor1Angle = motor1AngleMsgLog.theta[-1]
    finalMotor2Angle = motor2AngleMsgLog.theta[-1]

    # Calculate the desired motor angles based on the final angles and step angle
    desiredMotor1Angle = np.round(finalMotor1Angle / stepAngle) * stepAngle
    desiredMotor2Angle = np.round(finalMotor2Angle / stepAngle) * stepAngle

    # Print the results
    print(f"Desired Motor 1 Angle: {desiredMotor1Angle}")
    print(f"Desired Motor 2 Angle: {desiredMotor2Angle}")
    print(f"Steps Commanded for Motor 1: {motor1StepCommandLog.stepsCommanded[-1]}")
    print(f"Steps Commanded for Motor 2: {motor2StepCommandLog.stepsCommanded[-1]}")

if __name__ == "__main__":
    test_combinedScenario(1e-12)
