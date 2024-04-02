Executive Summary
-----------------
This document describes the stepper motor's module in Basilsk software. The purpose of this module is to control stepper motors by inputting a desired motor angle and compute the number of steps required to reach that angle.

Message Connection Descriptions
-------------------------------
.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - spinningBodyInMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - Input message of desired motor angle (theta)
    * - motorStepCountOutMsg
      - :ref:`MotorStepCountMsgPayload`
      - Output message of steps commanded (numSteps) 

Detailed Module Description
---------------------------
The stepperMotor module calculates and manages the amount of motor steps needed to achieve a given angle and tracks the motor's actuation over time. The module receives an input message (spinningBodyInMsg) with the desired theta, preforms the calculation and output the steps needed by  (motorStepCountOutMsg). 

This module computes the delta angle by finding the difference between the desired angle and the current motor angle. The module uses the ceil rounding strategy for the current motor angle to ensure that its aligned with the nearest step boundary. This precision is essential because stepper motors can only move in whole steps. If the module is interrupted during execution, the motor is required to complete the step it's currently within before making any further movements. This approach minimizes deviations from the desired angle and guarantees that the motor always moves in whole steps.      
        
        :code: configData->deltaAngle = confiData->desiredAngle - (ceil(configData->currentMotorAngle / configData->stepAngle) * configData->stepAngle);

Stepper motors can’t except half steps dynamically but the simulation itself can end up calculating steps with fractional parts due to angle divisions and possible message interruptions during steps executions. Because of this, the code evaluates whether the fractional steps should be ceiled if they are close to the step's finish or floored if they are at the step's beginning. This rounding mechanism helps in commanding the appropriate number of steps to ensure that the motor’s final position is as close as possible to the desired angle and that only whole steps are captured as a stepsCommanded value.
        
        :code: double tempStepsCommanded = configData->deltaAngle / configData->stepAngle;
        'if ((ceil(tempStepsCommanded) - tempStepsCommanded) > (tempStepsCommanded - floor(tempStepsCommanded))) {
            configData->stepsCommanded = floor(tempStepsCommanded);
        } else {
            configData->stepsCommanded = ceil(tempStepsCommanded);
        }'

The module in the end updates the motor's status to ensure that it accurately reported the number of steps commanded and reached the desired angle. During the simulation execution, if the module exceeded the motor’s desired angle, it sets the stepsTaken (determines the number of whole steps completed) to match the stepsCommanded value and sets the current angle to match the desired angle. This logic guarantees precise control over the motor in maintaining the commanded steps and angles.
