/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

*/

#include "twoAxisGimbal.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "architecture/utilities/macroDefinitions.h"
#include "linearInterpolation.hpp"
#include "bilinearInterpolation.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

TwoAxisGimbal::TwoAxisGimbal(std::string pathToTipTable, std::string pathToTiltTable) {
    // Read the motor-to-gimbal tip angle data file
    std::ifstream file(pathToTipTable);
    if (!file.is_open()) {
        _bskLog(this->bskLogger, BSK_ERROR, "Motor-to-gimbal tip angle interpolation table was not found.");
    }
    std::string line;
    int row = 0;
    while (getline(file, line) && row < 319) {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;
        while (getline(ss, cell, ',') && col < 319) {
            this->motor_to_gimbal_tip_angle[row][col] = DEG2RAD * stod(cell);
            col++;
        }
        row++;
    }
    file.close();

    // Read the motor-to-gimbal tilt angle data file
    std::ifstream file2(pathToTiltTable);
    if (!file2.is_open()) {
        _bskLog(this->bskLogger, BSK_ERROR, "Motor-to-gimbal tilt angle interpolation table was not found.");
    }
    row = 0;
    while (getline(file2, line) && row < 319) {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;
        while (getline(ss, cell, ',') && col < 319) {
            this->motor_to_gimbal_tilt_angle[row][col] = DEG2RAD * stod(cell);
            col++;
        }
        row++;
    }
    file2.close();
}

/*! This method checks the input messages to ensure they are linked. This method also resets module parameters to their
default values.
 @return void
 @param callTime [ns] Time the method is called
*/
void TwoAxisGimbal::Reset(uint64_t callTime) {
    if (!this->motor1StepCmdInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "twoAxisGimbal.motor1StepCmdInMsg wasn't connected.");
    }
    if (!this->motor2StepCmdInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "twoAxisGimbal.motor2StepCmdInMsg wasn't connected.");
    }
    if (!this->motor1StateInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "twoAxisGimbal.motor1StateInMsg wasn't connected.");
    }
    if (!this->motor2StateInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "twoAxisGimbal.motor2StateInMsg wasn't connected.");
    }

    // Set module parameter values for module reset
    this->gimbalPRVRotHat = {0.0, 0.0, 1.0};
    this->gimbalPRVTheta = 0.0;
    this->gimbalPRVThetaDot = 0.0;
    this->gimbalPRVThetaDDot = 0.0;
    this->previousWrittenTime = -1;
    this->newMsg = false;
    this->segment1Complete = false;
    this->segment2Complete = false;
    this->gimbalStepComplete = true;
    this->completion = true;
}

/*! This method determines the two-axis gimbal tip and tilt angles and profiles its prescribed hub-relative rotational
states using the two stepper motor input state messages. The gimbal tip and tilt angles and the hub-relative prescribed
rotational states are then written to the output messages.
 @return void
 @param callTime [ns] Time the method is called
*/
void TwoAxisGimbal::UpdateState(uint64_t callTime) {
    // Read the input messages
    MotorStepCommandMsgPayload motor1StepCmdIn = MotorStepCommandMsgPayload();
    MotorStepCommandMsgPayload motor2StepCmdIn = MotorStepCommandMsgPayload();
    StepperMotorMsgPayload motor1StateIn = StepperMotorMsgPayload();
    StepperMotorMsgPayload motor2StateIn = StepperMotorMsgPayload();

    if (this->motor1StepCmdInMsg.isWritten() && this->motor2StepCmdInMsg.isWritten()
                                             && this->motor1StateInMsg.isWritten()
                                             && this->motor2StateInMsg.isWritten()) {
        motor1StepCmdIn = this->motor1StepCmdInMsg();
        motor2StepCmdIn = this->motor2StepCmdInMsg();
        motor1StateIn = this->motor1StateInMsg();
        motor2StateIn = this->motor2StateInMsg();

        // Store the current motor angles
        this->motor1Theta = motor1StateIn.theta;
        this->motor2Theta = motor2StateIn.theta;

        if (this->previousWrittenTime < this->motor1StepCmdInMsg.timeWritten()) {
            this->previousWrittenTime = this->motor1StepCmdInMsg.timeWritten();

            // Store the motor steps commanded
            this->motor1StepsCommanded = motor1StepCmdIn.stepsCommanded;
            this->motor2StepsCommanded = motor2StepCmdIn.stepsCommanded;

            // Set up other module variables depending on motor actuation case
            if (this->motor1StepsCommanded != 0 || this->motor2StepsCommanded != 0) {
                this->completion = false;
                this->segment1Complete = false;
                this->segment2Complete = false;
                if ((this->motor1StepsCommanded == 0 || this->motor2StepsCommanded == 0) || (this->motor1StepsCommanded == this->motor2StepsCommanded)) {
                    this->segment2Complete = true;
                }
            } else {
                this->completion = true;
                this->gimbalPRVTheta = 0.0;
                this->gimbalPRV_F0M = this->motorAnglesToGimbalPRV(this->motor1Theta, this->motor2Theta);
            }
            this->newMsg = true;
        }
    }

    // Actuate the motor only if an active request is written
    if (!(this->completion)) {
        this->actuateGimbal(callTime * NANO2SEC);
    }

    // Write the module output messages
    this->writeOutputMessages(callTime);
}

/*! This high-level method is used to simulate the gimbal prv states in time.
 @return void
 @param t [s] Time the method is called
*/
void TwoAxisGimbal::actuateGimbal(double t) {
    // Reset the gimbal actuation parameters when the current request is complete and a new request is received
    if (this->newMsg && this->gimbalStepComplete) {
        this->resetGimbal(t);
    }

    // Update the gimbal rotation parameters after each gimbal step is completed
    if (this->gimbalStepComplete) {
        this->updateGimbalRotationParameters();
    }

    // Update the scalar gimbal states during each step
    if (this->isInGimbalStepFirstHalf(t)) {
        this->computeGimbalStepFirstHalf(t);
    } else if (this->isInGimbalStepSecondHalf(t)) {
        this->computeGimbalStepSecondHalf(t);
    } else {
        this->computeGimbalStepComplete(t);
    }
}

/*! This method resets the gimbal states when the current request is complete and a new request is received.
 @return void
 @param t [s] Time the method is called
*/
void TwoAxisGimbal::resetGimbal(double t) {
    // Reset the gimbal step count to zero
    this->gimbalStepCount = 0;

    // Update the initial time as the current simulation time
    this->tInit = t;

    // Find the initial gimbal attitude
    this->gimbalPRV_F0M = this->motorAnglesToGimbalPRV(this->motor1Theta, this->motor2Theta);

    // Compute the parameters to profile the gimbal actuation for each actuation segment
    this->computeGimbalActuationParameters();

    this->newMsg = false;
}

/*! This method determines the gimbal PRV hub-relative attitude given the stepper motor angles.
 @return Eigen::Vector3d
 @param motor1Angle [rad] Stepper motor 1 angle
 @param motor2Angle [rad] Stepper motor 2 angle
*/
Eigen::Vector3d TwoAxisGimbal::motorAnglesToGimbalPRV(double motor1Angle, double motor2Angle) {
    // Interpolate the gimbal angles from the given motor angles
    auto [gimbalTipAngle, gimbalTiltAngle] = this->motorAnglesToGimbalAngles(motor1Angle, motor2Angle);

    // Use the gimbal angles to determine the gimbal PRV attitude
    Eigen::Vector3d prvTip = gimbalTipAngle * this->gimbalRotHat1_M;
    Eigen::Vector3d prvTilt = gimbalTiltAngle * this->gimbalRotHat2_F;
    Eigen::Vector3d gimbalPRV = addPrv(prvTip, prvTilt);

    return gimbalPRV;
}

/*! This method determines the gimbal sequential tip and tilt angles given the stepper motor angles.
 @return std::pair<double, double>
 @param motor1Angle [rad] Stepper motor 1 angle
 @param motor2Angle [rad] Stepper motor 2 angle
*/
std::pair<double, double> TwoAxisGimbal::motorAnglesToGimbalAngles(double motor1Angle, double motor2Angle) {

    std::pair<double, double> gimbalAngles;

    if (this->bilinearInterpolationRequired(motor1Angle, motor2Angle)) {
        gimbalAngles = this->bilinearlyInterpolateGimbalAngles(motor1Angle, motor2Angle);
    } else if (this->noInterpolationRequired(motor1Angle, motor2Angle)) {
        double gimbalTipAngle = this->pullGimbalTipAngle(motor1Angle, motor2Angle);
        double gimbalTiltAngle = this->pullGimbalTiltAngle(motor1Angle, motor2Angle);
        gimbalAngles = {gimbalTipAngle, gimbalTiltAngle};
    } else if (this->linearInterpolationRequired(motor1Angle)) {
        gimbalAngles = this->linearlyInterpolateGimbalAnglesMotor1Fixed(motor1Angle, motor2Angle);
    } else {
        gimbalAngles = this->linearlyInterpolateGimbalAnglesMotor2Fixed(motor1Angle, motor2Angle);
    }

    return gimbalAngles;
}

/*! This method determines if no interpolation is required to obtain the gimbal angles.
 @return bool
 @param motor1Angle [rad] Stepper motor 1 angle
 @param motor2Angle [rad] Stepper motor 2 angle
*/
bool TwoAxisGimbal::noInterpolationRequired(double motor1Angle, double motor2Angle) {
    int compare1a = motor1Angle / this->tableStepAngle;
    double compare2a = motor1Angle / this->tableStepAngle;
    double compare3a = compare2a - compare1a;

    int compare1b = motor2Angle / this->tableStepAngle;
    double compare2b = motor2Angle / this->tableStepAngle;
    double compare3b = compare2b - compare1b;

    return (compare3a < 1e-10 && compare3b < 1e-10);
}

/*! This method determines if bilinear interpolation is required to obtain the gimbal angles.
 @return bool
 @param motor1Angle [rad] Stepper motor 1 angle
 @param motor2Angle [rad] Stepper motor 2 angle
*/
bool TwoAxisGimbal::bilinearInterpolationRequired(double motor1Angle, double motor2Angle) {
    int compare1a = motor1Angle / this->tableStepAngle;
    double compare2a = motor1Angle / this->tableStepAngle;
    double compare3a = compare2a - compare1a;

    int compare1b = motor2Angle / this->tableStepAngle;
    double compare2b = motor2Angle / this->tableStepAngle;
    double compare3b = compare2b - compare1b;

    return (!(compare3a < 1e-10) && !(compare3b < 1e-10));
}

/*! This method determines if linear interpolation is required to obtain the gimbal angles.
 @return bool
 @param motor1Angle [rad] Stepper motor 1 angle
 @param motor2Angle [rad] Stepper motor 2 angle
*/
bool TwoAxisGimbal::linearInterpolationRequired(double motorAngle) {
    int compare1 = motorAngle / this->tableStepAngle;
    double compare2 = motorAngle / this->tableStepAngle;
    double compare3 = compare2 - compare1;

    return (compare3 < 1e-10);
}

/*! This method calls a bilinear interpolation function to interpolate the motor angles to gimbal angles.
 @return std::pair<double, double>
 @param motor1Angle [rad] Stepper motor 1 angle
 @param motor2Angle [rad] Stepper motor 2 angle
*/
std::pair<double, double> TwoAxisGimbal::bilinearlyInterpolateGimbalAngles(double motor1Angle, double motor2Angle) {
    // Find the upper and lower interpolation table angle bounds for motor1 and motor 2
    double lowerMotor1Angle = this->tableStepAngle * floor(motor1Angle / this->tableStepAngle);
    double upperMotor1Angle = this->tableStepAngle * ceil(motor1Angle / this->tableStepAngle);
    double lowerMotor2Angle = this->tableStepAngle * floor(motor2Angle / this->tableStepAngle);
    double upperMotor2Angle = this->tableStepAngle * ceil(motor2Angle / this->tableStepAngle);

    // Bilinearly interpolate the gimbal tip angle
    double z11_tip = this->pullGimbalTipAngle(lowerMotor1Angle, lowerMotor2Angle);
    double z12_tip = this->pullGimbalTipAngle(lowerMotor1Angle, upperMotor2Angle);
    double z21_tip = this->pullGimbalTipAngle(upperMotor1Angle, lowerMotor2Angle);
    double z22_tip = this->pullGimbalTipAngle(upperMotor1Angle, upperMotor2Angle);
    double gimbalTipAngle = bilinearInterpolation(lowerMotor1Angle,
                                           upperMotor1Angle,
                                           lowerMotor2Angle,
                                           upperMotor2Angle,
                                           z11_tip,
                                           z12_tip,
                                           z21_tip,
                                           z22_tip,
                                           motor1Angle,
                                           motor2Angle);

    // Bilinearly interpolate the gimbal tilt angle
    double z11_tilt = this->pullGimbalTiltAngle(lowerMotor1Angle, lowerMotor2Angle);
    double z12_tilt = this->pullGimbalTiltAngle(lowerMotor1Angle, upperMotor2Angle);
    double z21_tilt = this->pullGimbalTiltAngle(upperMotor1Angle, lowerMotor2Angle);
    double z22_tilt = this->pullGimbalTiltAngle(upperMotor1Angle, upperMotor2Angle);
    double gimbalTiltAngle = bilinearInterpolation(lowerMotor1Angle,
                                            upperMotor1Angle,
                                            lowerMotor2Angle,
                                            upperMotor2Angle,
                                            z11_tilt,
                                            z12_tilt,
                                            z21_tilt,
                                            z22_tilt,
                                            motor1Angle,
                                            motor2Angle);

    std::pair<double, double> gimbalAngles = {gimbalTipAngle, gimbalTiltAngle};

    return gimbalAngles;
}

/*! This method calls a linear interpolation function to interpolate the motor angles to gimbal angles using a fixed motor 1 angle and bounded motor 2 angle.
 @return std::pair<double, double>
 @param motor1Angle [rad] Stepper motor 1 angle
 @param motor2Angle [rad] Stepper motor 2 angle
*/
std::pair<double, double> TwoAxisGimbal::linearlyInterpolateGimbalAnglesMotor1Fixed(double motor1Angle, double motor2Angle) {
    // Find the upper and lower interpolation table angle bounds for the bounded motor angle
    double lowerMotorAngle = this->tableStepAngle * floor(motor2Angle / this->tableStepAngle);
    double upperMotorAngle = this->tableStepAngle * ceil(motor2Angle / this->tableStepAngle);

    // Linearly interpolate the gimbal tilt angle
    double y1_tip = this->pullGimbalTipAngle(motor1Angle, lowerMotorAngle);
    double y2_tip = this->pullGimbalTipAngle(motor1Angle, upperMotorAngle);
    double gimbalTipAngle = linearInterpolation(lowerMotorAngle, upperMotorAngle, y1_tip, y2_tip, motor2Angle);

    // Linearly interpolate the gimbal tilt angle
    double y1_tilt = this->pullGimbalTiltAngle(motor1Angle, lowerMotorAngle);
    double y2_tilt = this->pullGimbalTiltAngle(motor1Angle, upperMotorAngle);
    double gimbalTiltAngle = linearInterpolation(lowerMotorAngle, upperMotorAngle, y1_tilt, y2_tilt, motor2Angle);

    std::pair<double, double> gimbalAngles = {gimbalTipAngle, gimbalTiltAngle};

    return gimbalAngles;
}

/*! This method calls a linear interpolation function to interpolate the motor angles to gimbal angles using a fixed motor 2 angle and bounded motor 1 angle.
 @return std::pair<double, double>
 @param motor1Angle [rad] Stepper motor 1 angle
 @param motor2Angle [rad] Stepper motor 2 angle
*/
std::pair<double, double> TwoAxisGimbal::linearlyInterpolateGimbalAnglesMotor2Fixed(double motor1Angle, double motor2Angle) {
    // Find the upper and lower interpolation table angle bounds for the bounded motor angle
    double lowerMotorAngle = this->tableStepAngle * floor(motor1Angle / this->tableStepAngle);
    double upperMotorAngle = this->tableStepAngle * ceil(motor1Angle / this->tableStepAngle);

    // Linearly interpolate the gimbal tilt angle
    double y1_tip = this->pullGimbalTipAngle(lowerMotorAngle, motor2Angle);
    double y2_tip = this->pullGimbalTipAngle(upperMotorAngle, motor2Angle);
    double gimbalTipAngle = linearInterpolation(lowerMotorAngle, upperMotorAngle, y1_tip, y2_tip, motor1Angle);

    // Linearly interpolate the gimbal tilt angle
    double y1_tilt = this->pullGimbalTiltAngle(lowerMotorAngle, motor2Angle);
    double y2_tilt = this->pullGimbalTiltAngle(upperMotorAngle, motor2Angle);
    double gimbalTiltAngle = linearInterpolation(lowerMotorAngle, upperMotorAngle, y1_tilt, y2_tilt, motor1Angle);

    std::pair<double, double> gimbalAngles = {gimbalTipAngle, gimbalTiltAngle};

    return gimbalAngles;
}

/*! Method used to compute and update the gimbal actuation parameters for each segment of required gimbal motion
 @return void
*/
void TwoAxisGimbal::computeGimbalActuationParameters() {
    if (!this->segment1Complete && this->segment2Complete) {
        this->computeSingleSegmentParameters();
    } else if (!this->segment1Complete) {
        this->computeSegment1Parameters();
    } else {
        this->computeSegment2Parameters();
    }

    this->gimbalStepAngle = fabs(this->gimbalPRVThetaRef / this->gimbalStepsCommanded);
    this->gimbalPRVThetaDDotMax = (4 * this->gimbalStepAngle) / (this->motorStepTime * this->motorStepTime);
}

/*! This method computes the gimbal actuation parameters for a single actuation segment (motors actuate the same number of steps).
 @return void
*/
void TwoAxisGimbal::computeSingleSegmentParameters() {
    if (this->motor1StepsCommanded == this->motor2StepsCommanded) { // Actuate both motors
        this->gimbalStepsCommanded = this->motor1StepsCommanded;
        this->motor1ThetaRef = this->motor1Theta + this->gimbalStepsCommanded * this->motorStepAngle;
        this->motor2ThetaRef = this->motor2Theta + this->gimbalStepsCommanded * this->motorStepAngle;
    } else if (this->motor2StepsCommanded == 0) {  // Actuate motor 1 only
        this->gimbalStepsCommanded = this->motor1StepsCommanded;
        this->motor1ThetaRef = this->motor1Theta + this->gimbalStepsCommanded * this->motorStepAngle;
        this->motor2ThetaRef = this->motor2Theta;
    } else {  // Actuate motor 2 only
        this->gimbalStepsCommanded = this->motor2StepsCommanded;
        this->motor1ThetaRef = this->motor1Theta;
        this->motor2ThetaRef = this->motor2Theta + this->gimbalStepsCommanded * this->motorStepAngle;
    }

    // Interpolate to find the reference gimbal attitude prv_FM
    Eigen::Vector3d gimbalPRV_FM = motorAnglesToGimbalPRV(this->motor1ThetaRef, this->motor2ThetaRef);

    // Find the relative gimbal prv for the rotation prv_FF0
    Eigen::Vector3d gimbalPRV_FF0 = subPrv(gimbalPRV_FM, this->gimbalPRV_F0M);

    // Find the prv angle and axis for the rotation
    if (this->gimbalStepsCommanded < 0) {
        this->gimbalPRVThetaRef = - gimbalPRV_FF0.norm();
        this->gimbalPRVRotHat = -1 * gimbalPRV_FF0 / gimbalPRV_FF0.norm();
    } else {
        this->gimbalPRVThetaRef = gimbalPRV_FF0.norm();
        this->gimbalPRVRotHat = gimbalPRV_FF0 / this->gimbalPRVThetaRef;
    }
}

/*! This method computes the gimbal actuation parameters for the first of two actuation segments (motors actuate different number of steps).
 @return void
*/
void TwoAxisGimbal::computeSegment1Parameters() {
    if (fabs(this->motor1StepsCommanded) > fabs(this->motor2StepsCommanded)) {
        this->gimbalStepsCommanded = this->motor2StepsCommanded;
        this->motor1ThetaRef = this->motor1Theta + this->gimbalStepsCommanded * this->motorStepAngle;
        this->motor2ThetaRef = this->motor2Theta + this->gimbalStepsCommanded * this->motorStepAngle;
    } else {
        this->gimbalStepsCommanded = this->motor1StepsCommanded;
        this->motor1ThetaRef = this->motor1Theta + this->gimbalStepsCommanded * this->motorStepAngle;
        this->motor2ThetaRef = this->motor2Theta + this->gimbalStepsCommanded * this->motorStepAngle;
    }

    // Interpolate to find the intermediate reference gimbal attitude prv_FIntM
    this->gimbalPRV_FIntM = motorAnglesToGimbalPRV(this->motor1ThetaRef, this->motor2ThetaRef);

    // Find the relative gimbal prv for the rotation prv_FF0
    Eigen::Vector3d gimbalPRV_FIntF0 = subPrv(this->gimbalPRV_FIntM, this->gimbalPRV_F0M);

    // Find the prv angle and axis for the rotation
    if (this->gimbalStepsCommanded < 0) {
        this->gimbalPRVThetaRef = - gimbalPRV_FIntF0.norm();
        this->gimbalPRVRotHat = -1 * gimbalPRV_FIntF0 / gimbalPRV_FIntF0.norm();
    } else {
        this->gimbalPRVThetaRef = gimbalPRV_FIntF0.norm();
        this->gimbalPRVRotHat = gimbalPRV_FIntF0 / this->gimbalPRVThetaRef;
    }
}

/*! This method computes the gimbal actuation parameters for the second of two actuation segments (motors actuate different number of steps).
 @return void
*/void TwoAxisGimbal::computeSegment2Parameters() {
    if (fabs(this->motor1StepsCommanded) > fabs(this->motor2StepsCommanded)) {
        this->gimbalStepsCommanded = (this->motor1StepsCommanded - this->motor2StepsCommanded);
        this->motor1ThetaRef = this->motor1ThetaRef + this->gimbalStepsCommanded * this->motorStepAngle;
    } else {
        this->gimbalStepsCommanded = (this->motor2StepsCommanded - this->motor1StepsCommanded);
        this->motor2ThetaRef = this->motor2ThetaRef + this->gimbalStepsCommanded * this->motorStepAngle;
    }

    // Interpolate to find the reference gimbal attitude prv_FM
    Eigen::Vector3d gimbalPRV_FM = motorAnglesToGimbalPRV(this->motor1ThetaRef, this->motor2ThetaRef);

    // Find the relative gimbal prv for the rotation prv_FFInt
    Eigen::Vector3d gimbalPRV_FFInt = subPrv(gimbalPRV_FM, this->gimbalPRV_FIntM);

    // Find the prv angle and axis for the rotation
    if (this->gimbalStepsCommanded < 0) {
        this->gimbalPRVThetaRef = - gimbalPRV_FFInt.norm();
        this->gimbalPRVRotHat = -1 * gimbalPRV_FFInt / gimbalPRV_FFInt.norm();
    } else {
        this->gimbalPRVThetaRef = gimbalPRV_FFInt.norm();
        this->gimbalPRVRotHat = gimbalPRV_FFInt / this->gimbalPRVThetaRef;
    }

    this->gimbalPRV_F0M = this->gimbalPRV_FIntM;
}

/*! This method pulls a specific gimbal tip angle from the tip interpolation table given specific motor angles.
 @return double
 @param motor1Angle [rad] Stepper motor 1 angle
 @param motor2Angle [rad] Stepper motor 2 angle
*/
double TwoAxisGimbal::pullGimbalTipAngle(double motor1Angle, double motor2Angle) {
    int motor1Idx = motor1Angle / this->tableStepAngle;
    int motor2Idx = motor2Angle / this->tableStepAngle;
    return this->motor_to_gimbal_tip_angle[motor2Idx][motor1Idx];
}

/*! This method pulls a specific gimbal tilt angle from the tilt interpolation table given specific motor angles.
 @return double
 @param motor1Angle [rad] Stepper motor 1 angle
 @param motor2Angle [rad] Stepper motor 2 angle
*/
double TwoAxisGimbal::pullGimbalTiltAngle(double motor1Angle, double motor2Angle) {
    int motor1Idx = motor1Angle / this->tableStepAngle;
    int motor2Idx = motor2Angle / this->tableStepAngle;
    return this->motor_to_gimbal_tilt_angle[motor2Idx][motor1Idx];
}

/*! This method updates the gimbal rotation parameters after a gimbal step is completed.
 @return void
*/
void TwoAxisGimbal::updateGimbalRotationParameters() {
    this->tf = this->tInit + this->motorStepTime;
    this->ts = this->tInit + this->motorStepTime / 2;

    this->intermediateGimbalPRVThetaInit = this->gimbalStepCount * this->gimbalStepAngle;
    if (this->gimbalStepsCommanded > 0) {
        this->intermediateGimbalPRVThetaRef = (this->gimbalStepCount + 1) * this->gimbalStepAngle;
        this->a = 0.5 * (this->gimbalStepAngle) / ((this->ts - this->tInit) * (this->ts - this->tInit));
        this->b = -0.5 * (this->gimbalStepAngle) / ((this->ts - this->tf) * (this->ts - this->tf));
    } else {
        this->intermediateGimbalPRVThetaRef = (this->gimbalStepCount - 1) * this->gimbalStepAngle;
        this->a = 0.5 * (-this->gimbalStepAngle) / ((this->ts - this->tInit) * (this->ts - this->tInit));
        this->b = -0.5 * (-this->gimbalStepAngle) / ((this->ts - this->tf) * (this->ts - this->tf));
    }
}

/*! This method determines if the gimbal is in the first half of a step.
 @return bool
 @param t [s] Time the method is called
*/
bool TwoAxisGimbal::isInGimbalStepFirstHalf(double t) {
    return (t < this->ts || t == this->ts) && (this->tf - this->tInit != 0.0);
}

/*! This method computes the gimbal states during the first half of each step.
 @return void
 @param t [s] Time the method is called
*/
void TwoAxisGimbal::computeGimbalStepFirstHalf(double t) {
    if (this->gimbalStepsCommanded > 0 && !this->newMsg) {
        this->gimbalPRVThetaDDot = this->gimbalPRVThetaDDotMax;
    } else if (!this->newMsg) {
        this->gimbalPRVThetaDDot = -this->gimbalPRVThetaDDotMax;
    }
    this->gimbalPRVThetaDot = this->gimbalPRVThetaDDot * (t - this->tInit);
    this->gimbalPRVTheta = this->a * (t - this->tInit) * (t - this->tInit) + this->intermediateGimbalPRVThetaInit;
    this->gimbalStepComplete = false;
}

/*! This method determines if the gimbal is in the second half of a step.
 @return bool
 @param t [s] Time the method is called
*/
bool TwoAxisGimbal::isInGimbalStepSecondHalf(double t) {
    return (t > this->ts && t < this->tf) && (this->tf - this->tInit != 0.0);
}

/*! This method computes the gimbal prv states during the second half of each step.
 @return void
 @param t [s] Time the method is called
*/
void TwoAxisGimbal::computeGimbalStepSecondHalf(double t) {
    if (this->gimbalStepsCommanded > 0 && !this->newMsg){
        this->gimbalPRVThetaDDot = -this->gimbalPRVThetaDDotMax;
    } else if (!this->newMsg) {
        this->gimbalPRVThetaDDot = this->gimbalPRVThetaDDotMax;
    }
    this->gimbalPRVThetaDot = this->gimbalPRVThetaDDot * (t - this->tInit) - this->gimbalPRVThetaDDot * (this->tf - this->tInit);
    this->gimbalPRVTheta = this->b * (t - this->tf) * (t - this->tf) + this->intermediateGimbalPRVThetaRef;
    this->gimbalStepComplete = false;
}

/*! This method computes the gimbal prv states when a step is complete.
 @return void
 @param t [s] Time the method is called
*/
void TwoAxisGimbal::computeGimbalStepComplete(double t) {
    this->gimbalStepComplete = true;
    this->gimbalPRVThetaDDot = 0.0;
    this->gimbalPRVThetaDot = 0.0;
    this->gimbalPRVTheta = this->intermediateGimbalPRVThetaRef;

    // Update the motor step count
    if (!this->newMsg) {
        if (this->gimbalStepsCommanded > 0) {
            this->gimbalStepCount++;
        } else {
            this->gimbalStepCount--;
        }
    } else {
        if (this->intermediateGimbalPRVThetaRef > this->intermediateGimbalPRVThetaInit) {
            this->gimbalStepCount++;
        } else {
            this->gimbalStepCount--;
        }
    }

    // Update the initial time
    this->tInit = t;

    // Update the completion boolean variable only when motor actuation is complete
    if ((this->gimbalStepCount == this->gimbalStepsCommanded) && !this->newMsg && !this->segment1Complete) {
        this->segment1Complete = true;
        this->newMsg = true;
        if (this->segment2Complete) {
            this->completion = true;
        }
    } else if ((this->gimbalStepCount == this->gimbalStepsCommanded) && !this->newMsg && this->segment1Complete) {
        this->segment2Complete = true;
        this->completion = true;
    }
}

/*! This method writes the module output messages and computes the output message data.
 @return void
 @param callTime [s] Time the method is called
*/
void TwoAxisGimbal::writeOutputMessages(uint64_t callTime) {
    // Create the output buffer messages
    TwoAxisGimbalMsgPayload twoAxisGimbalOut = TwoAxisGimbalMsgPayload();
    PrescribedRotationMsgPayload prescribedRotationOut = PrescribedRotationMsgPayload();

    // Compute the angular velocity and angular acceleration of frame F wrt frame M in F frame components
    Eigen::Vector3d omega_FM_F = this->gimbalPRVThetaDot * this->gimbalPRVRotHat;  // [rad/s]
    Eigen::Vector3d omegaPrime_FM_F = this->gimbalPRVThetaDDot * this->gimbalPRVRotHat;  // [rad/s^2]

    // Determine the DCM dcm_FM representing the current gimbal attitude relative to the mount frame
    Eigen::Vector3d relativePRV = this->gimbalPRVTheta * this->gimbalPRVRotHat;
    Eigen::Vector3d prv_FM = addPrv(this->gimbalPRV_F0M, relativePRV);
    Eigen::Matrix3d dcm_FM = prvToDcm(prv_FM);

    // Compute the MRP sigma_FM representing the current gimbal attitude relative to the mount frame
    Eigen::Vector3d sigma_FM = dcmToMrp(dcm_FM);

    // Determine the gimbal tip and tilt angles and write them to the output buffer message
    twoAxisGimbalOut.theta1 = atan(dcm_FM(1,2) / dcm_FM(1,1));
    twoAxisGimbalOut.theta2 = atan(dcm_FM(2,0) / dcm_FM(0,0));

    // Write the prescribed motion output buffer message
    eigenVector3d2CArray(omega_FM_F, prescribedRotationOut.omega_FM_F);
    eigenVector3d2CArray(omegaPrime_FM_F, prescribedRotationOut.omegaPrime_FM_F);
    eigenVector3d2CArray(sigma_FM, prescribedRotationOut.sigma_FM);

    // Write the output messages
    this->twoAxisGimbalOutMsg.write(&twoAxisGimbalOut, moduleID, callTime);
    this->prescribedRotationOutMsg.write(&prescribedRotationOut, moduleID, callTime);
}

/*! Setter method for the motor step angle.
 @return void
 @param stepAngle [rad] Motor step angle
*/
void TwoAxisGimbal::setMotorStepAngle(const double stepAngle) {
    this->motorStepAngle = stepAngle;
}

/*! Setter method for the motor step time.
 @return void
 @param stepTime [s] Motor step time
*/
void TwoAxisGimbal::setMotorStepTime(const double stepTime) {
    this->motorStepTime = stepTime;
}

/*! Getter method for the gimbal rotation axis 1.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d &TwoAxisGimbal::getGimbalRotHat1_M() const {
    return this->gimbalRotHat1_M;
}

/*! Getter method for the gimbal rotation axis 2.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d &TwoAxisGimbal::getGimbalRotHat2_F() const {
    return this->gimbalRotHat2_F;
}

/*! Getter method for the motor step angle.
 @return double
*/
double TwoAxisGimbal::getMotorStepAngle() const {
    return this->motorStepAngle;
}

/*! Getter method for the motor step time.
 @return double
*/
double TwoAxisGimbal::getMotorStepTime() const {
    return this->motorStepTime;
}

/*! Method to load the motor-to-gimbal tip angle lookup table.
 @return void
 @param pathToTipTable String for the file absolute path
*/
void TwoAxisGimbal::loadMotorToGimbalTipAngleLookupTable(std::string pathToTipTable) {
    std::ifstream file(pathToTipTable);
    if (!file.is_open()) {
        _bskLog(this->bskLogger, BSK_ERROR, "Motor-to-gimbal tip angle interpolation table was not found.");
    }
    std::string line;
    int row = 0;
    while (getline(file, line) && row < 319) {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;
        while (getline(ss, cell, ',') && col < 319) {
            this->motor_to_gimbal_tip_angle[row][col] = DEG2RAD * stod(cell);
            col++;
        }
        row++;
    }
    file.close();
}

/*! Method to load the motor-to-gimbal tilt angle lookup table.
 @return void
 @param pathToTiltTable String for the file absolute path
*/
void TwoAxisGimbal::loadMotorToGimbalTiltAngleLookupTable(std::string pathToTiltTable) {
    std::ifstream file(pathToTiltTable);
    if (!file.is_open()) {
        _bskLog(this->bskLogger, BSK_ERROR, "Motor-to-gimbal tilt angle interpolation table was not found.");
    }
    std::string line;
    int row = 0;
    while (getline(file, line) && row < 319) {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;
        while (getline(ss, cell, ',') && col < 319) {
            this->motor_to_gimbal_tilt_angle[row][col] = DEG2RAD * stod(cell);
            col++;
        }
        row++;
    }
    file.close();
}
