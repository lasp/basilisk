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
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
using namespace std;
const double DEG2RAD = M_PI / 180.0;

TwoAxisGimbal::TwoAxisGimbal() {
    // Read the motor-to-gimbal tip angle data file
    ifstream file("/Users/leahkiner/Repositories/LASP_Basilisk/src/simulation/deviceInterface/twoAxisGimbal/motor_to_gimbal_tip_angle.csv");
    if (!file.is_open()) {
        cout << "Error opening tip file!" << endl;
    }
    string line;
    int row = 0;
    while (getline(file, line) && row < 320) {
        stringstream ss(line);
        string cell;
        int col = 0;
        while (getline(ss, cell, ',') && col < 320) {
            this->motor_to_gimbal_tip_angle[row][col] = DEG2RAD * stod(cell);
            col++;
        }
        row++;
    }

    file.close();

    // Read the motor-to-gimbal tilt angle data file
    ifstream file2("/Users/leahkiner/Repositories/LASP_Basilisk/src/simulation/deviceInterface/twoAxisGimbal/motor_to_gimbal_tilt_angle.csv");
    if (!file2.is_open()) {
        cout << "Error opening tilt file!" << endl;
    }
    string line2;
    int row2 = 0;
    while (getline(file2, line2) && row2 < 320) {
        stringstream ss(line2);
        string cell2;
        int col2 = 0;
        while (getline(ss, cell2, ',') && col2 < 320) {
            this->motor_to_gimbal_tilt_angle[row2][col2] = DEG2RAD * stod(cell2);
            col2++;
        }
        row2++;
    }

    file2.close();

    this->tableStepAngle = DEG2RAD * 0.5;  // [rad]
}

/*! This method checks the input messages to ensure they are linked.
 @return void
 @param callTime [ns] Time the method is called
*/
void TwoAxisGimbal::Reset(uint64_t callTime) {
    if (!this->motor1InitStateInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "twoAxisGimbal.motor1InitStateInMsg wasn't connected.");
    }
    if (!this->motor2InitStateInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "twoAxisGimbal.motor2InitStateInMsg wasn't connected.");
    }
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

    // Initialize the module parameters to zero
    this->intermediateGimbalPRVThetaInit = 0.0;
    this->gimbalPRVTheta = 0.0;
    this->gimbalPRVThetaDot = 0.0;
    this->gimbalPRVThetaDDot = 0.0;
    this->tInit = 0.0;
    this->gimbalStepCount = 0;
    this->gimbalPRVRotHat = {0.0, 0.0, 1.0};

    // Set the previous written time to a negative value to capture a message written at time zero
    this->previousWrittenTime = -1;

    // Initialize the module boolean parameters
    this->completion = true;
    this->gimbalStepComplete = true;
    this->newMsg = false;
    this->segment1Complete = false;
    this->segment2Complete = false;
}

/*! This method determines the two-axis gimbal tip and tilt angles and profiles its prescribed hub-relative rotational
states using the two stepper motor input state messages. The gimbal tip and tilt angles and the hub-relative prescribed
rotational states are then written to the output messages.
 @return void
 @param callTime [ns] Time the method is called
*/
void TwoAxisGimbal::UpdateState(uint64_t callTime) {
    // Read the input messages
    HingedRigidBodyMsgPayload motor1InitStateIn = HingedRigidBodyMsgPayload();
    HingedRigidBodyMsgPayload motor2InitStateIn = HingedRigidBodyMsgPayload();
    MotorStepCommandMsgPayload motor1StepCmdIn = MotorStepCommandMsgPayload();
    MotorStepCommandMsgPayload motor2StepCmdIn = MotorStepCommandMsgPayload();
    StepperMotorMsgPayload motor1StateIn = StepperMotorMsgPayload();
    StepperMotorMsgPayload motor2StateIn = StepperMotorMsgPayload();

    if (this->motor1InitStateInMsg.isWritten() && this->motor2InitStateInMsg.isWritten()
                                               && this->motor1StepCmdInMsg.isWritten()
                                               && this->motor2StepCmdInMsg.isWritten()
                                               && this->motor1StateInMsg.isWritten()
                                               && this->motor2StateInMsg.isWritten()) {
        motor1InitStateIn = this->motor1InitStateInMsg();
        motor2InitStateIn = this->motor2InitStateInMsg();
        motor1StepCmdIn = this->motor1StepCmdInMsg();
        motor2StepCmdIn = this->motor2StepCmdInMsg();
        motor1StateIn = this->motor1StateInMsg();
        motor2StateIn = this->motor2StateInMsg();

        if (this->previousWrittenTime < this->motor1StepCmdInMsg.timeWritten()) {
            cout << "NEW MESSAGE!" << endl;
            this->previousWrittenTime = this->motor1StepCmdInMsg.timeWritten();

            // Store the initial motor angles
            this->motor1ThetaInit = motor1StateIn.theta;
            this->motor2ThetaInit = motor2StateIn.theta;

            // Find the initial gimbal attitude prv_F0M
            this->gimbalPRV_F0M = this->motorAnglesToGimbalPRV(this->motor1ThetaInit, this->motor2ThetaInit);

            // Store the motor steps commanded
            this->motor1StepsCommanded = motor1StepCmdIn.stepsCommanded;
            this->motor2StepsCommanded = motor2StepCmdIn.stepsCommanded;

            if (this->motor1StepsCommanded != 0 || this->motor2StepsCommanded != 0) {
                this->completion = false;
                this->segment1Complete = false;
                this->segment2Complete = false;
                if ((this->motor1StepsCommanded == 0 || this->motor2StepsCommanded == 0) || (this->motor1StepsCommanded == this->motor2StepsCommanded)) {
                    this->segment2Complete = true;
                }
                this->computeGimbalActuationParameters();
            } else {
                this->completion = true;
                this->gimbalPRVTheta = 0.0;
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

/*! This method determines the gimbal PRV hub-relative attitude given the stepper motor angles.
 @return Eigen::Vector3d
 @param motor1Angle [rad] Stepper motor 1 angle
 @param motor2Angle [rad] Stepper motor 2 angle
*/
Eigen::Vector3d TwoAxisGimbal::motorAnglesToGimbalPRV(double motor1Angle, double motor2Angle) {
    // Interpolate to find the reference gimbal attitude prv_FM
    std::pair<double, double> gimbalAngles = this->motorAnglesToGimbalAngles(motor1Angle, motor2Angle);
    double gimbalTipAngle = gimbalAngles.first;
    double gimbalTiltAngle = gimbalAngles.second;
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
    int compare1a = motor1Angle / this->tableStepAngle;
    double compare2a = motor1Angle / this->tableStepAngle;
    double compare3a = compare2a - compare1a;

    int compare1b = motor2Angle / this->tableStepAngle;
    double compare2b = motor2Angle / this->tableStepAngle;
    double compare3b = compare2b - compare1b;

    double gimbalTipAngle;
    double gimbalTiltAngle;

    if (compare3a < 1e-10 && compare3b < 1e-10) {  // Do not need to interpolate
        gimbalTipAngle = this->pullGimbalTipAngle(motor1Angle, motor2Angle);
        gimbalTiltAngle = this->pullGimbalTiltAngle(motor1Angle, motor2Angle);
    } else if (compare3a < 1e-10 || compare3b < 1e-10) {  // Linear interpolation required
        if (compare3a < 1e-10) {
            double lowerMotor2Angle = this->tableStepAngle * floor(motor2Angle / this->tableStepAngle);
            double upperMotor2Angle = this->tableStepAngle * ceil(motor2Angle / this->tableStepAngle);

            double z1_tip = this->pullGimbalTipAngle(motor1Angle, lowerMotor2Angle);
            double z2_tip = this->pullGimbalTipAngle(motor1Angle, upperMotor2Angle);
            gimbalTipAngle = this->bilinearInterpolation(motor1Angle,
                                                         motor1Angle,
                                                         lowerMotor2Angle,
                                                         upperMotor2Angle,
                                                         z1_tip,
                                                         z2_tip,
                                                         z1_tip,
                                                         z2_tip,
                                                         motor1Angle,
                                                         motor2Angle);

            double z1_tilt = this->pullGimbalTiltAngle(motor1Angle, lowerMotor2Angle);
            double z2_tilt = this->pullGimbalTiltAngle(motor1Angle, upperMotor2Angle);
            gimbalTiltAngle = this->bilinearInterpolation(motor1Angle,
                                                          motor1Angle,
                                                          lowerMotor2Angle,
                                                          upperMotor2Angle,
                                                          z1_tilt,
                                                          z2_tilt,
                                                          z1_tilt,
                                                          z2_tilt,
                                                          motor1Angle,
                                                          motor2Angle);
        } else {
            double lowerMotor1Angle = this->tableStepAngle * floor(motor1Angle / this->tableStepAngle);
            double upperMotor1Angle = this->tableStepAngle * ceil(motor1Angle / this->tableStepAngle);

            double z1_tip = this->pullGimbalTipAngle(lowerMotor1Angle, motor2Angle);
            double z2_tip = this->pullGimbalTipAngle(upperMotor1Angle, motor2Angle);
            gimbalTipAngle = this->bilinearInterpolation(lowerMotor1Angle,
                                                         upperMotor1Angle,
                                                         motor2Angle,
                                                         motor2Angle,
                                                         z1_tip,
                                                         z2_tip,
                                                         z1_tip,
                                                         z2_tip,
                                                         motor1Angle,
                                                         motor2Angle);

            double z1_tilt = this->pullGimbalTiltAngle(lowerMotor1Angle, motor2Angle);
            double z2_tilt = this->pullGimbalTiltAngle(upperMotor1Angle, motor2Angle);
            gimbalTiltAngle = this->bilinearInterpolation(lowerMotor1Angle,
                                                          upperMotor1Angle,
                                                          motor2Angle,
                                                          motor2Angle,
                                                          z1_tilt,
                                                          z2_tilt,
                                                          z1_tilt,
                                                          z2_tilt,
                                                          motor1Angle,
                                                          motor2Angle);
        }
    } else {  // Bilinear interpolation required
        double lowerMotor1Angle = this->tableStepAngle * floor(motor1Angle / this->tableStepAngle);
        double upperMotor1Angle = this->tableStepAngle * ceil(motor1Angle / this->tableStepAngle);
        double lowerMotor2Angle = this->tableStepAngle * floor(motor2Angle / this->tableStepAngle);
        double upperMotor2Angle = this->tableStepAngle * ceil(motor2Angle / this->tableStepAngle);

        double z11_tip = this->pullGimbalTipAngle(lowerMotor1Angle, lowerMotor2Angle);
        double z12_tip = this->pullGimbalTipAngle(lowerMotor1Angle, upperMotor2Angle);
        double z21_tip = this->pullGimbalTipAngle(upperMotor1Angle, lowerMotor2Angle);
        double z22_tip = this->pullGimbalTipAngle(upperMotor1Angle, upperMotor2Angle);

        double z11_tilt = this->pullGimbalTiltAngle(lowerMotor1Angle, lowerMotor2Angle);
        double z12_tilt = this->pullGimbalTiltAngle(lowerMotor1Angle, upperMotor2Angle);
        double z21_tilt = this->pullGimbalTiltAngle(upperMotor1Angle, lowerMotor2Angle);
        double z22_tilt = this->pullGimbalTiltAngle(upperMotor1Angle, upperMotor2Angle);

        gimbalTipAngle = this->bilinearInterpolation(lowerMotor1Angle,
                                                     upperMotor1Angle,
                                                     lowerMotor2Angle,
                                                     upperMotor2Angle,
                                                     z11_tip,
                                                     z12_tip,
                                                     z21_tip,
                                                     z22_tip,
                                                     motor1Angle,
                                                     motor2Angle);
        gimbalTiltAngle = this->bilinearInterpolation(lowerMotor1Angle,
                                                      upperMotor1Angle,
                                                      lowerMotor2Angle,
                                                      upperMotor2Angle,
                                                      z11_tilt,
                                                      z12_tilt,
                                                      z21_tilt,
                                                      z22_tilt,
                                                      motor1Angle,
                                                      motor2Angle);
    }

    std::pair<double, double> gimbalAngles = {gimbalTipAngle, gimbalTiltAngle};
    return gimbalAngles;
}

/*! Method used to compute and update the gimbal actuation parameters for each segment of required gimbal motion
 @return void
*/
void TwoAxisGimbal::computeGimbalActuationParameters() {
    Eigen::Vector3d gimbalPRV_FM = {0.0, 0.0, 0.0};
    Eigen::Vector3d gimbalPRV_FF0 = {0.0, 0.0, 0.0};
    Eigen::Vector3d gimbalPRV_FIntF0 = {0.0, 0.0, 0.0};
    Eigen::Vector3d gimbalPRV_FFInt = {0.0, 0.0, 0.0};

    if (!this->segment1Complete && this->segment2Complete) { // Single prv
        if (this->motor1StepsCommanded == this->motor2StepsCommanded) { // Actuate both motors
            cout << "Commanded motor 1 and motor 2 steps match: ACTUATE BOTH MOTORS" << endl;
            this->gimbalStepsCommanded = this->motor1StepsCommanded;
            this->motor1ThetaRef = this->motor1ThetaInit + this->gimbalStepsCommanded * this->motorStepAngle;
            this->motor2ThetaRef = this->motor2ThetaInit + this->gimbalStepsCommanded * this->motorStepAngle;

            // Interpolate to find the reference gimbal attitude prv_FM
            gimbalPRV_FM = motorAnglesToGimbalPRV(this->motor1ThetaRef, this->motor2ThetaRef);

            // Find the relative gimbal prv for the rotation prv_FF0
            gimbalPRV_FF0 = subPrv(gimbalPRV_FM, this->gimbalPRV_F0M);

            // Find the prv angle and axis for the rotation
            if (this->gimbalStepsCommanded < 0) {
                this->gimbalPRVThetaRef = - gimbalPRV_FF0.norm();
                this->gimbalPRVRotHat = -1 * gimbalPRV_FF0 / gimbalPRV_FF0.norm();
            } else {
                this->gimbalPRVThetaRef = gimbalPRV_FF0.norm();
                this->gimbalPRVRotHat = gimbalPRV_FF0 / this->gimbalPRVThetaRef;
            }

            this->gimbalStepAngle = fabs(this->gimbalPRVThetaRef / this->gimbalStepsCommanded);
            this->gimbalPRVThetaDDotMax = (4 * this->gimbalStepAngle) / (this->motorStepTime * this->motorStepTime);
        } else if (this->motor2StepsCommanded == 0) {  // Actuate motor 1
            cout << "Commanded motor 2 steps is zero: ACTUATE MOTOR 1 ONLY" << endl;
            this->gimbalStepsCommanded = this->motor1StepsCommanded;
            this->motor1ThetaRef = this->motor1ThetaInit + this->gimbalStepsCommanded * this->motorStepAngle;
            this->motor2ThetaRef = this->motor2ThetaInit;

            // Interpolate to find the reference gimbal attitude prv_FM
            gimbalPRV_FM = motorAnglesToGimbalPRV(this->motor1ThetaRef, this->motor2ThetaRef);

            // Find the relative gimbal prv for the rotation prv_FF0
            gimbalPRV_FF0 = subPrv(gimbalPRV_FM, this->gimbalPRV_F0M);

            // Find the prv angle and axis for the rotation
            if (this->gimbalStepsCommanded < 0) {
                this->gimbalPRVThetaRef = - gimbalPRV_FF0.norm();
                this->gimbalPRVRotHat = -1 * gimbalPRV_FF0 / gimbalPRV_FF0.norm();
            } else {
                this->gimbalPRVThetaRef = gimbalPRV_FF0.norm();
                this->gimbalPRVRotHat = gimbalPRV_FF0 / this->gimbalPRVThetaRef;
            }

            this->gimbalStepAngle = fabs(this->gimbalPRVThetaRef / this->gimbalStepsCommanded);
            this->gimbalPRVThetaDDotMax = (4 * this->gimbalStepAngle) / (this->motorStepTime * this->motorStepTime);
        } else if (this->motor1StepsCommanded == 0) {  // Actuate motor 2
            cout << "Commanded motor 1 steps is zero: ACTUATE MOTOR 2 ONLY" << endl;
            this->gimbalStepsCommanded = this->motor2StepsCommanded;
            this->motor1ThetaRef = this->motor1ThetaInit;
            this->motor2ThetaRef = this->motor2ThetaInit + this->gimbalStepsCommanded * this->motorStepAngle;

            // Interpolate to find the reference gimbal attitude prv_FM
            gimbalPRV_FM = motorAnglesToGimbalPRV(this->motor1ThetaRef, this->motor2ThetaRef);

            // Find the relative gimbal prv for the rotation prv_FF0
            gimbalPRV_FF0 = subPrv(gimbalPRV_FM, this->gimbalPRV_F0M);

            // Find the prv angle and axis for the rotation
            if (this->gimbalStepsCommanded < 0) {
                this->gimbalPRVThetaRef = - gimbalPRV_FF0.norm();
                this->gimbalPRVRotHat = -1 * gimbalPRV_FF0 / gimbalPRV_FF0.norm();
            } else {
                this->gimbalPRVThetaRef = gimbalPRV_FF0.norm();
                this->gimbalPRVRotHat = gimbalPRV_FF0 / this->gimbalPRVThetaRef;
            }

            this->gimbalStepAngle = fabs(this->gimbalPRVThetaRef / this->gimbalStepsCommanded);
            this->gimbalPRVThetaDDotMax = (4 * this->gimbalStepAngle) / (this->motorStepTime * this->motorStepTime);
        }
    } else if  (!this->segment1Complete && !this->segment2Complete) { // Set first of two prvs
        if (fabs(this->motor1StepsCommanded) > fabs(this->motor2StepsCommanded)) {
            cout << "Commanded motor 1 steps > Commanded motor 2 steps: ACTUATE BOTH MOTORS FOLLOWED BY ACTUATION OF SINGLE MOTOR" << endl;
            cout << "STEP 1: ACTUATE BOTH MOTORS" << endl;
            this->gimbalStepsCommanded = this->motor2StepsCommanded;
            this->motor1ThetaRef = this->motor1ThetaInit + this->gimbalStepsCommanded * this->motorStepAngle;
            this->motor2ThetaRef = this->motor2ThetaInit + this->gimbalStepsCommanded * this->motorStepAngle;

            // Interpolate to find the reference gimbal attitude prv_FM
            this->gimbalPRV_FIntM = motorAnglesToGimbalPRV(this->motor1ThetaRef, this->motor2ThetaRef);

            // Find the relative gimbal prv for the rotation prv_FF0
            gimbalPRV_FIntF0 = subPrv(this->gimbalPRV_FIntM, this->gimbalPRV_F0M);

            // Find the prv angle and axis for the rotation
            if (this->gimbalStepsCommanded < 0) {
                this->gimbalPRVThetaRef = - gimbalPRV_FIntF0.norm();
                this->gimbalPRVRotHat = -1 * gimbalPRV_FIntF0 / gimbalPRV_FIntF0.norm();
            } else {
                this->gimbalPRVThetaRef = gimbalPRV_FIntF0.norm();
                this->gimbalPRVRotHat = gimbalPRV_FIntF0 / this->gimbalPRVThetaRef;
            }

            this->gimbalStepAngle = fabs(this->gimbalPRVThetaRef / this->gimbalStepsCommanded);
            this->gimbalPRVThetaDDotMax = (4 * this->gimbalStepAngle) / (this->motorStepTime * this->motorStepTime);
        } else {
            cout << "Commanded motor 2 steps > Commanded motor 1 steps: ACTUATE BOTH MOTORS FOLLOWED BY ACTUATION OF SINGLE MOTOR" << endl;
            cout << "STEP 1: ACTUATE BOTH MOTORS" << endl;
            this->gimbalStepsCommanded = this->motor1StepsCommanded;
            this->motor1ThetaRef = this->motor1ThetaInit + this->gimbalStepsCommanded * this->motorStepAngle;
            this->motor2ThetaRef = this->motor2ThetaInit + this->gimbalStepsCommanded * this->motorStepAngle;

            // Interpolate to find the reference gimbal attitude prv_FM
            this->gimbalPRV_FIntM = motorAnglesToGimbalPRV(this->motor1ThetaRef, this->motor2ThetaRef);

            // Find the relative gimbal prv for the rotation prv_FF0
            gimbalPRV_FIntF0 = subPrv(this->gimbalPRV_FIntM, this->gimbalPRV_F0M);

            // Find the prv angle and axis for the rotation
            if (this->gimbalStepsCommanded < 0) {
                this->gimbalPRVThetaRef = - gimbalPRV_FIntF0.norm();
                this->gimbalPRVRotHat = -1 * gimbalPRV_FIntF0 / gimbalPRV_FIntF0.norm();
            } else {
                this->gimbalPRVThetaRef = gimbalPRV_FIntF0.norm();
                this->gimbalPRVRotHat = gimbalPRV_FIntF0 / this->gimbalPRVThetaRef;
            }

            this->gimbalStepAngle = fabs(this->gimbalPRVThetaRef / this->gimbalStepsCommanded);
            this->gimbalPRVThetaDDotMax = (4 * this->gimbalStepAngle) / (this->motorStepTime * this->motorStepTime);
        }
    } else if  (this->segment1Complete && !this->segment2Complete) {  // Set second of two prvs
        if (fabs(this->motor1StepsCommanded) > fabs(this->motor2StepsCommanded)) {
            cout << "Commanded motor 1 steps > Commanded motor 2 steps: ACTUATE BOTH MOTORS FOLLOWED BY ACTUATION OF SINGLE MOTOR" << endl;
            cout << "STEP 2: ACTUATE MOTOR 1" << endl;
            this->gimbalStepsCommanded = (this->motor1StepsCommanded - this->motor2StepsCommanded);
            this->motor1ThetaRef = this->motor1ThetaRef + this->gimbalStepsCommanded * this->motorStepAngle;

            // Interpolate to find the reference gimbal attitude prv_FM
            gimbalPRV_FM = motorAnglesToGimbalPRV(this->motor1ThetaRef, this->motor2ThetaRef);

            // Find the relative gimbal prv for the rotation prv_FF0
            gimbalPRV_FFInt = subPrv(gimbalPRV_FM, this->gimbalPRV_FIntM);

            // Find the prv angle and axis for the rotation
            if (this->gimbalStepsCommanded < 0) {
                this->gimbalPRVThetaRef = - gimbalPRV_FFInt.norm();
                this->gimbalPRVRotHat = -1 * gimbalPRV_FFInt / gimbalPRV_FFInt.norm();
            } else {
                this->gimbalPRVThetaRef = gimbalPRV_FFInt.norm();
                this->gimbalPRVRotHat = gimbalPRV_FFInt / this->gimbalPRVThetaRef;
            }

            this->gimbalStepAngle = fabs(this->gimbalPRVThetaRef / this->gimbalStepsCommanded);
            this->gimbalPRVThetaDDotMax = (4 * this->gimbalStepAngle) / (this->motorStepTime * this->motorStepTime);

            this->gimbalPRV_F0M = this->gimbalPRV_FIntM;
        } else {
            cout << "Commanded motor 2 steps > Commanded motor 1 steps: ACTUATE BOTH MOTORS FOLLOWED BY ACTUATION OF SINGLE MOTOR" << endl;
            cout << "STEP 2: ACTUATE MOTOR 2" << endl;
            this->gimbalStepsCommanded = (this->motor2StepsCommanded - this->motor1StepsCommanded);
            this->motor2ThetaRef = this->motor2ThetaRef + this->gimbalStepsCommanded * this->motorStepAngle;

            // Interpolate to find the reference gimbal attitude prv_FM
            gimbalPRV_FM = motorAnglesToGimbalPRV(this->motor1ThetaRef, this->motor2ThetaRef);

            // Find the relative gimbal prv for the rotation prv_FF0
            gimbalPRV_FFInt = subPrv(gimbalPRV_FM, this->gimbalPRV_FIntM);

            // Find the prv angle and axis for the rotation
            if (this->gimbalStepsCommanded < 0) {
                this->gimbalPRVThetaRef = - gimbalPRV_FFInt.norm();
                this->gimbalPRVRotHat = -1 * gimbalPRV_FFInt / gimbalPRV_FFInt.norm();
            } else {
                this->gimbalPRVThetaRef = gimbalPRV_FFInt.norm();
                this->gimbalPRVRotHat = gimbalPRV_FFInt / this->gimbalPRVThetaRef;
            }

            this->gimbalStepAngle = fabs(this->gimbalPRVThetaRef / this->gimbalStepsCommanded);
            this->gimbalPRVThetaDDotMax = (4 * this->gimbalStepAngle) / (this->motorStepTime * this->motorStepTime);

            this->gimbalPRV_F0M = this->gimbalPRV_FIntM;
        }
    }
    cout << "computeGimbalActuationParameters - - - gimbalStepsCommanded " << this->gimbalStepsCommanded << endl;
    cout << "computeGimbalActuationParameters - - - gimbalPRVThetaRef " << (180.0 / M_PI) * this->gimbalPRVThetaRef << endl;
    cout << "computeGimbalActuationParameters - - - gimbalStepAngle " << (180.0 / M_PI) * this->gimbalStepAngle << endl;
    cout << "computeGimbalActuationParameters - - - gimbalPRVThetaDDotMax " << this->gimbalPRVThetaDDotMax << endl;
}

/*! This function uses bilinear interpolation to solve for the value of an unknown function of two variables f(x,y)
 at the point (x,y).
@return double
@param x1 Data point x1
@param x2 Data point x2
@param y1 Data point y1
@param y2 Data point y2
@param z11 Function value at point (x1, y1)
@param z12 Function value at point (x1, y2)
@param z21 Function value at point (x2, y1)
@param z22 Function value at point (x2, y2)
@param x Function x coordinate for interpolation
@param y Function y coordinate for interpolation
*/
double TwoAxisGimbal::bilinearInterpolation(double x1,
                                            double x2,
                                            double y1,
                                            double y2,
                                            double z11,
                                            double z12,
                                            double z21,
                                            double z22,
                                            double x,
                                            double y) {
    if (x1 != x2 && y1 != y2) { // bilinear interpolation
        return 1 / ((x2 - x1) * (y2 - y1)) * (z11 * (x2 - x) * (y2 - y) + z21 * (x - x1) * (y2 - y)
                                              + z12 * (x2 - x) * (y - y1)
                                              + z22 * (x - x1) * (y - y1));
    } else if (x1 != x2) { // linear interpolation along x
        return z11 * (x2 - x) / (x2 - x1) + z12 * (x - x1) / (x2 - x1);
    } else { // linear interpolation along y
        return z21 * (y2 - y) / (y2 - y1) + z22 * (y - y1) / (y2 - y1);
    }
}

/*! This method pulls a specific gimbal tip angle from the tip interpolation table given specific motor angles.
 @return double
 @param motor1Angle [rad] Stepper motor 1 angle
 @param motor2Angle [rad] Stepper motor 2 angle
*/
double TwoAxisGimbal::pullGimbalTipAngle(double motor1Angle, double motor2Angle) {
    int motor1Idx = motor1Angle / (DEG2RAD * 0.5);
    int motor2Idx = motor2Angle / (DEG2RAD * 0.5);
    return this->motor_to_gimbal_tip_angle[motor2Idx][motor1Idx];
}

/*! This method pulls a specific gimbal tilt angle from the tilt interpolation table given specific motor angles.
 @return double
 @param motor1Angle [rad] Stepper motor 1 angle
 @param motor2Angle [rad] Stepper motor 2 angle
*/
double TwoAxisGimbal::pullGimbalTiltAngle(double motor1Angle, double motor2Angle) {
    int motor1Idx = motor1Angle / (DEG2RAD * 0.5);
    int motor2Idx = motor2Angle / (DEG2RAD * 0.5);
    return this->motor_to_gimbal_tilt_angle[motor2Idx][motor1Idx];
}

/*! This high-level method is used to simulate the gimbal prv states in time.
 @return void
 @param t [s] Time the method is called
*/
void TwoAxisGimbal::actuateGimbal(double t) {
    // Reset the gimbal states when the current request is complete and a new request is received
    if (this->newMsg && this->gimbalStepComplete) {
        this->resetGimbal(t);
    }

    // Define temporal information for the rotation
    this->tf = this->tInit + this->motorStepTime;
    this->ts = this->tInit + this->motorStepTime / 2;

    // Update the intermediate initial and reference gimbal angles and the parabolic constants when a step is completed
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

    // Re-compute the actuation parameters if the first segment is complete and a second actuation segment is required
    if (this->segment1Complete && !this->segment2Complete) {
        this->computeGimbalActuationParameters();
    }

    this->newMsg = false;
}

/*! This method updates the gimbal rotation parameters after a gimbal step is completed.
 @return void
*/
void TwoAxisGimbal::updateGimbalRotationParameters() {
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

/*! Setter method for the gimbal rotation axis 1.
 @return void
 @param rotHat1_M Gimbal tip angle axis of rotation 1 expressed in M frame components (unit vector)
*/
void TwoAxisGimbal::setGimbalRotHat1_M(const Eigen::Vector3d &rotHat1_M) {
    this->gimbalRotHat1_M = rotHat1_M / rotHat1_M.norm();
}

/*! Setter method for the gimbal rotation axis 2.
 @return void
 @param rotHat2_F Gimbal tilt angle axis of rotation 2 expressed in F frame components (unit vector)
*/
void TwoAxisGimbal::setGimbalRotHat2_F(const Eigen::Vector3d &rotHat2_F) {
    this->gimbalRotHat2_F = rotHat2_F / rotHat2_F.norm();
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
