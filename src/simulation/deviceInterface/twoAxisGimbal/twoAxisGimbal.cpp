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
#include <math.h>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
using namespace std;

TwoAxisGimbal::TwoAxisGimbal() {
    // Read the motor-to-gimbal tip angle data file
    ifstream file("motor_to_gimbal_tip_angle.csv");
    string line;
    int row = 0;
    while (getline(file, line) && row < 321) {
        stringstream ss(line);
        string cell;
        int col = 0;
        while (getline(ss, cell, ',') && col < 321) {
            cout << "\nCell: " << cell << endl;
            this->motor_to_gimbal_tip_angle[row][col] = stod(cell);
            cout << "Table element: " << endl;
            cout << this->motor_to_gimbal_tip_angle[row][col] << endl;
            col++;
        }
    }
    row++;
    file.close();

    // Read the motor-to-gimbal tilt angle data file
    ifstream file2("motor_to_gimbal_tilt_angle.csv");
    string line2;
    int row2 = 0;
    while (getline(file2, line2) && row2 < 321) {
        stringstream ss(line2);
        string cell2;
        int col2 = 0;
        while (getline(ss, cell2, ',') && col2 < 321) {
            this->motor_to_gimbal_tilt_angle[row2][col2] = stod(cell2);
            col2++;
        }
    }
    row2++;
    file2.close();
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
    this->gimbalPRVTheta = 0.0;
    this->gimbalPRVThetaDot = 0.0;
    this->gimbalPRVThetaDDot = 0.0;
    this->tInit = 0.0;
    this->gimbalStepCount = 0;

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

        if (this->previousWrittenTime <  this->motor1InitStateInMsg.timeWritten()) {
            this->previousWrittenTime = this->motor1InitStateInMsg.timeWritten();

            // Store the initial motor angles
            this->motor1ThetaInit = motor1InitStateIn.theta;
            this->motor2ThetaInit = motor2InitStateIn.theta;

            // Store the motor steps commanded
            this->motor1StepsCommanded = motor1StepCmdIn.stepsCommanded;
            this->motor2StepsCommanded = motor2StepCmdIn.stepsCommanded;

            if (this->motor1StepsCommanded != 0 || this->motor2StepsCommanded != 0) {
                this->completion = false;
                if ((this->motor1StepsCommanded == 0 || this->motor2StepsCommanded == 0)
                    || (this->motor1StepsCommanded == this->motor2StepsCommanded)) {
                    this->segment2Complete = true;
                }
                // Interpolate to find the initial gimbal attitude prv_F0M
                this->gimbalPRV_F0M = {0.0, 0.0, 0.0};  // INTERPOLATE
                this->computeGimbalActuationParameters();
            } else {
                this->completion = true;
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

void TwoAxisGimbal::computeGimbalActuationParameters() {
    Eigen::Vector3d gimbalPRV_F0M = {0.0, 0.0, 0.0};
    Eigen::Vector3d gimbalPRV_FM = {0.0, 0.0, 0.0};
    Eigen::Vector3d gimbalPRV_FF0 = {0.0, 0.0, 0.0};
    Eigen::Vector3d gimbalPRV_FIntM = {0.0, 0.0, 0.0};
    Eigen::Vector3d gimbalPRV_FIntF0 = {0.0, 0.0, 0.0};
    Eigen::Vector3d gimbalPRV_FFInt = {0.0, 0.0, 0.0};

    if (!this->segment1Complete && this->segment2Complete) { // Single prv
        if (this->motor1StepsCommanded == this->motor2StepsCommanded) { // Actuate both motors
            this->gimbalStepsCommanded = this->motor1StepsCommanded;
            this->motor1ThetaRef = this->motor1ThetaInit + this->gimbalStepsCommanded * this->motorStepAngle;
            this->motor2ThetaRef = this->motor2ThetaInit + this->gimbalStepsCommanded * this->motorStepAngle;
            // Interpolate to find the reference gimbal attitude prv_FM
            this->interpolateMotorAngles();
            Eigen::Vector3d prvTip = this->gimbalTheta1Ref * this->gimbalRotHat1_M;
            Eigen::Vector3d prvTilt = this->gimbalTheta2Ref * this->gimbalRotHat2_F;
            gimbalPRV_FM = addPrv(prvTilt, prvTip);
            // Find the relative gimbal prv for the rotation prv_FF0
            gimbalPRV_FF0 = subPrv(gimbalPRV_FM, this->gimbalPRV_F0M);
            // Find the angle the gimbal must rotate through for the rotation
            this->gimbalPRVThetaRef = gimbalPRV_FF0.norm();
            this->gimbalStepAngle = fabs(this->gimbalPRVThetaRef) / this->gimbalStepsCommanded;
            this->gimbalPRVThetaDDotMax = (4 * this->gimbalStepAngle) / (this->motorStepTime * this->motorStepTime);
        } else if (this->motor2StepsCommanded == 0) {  // Actuate motor 1
            this->gimbalStepsCommanded = this->motor1StepsCommanded;
            this->motor1ThetaRef = this->motor1ThetaInit + this->gimbalStepsCommanded * this->motorStepAngle;
            this->motor2ThetaRef = this->motor2ThetaInit;
            // Interpolate to find the reference gimbal attitude prv_FM
            this->interpolateMotorAngles();
            Eigen::Vector3d prvTip = this->gimbalTheta1Ref * this->gimbalRotHat1_M;
            Eigen::Vector3d prvTilt = this->gimbalTheta2Ref * this->gimbalRotHat2_F;
            gimbalPRV_FM = addPrv(prvTilt, prvTip);
            // Find the relative gimbal prv for the rotation prv_FF0
            gimbalPRV_FF0 = subPrv(gimbalPRV_FM, this->gimbalPRV_F0M);
            // Find the angle the gimbal must rotate through for the rotation
            this->gimbalPRVThetaRef = gimbalPRV_FF0.norm();
            this->gimbalStepAngle = fabs(this->gimbalPRVThetaRef) / this->gimbalStepsCommanded;
            this->gimbalPRVThetaDDotMax = (4 * this->gimbalStepAngle) / (this->motorStepTime * this->motorStepTime);
        } else if (this->motor1StepsCommanded == 0) {  // Actuate motor 2
            this->gimbalStepsCommanded = this->motor2StepsCommanded;
            this->motor1ThetaRef = this->motor1ThetaInit;
            this->motor2ThetaRef = this->motor2ThetaInit + this->gimbalStepsCommanded * this->motorStepAngle;
            // Interpolate to find the reference gimbal attitude prv_FM
            this->interpolateMotorAngles();
            Eigen::Vector3d prvTip = this->gimbalTheta1Ref * this->gimbalRotHat1_M;
            Eigen::Vector3d prvTilt = this->gimbalTheta2Ref * this->gimbalRotHat2_F;
            gimbalPRV_FM = addPrv(prvTilt, prvTip);
            // Find the relative gimbal prv for the rotation prv_FF0
            gimbalPRV_FF0 = subPrv(gimbalPRV_FM, this->gimbalPRV_F0M);
            // Find the angle the gimbal must rotate through for the rotation
            this->gimbalPRVThetaRef = gimbalPRV_FF0.norm();
            this->gimbalStepAngle = fabs(this->gimbalPRVThetaRef) / this->gimbalStepsCommanded;
            this->gimbalPRVThetaDDotMax = (4 * this->gimbalStepAngle) / (this->motorStepTime * this->motorStepTime);
        }
    } else if  (!this->segment1Complete && !this->segment2Complete) { // Set first of two prvs
        if (this->motor1StepsCommanded > this->motor2StepsCommanded) {
            this->gimbalStepsCommanded = this->motor2StepsCommanded;
            this->motor1ThetaRef = this->motor1ThetaInit + this->gimbalStepsCommanded * this->motorStepAngle;
            this->motor2ThetaRef = this->motor2ThetaInit + this->gimbalStepsCommanded * this->motorStepAngle;
            // Interpolate to find the reference gimbal attitude prv_FM
            this->interpolateMotorAngles();
            Eigen::Vector3d prvTip = this->gimbalTheta1Ref * this->gimbalRotHat1_M;
            Eigen::Vector3d prvTilt = this->gimbalTheta2Ref * this->gimbalRotHat2_F;
            gimbalPRV_FIntM = addPrv(prvTilt, prvTip);
            // Find the relative gimbal prv for the rotation prv_FF0
            gimbalPRV_FIntF0 = subPrv(gimbalPRV_FIntM, this->gimbalPRV_F0M);
            // Find the angle the gimbal must rotate through for the rotation
            this->gimbalPRVThetaRef = gimbalPRV_FF0.norm();
            this->gimbalStepAngle = fabs(this->gimbalPRVThetaRef) / this->gimbalStepsCommanded;
            this->gimbalPRVThetaDDotMax = (4 * this->gimbalStepAngle) / (this->motorStepTime * this->motorStepTime);
        } else {
            this->gimbalStepsCommanded = this->motor1StepsCommanded;
            this->motor1ThetaRef = this->motor1ThetaInit + this->gimbalStepsCommanded * this->motorStepAngle;
            this->motor2ThetaRef = this->motor2ThetaInit + this->gimbalStepsCommanded * this->motorStepAngle;
            // Interpolate to find the reference gimbal attitude prv_FM
            this->interpolateMotorAngles();
            Eigen::Vector3d prvTip = this->gimbalTheta1Ref * this->gimbalRotHat1_M;
            Eigen::Vector3d prvTilt = this->gimbalTheta2Ref * this->gimbalRotHat2_F;
            gimbalPRV_FIntM = addPrv(prvTilt, prvTip);
            // Find the relative gimbal prv for the rotation prv_FF0
            gimbalPRV_FIntF0 = subPrv(gimbalPRV_FIntM, this->gimbalPRV_F0M);
            // Find the angle the gimbal must rotate through for the rotation
            this->gimbalPRVThetaRef = gimbalPRV_FF0.norm();
            this->gimbalStepAngle = fabs(this->gimbalPRVThetaRef) / this->gimbalStepsCommanded;
            this->gimbalPRVThetaDDotMax = (4 * this->gimbalStepAngle) / (this->motorStepTime * this->motorStepTime);
        }
    } else if  (this->segment1Complete && !this->segment2Complete) {  // Set second of two prvs
        if (this->motor1StepsCommanded > this->motor2StepsCommanded) {
            this->gimbalStepsCommanded = (this->motor1StepsCommanded - this->motor2StepsCommanded);
            this->motor1ThetaRef = this->motor1ThetaRef + this->gimbalStepsCommanded * this->motorStepAngle;
            // Interpolate to find the reference gimbal attitude prv_FM
            this->interpolateMotorAngles();
            Eigen::Vector3d prvTip = this->gimbalTheta1Ref * this->gimbalRotHat1_M;
            Eigen::Vector3d prvTilt = this->gimbalTheta2Ref * this->gimbalRotHat2_F;
            gimbalPRV_FM = addPrv(prvTilt, prvTip);
            // Find the relative gimbal prv for the rotation prv_FF0
            gimbalPRV_FFInt = subPrv(gimbalPRV_FM, gimbalPRV_FIntM);
            // Find the angle the gimbal must rotate through for the rotation
            this->gimbalPRVThetaRef = gimbalPRV_FFInt.norm();
            this->gimbalStepAngle = fabs(this->gimbalPRVThetaRef) / this->gimbalStepsCommanded;
            this->gimbalPRVThetaDDotMax = (4 * this->gimbalStepAngle) / (this->motorStepTime * this->motorStepTime);
        } else {
            this->gimbalStepsCommanded = (this->motor2StepsCommanded - this->motor1StepsCommanded);
            this->motor2ThetaRef = this->motor2ThetaRef + this->gimbalStepsCommanded * this->motorStepAngle;
            // Interpolate to find the reference gimbal attitude prv_FM
            this->interpolateMotorAngles();
            Eigen::Vector3d prvTip = this->gimbalTheta1Ref * this->gimbalRotHat1_M;
            Eigen::Vector3d prvTilt = this->gimbalTheta2Ref * this->gimbalRotHat2_F;
            gimbalPRV_FM = addPrv(prvTilt, prvTip);
            // Find the relative gimbal prv for the rotation prv_FF0
            gimbalPRV_FFInt = subPrv(gimbalPRV_FM, gimbalPRV_FIntM);
            // Find the angle the gimbal must rotate through for the rotation
            this->gimbalPRVThetaRef = gimbalPRV_FFInt.norm();
            this->gimbalStepAngle = fabs(this->gimbalPRVThetaRef) / this->gimbalStepsCommanded;
            this->gimbalPRVThetaDDotMax = (4 * this->gimbalStepAngle) / (this->motorStepTime * this->motorStepTime);
        }
    }
}

void TwoAxisGimbal::interpolateMotorAngles() {
    double tableStepAngle = 0.5;  // [deg]

    int compare1a = this->motor1ThetaRef / tableStepAngle;
    double compare2a = this->motor1ThetaRef / tableStepAngle;
    double compare3a = compare2a - compare1a;

    int compare1b = this->motor2ThetaRef / tableStepAngle;
    double compare2b = this->motor2ThetaRef / tableStepAngle;
    double compare3b = compare2b - compare1b;

    if (compare3a == 0.0 && compare3b == 0.0) {  // Do not need to interpolate
        this->gimbalTheta1Ref = this->pullGimbalTipAngle(this->motor1ThetaRef, this->motor2ThetaRef);
        this->gimbalTheta2Ref = this->pullGimbalTiltAngle(this->motor1ThetaRef, this->motor2ThetaRef);
    } else if (compare3a == 0.0 || compare3b == 0.0) {  // Linear interpolation required
        if (compare3a == 0.0) {
            double lowerMotor2Angle = tableStepAngle * floor(this->motor2ThetaRef / tableStepAngle);
            double upperMotor2Angle = tableStepAngle * ceil(this->motor2ThetaRef / tableStepAngle);

            double z1_tip = this->pullGimbalTipAngle(this->motor1ThetaRef, lowerMotor2Angle);
            double z2_tip = this->pullGimbalTiltAngle(this->motor1ThetaRef, upperMotor2Angle);
            this->gimbalTheta1Ref = this->linearInterpolation(lowerMotor2Angle, upperMotor2Angle, z1_tip, z2_tip, this->motor1ThetaRef);

            double z1_tilt = this->pullGimbalTiltAngle(this->motor1ThetaRef, lowerMotor2Angle);
            double z2_tilt = this->pullGimbalTiltAngle(this->motor1ThetaRef, upperMotor2Angle);
            this->gimbalTheta2Ref = this->linearInterpolation(lowerMotor2Angle, upperMotor2Angle, z1_tilt, z2_tilt, this->motor2ThetaRef);
        } else {
            double lowerMotor1Angle = tableStepAngle * floor(this->motor1ThetaRef / tableStepAngle);
            double upperMotor1Angle = tableStepAngle * ceil(this->motor1ThetaRef / tableStepAngle);

            double z1_tip = this->pullGimbalTipAngle(lowerMotor1Angle, this->motor2ThetaRef);
            double z2_tip = this->pullGimbalTipAngle(upperMotor1Angle, this->motor2ThetaRef);
            this->gimbalTheta1Ref = this->linearInterpolation(lowerMotor1Angle, upperMotor1Angle, z1_tip, z2_tip, this->motor1ThetaRef);

            double z1_tilt = this->pullGimbalTiltAngle(lowerMotor1Angle, this->motor2ThetaRef);
            double z2_tilt = this->pullGimbalTiltAngle(upperMotor1Angle, this->motor2ThetaRef);
            this->gimbalTheta2Ref = this->linearInterpolation(lowerMotor1Angle, upperMotor1Angle, z1_tilt, z2_tilt, this->motor1ThetaRef);
        }
    } else {  // Bilinear interpolation required
        double lowerMotor1Angle = tableStepAngle * floor(this->motor1ThetaRef / tableStepAngle);
        double upperMotor1Angle = tableStepAngle * ceil(this->motor1ThetaRef / tableStepAngle);
        double lowerMotor2Angle = tableStepAngle * floor(this->motor2ThetaRef / tableStepAngle);
        double upperMotor2Angle = tableStepAngle * ceil(this->motor2ThetaRef / tableStepAngle);

        double z11_tip = this->pullGimbalTipAngle(lowerMotor1Angle, lowerMotor2Angle);
        double z12_tip = this->pullGimbalTipAngle(lowerMotor1Angle, upperMotor2Angle);
        double z21_tip = this->pullGimbalTipAngle(upperMotor1Angle, lowerMotor2Angle);
        double z22_tip = this->pullGimbalTipAngle(upperMotor1Angle, upperMotor2Angle);

        double z11_tilt = this->pullGimbalTiltAngle(lowerMotor1Angle, lowerMotor2Angle);
        double z12_tilt = this->pullGimbalTiltAngle(lowerMotor1Angle, upperMotor2Angle);
        double z21_tilt = this->pullGimbalTiltAngle(upperMotor1Angle, lowerMotor2Angle);
        double z22_tilt = this->pullGimbalTiltAngle(upperMotor1Angle, upperMotor2Angle);

        this->gimbalTheta1Ref = this->bilinearInterpolation(lowerMotor1Angle,
                                                            upperMotor1Angle,
                                                            lowerMotor2Angle,
                                                            upperMotor2Angle,
                                                            z11_tip,
                                                            z12_tip,
                                                            z21_tip,
                                                            z22_tip,
                                                            this->motor1ThetaRef,
                                                            this->motor2ThetaRef);
        this->gimbalTheta2Ref = this->bilinearInterpolation(lowerMotor1Angle,
                                                            upperMotor1Angle,
                                                            lowerMotor2Angle,
                                                            upperMotor2Angle,
                                                            z11_tilt,
                                                            z12_tilt,
                                                            z21_tilt,
                                                            z22_tilt,
                                                            this->motor1ThetaRef,
                                                            this->motor2ThetaRef);
    }
}

double TwoAxisGimbal::pullGimbalTipAngle(double motor1Angle, double motor2Angle) {
    int motor1Idx = motor1Angle / 0.5 + 1;
    int motor2Idx = motor2Angle / 0.5 + 1;
    return this->motor_to_gimbal_tip_angle[motor2Idx][motor1Idx];
}

double TwoAxisGimbal::pullGimbalTiltAngle(double motor1Angle, double motor2Angle) {
    int motor1Idx = motor1Angle / 0.5 + 1;
    int motor2Idx = motor2Angle / 0.5 + 1;
    return this->motor_to_gimbal_tilt_angle[motor2Idx][motor1Idx];
}

double TwoAxisGimbal::linearInterpolation(double x1, double x2, double z1, double z2, double x) {
    return z1 * (x2 - x) / (x2 - x1) + z2 * (x - x1) / (x2 - x1);
}

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
    return 1 / ((x2 - x1) * (y2 - y1)) * (z11 * (x2 - x) * (y2 - y) + z21 * (x - x1) * (y2 - y)
                                                                    + z12 * (x2 - x) * (y - y1)
                                                                    + z22 * (x - x1) * (y - y1));
}

/*! This high-level method is used to simulate the gimbal prv states in time.
 @return void
 @param t [s] Time the method is called
*/
void TwoAxisGimbal::actuateGimbal(double t) {
    // Reset the gimbal states when the current request is complete and a new request is received
    if ((this->newMsg && this->gimbalStepComplete) || this->segment1Complete) {
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

    // Compute the actuation parameters if a second actuation segment is required
    if (!this->segment2Complete) {
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
    } else if ((this->gimbalStepCount == this->gimbalStepsCommanded) && !this->newMsg && this->segment1Complete) {
        this->segment2Complete = true;
        this->completion = true;
    }
}

/*! This method writes the module output messages and computes the output message data.
 @return void
*/
void TwoAxisGimbal::writeOutputMessages(uint64_t callTime) {
    // Create the output buffer messages
    TwoAxisGimbalMsgPayload twoAxisGimbalOut;
    PrescribedRotationMsgPayload prescribedRotationOut;

    // Zero the output messages
    twoAxisGimbalOut = TwoAxisGimbalMsgPayload();
    prescribedRotationOut = PrescribedRotationMsgPayload();

    // Compute the angular velocity of frame F wrt frame M in F frame components
    Eigen::Vector3d omega_FM_F = this->gimbalPRVThetaDot * this->gimbalPRVRotHat;  // [rad/s]

    // Compute the B frame time derivative of omega_FM_F in F frame components
    Eigen::Vector3d omegaPrime_FM_F = this->gimbalPRVThetaDDot * this->gimbalPRVRotHat;  // [rad/s^2]

    // Determine the DCM dcm_FM representing the current gimbal attitude relative to the mount frame
    Eigen::Vector3d prv_FM = this->gimbalPRVTheta * this->gimbalPRVRotHat;
    Eigen::Matrix3d dcm_FM = prvToDcm(prv_FM);

    // Compute the MRP sigma_FM representing the current gimbal attitude relative to the mount frame
    Eigen::Vector3d sigma_FM = dcmToMrp(dcm_FM);

    // Determine the gimbal tip and tilt angles
    this->gimbalTheta1 = atan(dcm_FM(1,2) / dcm_FM(1,0));
    this->gimbalTheta2 = atan(dcm_FM(2,0) / dcm_FM(0,0));

    // Copy the module variables to the output buffer messages
    twoAxisGimbalOut.theta1 = this->gimbalTheta1;
    twoAxisGimbalOut.theta2 = this->gimbalTheta2;
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
