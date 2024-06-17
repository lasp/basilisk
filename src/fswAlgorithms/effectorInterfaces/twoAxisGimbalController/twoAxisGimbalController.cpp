/*
 ISC License

 Copyright (c) 2024, Laboratory For Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "twoAxisGimbalController.h"
#include "architecture/utilities/linearInterpolation.hpp"
#include "architecture/utilities/bilinearInterpolation.hpp"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

/*! Module constructor. The path to the interpolation table data files must be specified.
 @return void
 @param pathToMotor1Table String for the gimbal-to-motor 1 angle file absolute path
 @param pathToMotor2Table String for the gimbal-to-motor 1 angle file absolute path
*/
TwoAxisGimbalController::TwoAxisGimbalController(std::string pathToMotor1Table, std::string pathToMotor2Table) {
    // Read the gimbal-to-motor angle lookup tables
    this->loadGimbalToMotor1AngleLookupTable(std::move(pathToMotor1Table));
    this->loadGimbalToMotor2AngleLookupTable(std::move(pathToMotor2Table));
}

/*! This method checks the input messages to ensure they are linked. This method also resets module parameters to their
default values.
 @return void
 @param callTime [ns] Time the method is called
*/
void TwoAxisGimbalController::Reset(uint64_t callTime) {
    if (!this->twoAxisGimbalInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "twoAxisGimbalController.twoAxisGimbalInMsg wasn't connected.");
    }

    this->motor1Angle = 0.0;
    this->motor2Angle = 0.0;
}

/*! This method determines the stepper motor angles corresponding to the given gimbal tip and tilt angles. The motor
angles are then written to the module output message.
 @return void
 @param callTime [ns] The current time of simulation
*/
void TwoAxisGimbalController::UpdateState(uint64_t callTime) {
    // Read the input messages
    auto twoAxisGimbalIn = TwoAxisGimbalMsgPayload();
    if (this->twoAxisGimbalInMsg.isWritten()) {
        twoAxisGimbalIn = this->twoAxisGimbalInMsg();
        this->gimbalTipAngleRef = twoAxisGimbalIn.theta1;
        this->gimbalTiltAngleRef = twoAxisGimbalIn.theta2;
    }

    // Use the provided gimbal angles to determine the motor angles
    this->gimbalAnglesToMotorAngles();

    // Write the module output messages
    auto motor1AngleOut = HingedRigidBodyMsgPayload();
    auto motor2AngleOut = HingedRigidBodyMsgPayload();
    motor1AngleOut.theta = this->motor1Angle;
    motor2AngleOut.theta = this->motor2Angle;
    this->motor1AngleOutMsg.write(&motor1AngleOut, moduleID, callTime);
    this->motor2AngleOutMsg.write(&motor2AngleOut, moduleID, callTime);
}

/*! This method determines the stepper motor angles given the gimbal sequential tip and tilt angles.
 @return void
*/
void TwoAxisGimbalController::gimbalAnglesToMotorAngles() {
    if (this->bilinearInterpolationRequired()) {
        this->bilinearlyInterpolateMotorAngles();
    } else if (this->noInterpolationRequired()) {
        this->motor1Angle = this->pullMotor1Angle(this->gimbalTipAngleRef,
                                                  this->gimbalTiltAngleRef);
        this->motor2Angle = this->pullMotor2Angle(this->gimbalTipAngleRef,
                                                  this->gimbalTiltAngleRef);

        // Check that a valid gimbal attitude was requested
        assert(!(this->motor1Angle < 0.0 && this->motor2Angle < 0.0));

    } else if (this->linearInterpolationRequired(this->gimbalTipAngleRef)) {
        this->linearlyInterpolateMotorAnglesTipAngleFixed();
    } else {
        this->linearlyInterpolateMotorAnglesTiltAngleFixed();
    }
}

/*! This method determines if bilinear interpolation is required to obtain the motor angles.
 @return bool
*/
bool TwoAxisGimbalController::bilinearInterpolationRequired() {
    int compare1a = abs(this->gimbalTipAngleRef / this->tableStepAngle);
    double compare2a = abs(this->gimbalTipAngleRef / this->tableStepAngle);
    double compare3a = compare2a - compare1a;

    int compare1b = abs(this->gimbalTiltAngleRef / this->tableStepAngle);
    double compare2b = abs(this->gimbalTiltAngleRef / this->tableStepAngle);
    double compare3b = compare2b - compare1b;

    return (!(compare3a < 1e-10) && !(compare3b < 1e-10));
}

/*! This method determines if linear interpolation is required to obtain the motor angles.
 @return bool
 @param gimbalAngle [rad] gimbal angle
*/
bool TwoAxisGimbalController::linearInterpolationRequired(double gimbalAngle) {
    int compare1 = abs(gimbalAngle / this->tableStepAngle);
    double compare2 = abs(gimbalAngle / this->tableStepAngle);
    double compare3 = compare2 - compare1;

    return (compare3 < 1e-10);
}

/*! This method determines if no interpolation is required to obtain the motor angles.
 @return bool
*/
bool TwoAxisGimbalController::noInterpolationRequired() {
    int compare1a = abs(this->gimbalTipAngleRef / this->tableStepAngle);
    double compare2a = abs(this->gimbalTipAngleRef / this->tableStepAngle);
    double compare3a = compare2a - compare1a;

    int compare1b = abs(this->gimbalTiltAngleRef / this->tableStepAngle);
    double compare2b = abs(this->gimbalTiltAngleRef / this->tableStepAngle);
    double compare3b = compare2b - compare1b;

    return (compare3a < 1e-10 && compare3b < 1e-10);
}

/*! This method calls the bilinear interpolation function to interpolate the motor angles from the gimbal angles.
The case where the gimbal angles are at the edge of the interpolation table is checked and the helper method
computeTableEdgeCase() is called to determine the appropriate motor angles.
 @return void
*/
void TwoAxisGimbalController::bilinearlyInterpolateMotorAngles() {
    // Find the upper and lower interpolation table angle bounds for the gimbal angles
    double lowerTipAngle = this->tableStepAngle * floor(this->gimbalTipAngleRef / this->tableStepAngle);
    double upperTipAngle = this->tableStepAngle * ceil(this->gimbalTipAngleRef / this->tableStepAngle);
    double lowerTiltAngle = this->tableStepAngle * floor(this->gimbalTiltAngleRef / this->tableStepAngle);
    double upperTiltAngle = this->tableStepAngle * ceil(this->gimbalTiltAngleRef / this->tableStepAngle);

    // Determine the bounding motor 1 angles
    double z11_m1 = this->pullMotor1Angle(lowerTipAngle, lowerTiltAngle);
    double z12_m1 = this->pullMotor1Angle(upperTipAngle, lowerTiltAngle);
    double z21_m1 = this->pullMotor1Angle(lowerTipAngle, upperTiltAngle);
    double z22_m1 = this->pullMotor1Angle(upperTipAngle, upperTiltAngle);

    // Determine the bounding motor 2 angles
    double z11_m2 = this->pullMotor2Angle(lowerTipAngle, lowerTiltAngle);
    double z12_m2 = this->pullMotor2Angle(upperTipAngle, lowerTiltAngle);
    double z21_m2 = this->pullMotor2Angle(lowerTipAngle, upperTiltAngle);
    double z22_m2 = this->pullMotor2Angle(upperTipAngle, upperTiltAngle);

    // Check that a valid gimbal attitude was requested
    assert (!(z11_m1 < 0.0 && z12_m1 < 0.0 && z21_m1 < 0.0 && z22_m1 < 0.0));

    if (z11_m1 > 0.0 && z12_m1 > 0.0 && z21_m1 > 0.0 && z22_m1 > 0.0) {
        this->motor1Angle = bilinearInterpolation(lowerTipAngle,
                                                  upperTipAngle,
                                                  lowerTiltAngle,
                                                  upperTiltAngle,
                                                  z11_m1,
                                                  z12_m1,
                                                  z21_m1,
                                                  z22_m1,
                                                  this->gimbalTipAngleRef,
                                                  this->gimbalTiltAngleRef);

        this->motor2Angle = bilinearInterpolation(lowerTipAngle,
                                                  upperTipAngle,
                                                  lowerTiltAngle,
                                                  upperTiltAngle,
                                                  z11_m2,
                                                  z12_m2,
                                                  z21_m2,
                                                  z22_m2,
                                                  this->gimbalTipAngleRef,
                                                  this->gimbalTiltAngleRef);
    } else {
        this->computeTableEdgeCase(lowerTipAngle,
                                   upperTipAngle,
                                   lowerTiltAngle,
                                   upperTiltAngle,
                                   z11_m1,
                                   z12_m1,
                                   z21_m1,
                                   z22_m1,
                                   z11_m2,
                                   z12_m2,
                                   z21_m2,
                                   z22_m2);
    }
}

/*! Method used to determine the motor angles at the bounding outside edges of the interpolation table.
If 3/4 motor angles are valid, a 3D triangle centroid method is used to determine the motor angle
If 2/4 motor angles are valid, linear interpolation is used to determine the motor angle
If 1/4 motor angles are valid, the motor angle is directly pulled as the valid value
 @return void
 @param lowerTipAngle [rad] Lower bounding gimbal tip angle
 @param upperTipAngle [rad] Upper bounding gimbal tip angle
 @param lowerTiltAngle [rad] Lower bounding gimbal tilt angle
 @param upperTiltAngle [rad] Upper bounding gimbal tilt angle
 @param z11_m1 [rad] Bounding motor 1 angle corresponding to (lowerTipAngle, lowerTiltAngle)
 @param z12_m1 [rad] Bounding motor 1 angle corresponding to (lowerTipAngle, upperTiltAngle)
 @param z21_m1 [rad] Bounding motor 1 angle corresponding to (upperTipAngle, lowerTiltAngle)
 @param z22_m1 [rad] Bounding motor 1 angle corresponding to (upperTipAngle, upperTiltAngle)
 @param z11_m2 [rad] Bounding motor 2 angle corresponding to (lowerTipAngle, lowerTiltAngle)
 @param z12_m2 [rad] Bounding motor 2 angle corresponding to (lowerTipAngle, upperTiltAngle)
 @param z21_m2 [rad] Bounding motor 2 angle corresponding to (upperTipAngle, lowerTiltAngle)
 @param z22_m2 [rad] Bounding motor 2 angle corresponding to (upperTipAngle, upperTiltAngle)
*/
void TwoAxisGimbalController::computeTableEdgeCase(double lowerTipAngle,
                                                   double upperTipAngle,
                                                   double lowerTiltAngle,
                                                   double upperTiltAngle,
                                                   double z11_m1,
                                                   double z12_m1,
                                                   double z21_m1,
                                                   double z22_m1,
                                                   double z11_m2,
                                                   double z12_m2,
                                                   double z21_m2,
                                                   double z22_m2) {
   if (z11_m1 < 0.0 && z21_m1 < 0.0) {  // 2/4 motor angles are valid
        if (z12_m1 > 0.0 && z22_m1 > 0.0) {
            this->motor1Angle = linearInterpolation(lowerTiltAngle,
                                                    upperTiltAngle,
                                                    z12_m1,
                                                    z22_m1,
                                                    this->gimbalTiltAngleRef);
            this->motor2Angle = linearInterpolation(lowerTiltAngle,
                                                    upperTiltAngle,
                                                    z12_m2,
                                                    z22_m2,
                                                    this->gimbalTiltAngleRef);
        } else if (z12_m1 < 0.0) {  // 1/4 motor angels are valid
            this->motor1Angle = this->pullMotor1Angle(upperTipAngle, upperTiltAngle);
            this->motor2Angle = this->pullMotor2Angle(upperTipAngle, upperTiltAngle);
        } else {  // 1/4 motor angels are valid
            this->motor1Angle = this->pullMotor1Angle(upperTipAngle, lowerTiltAngle);
            this->motor2Angle = this->pullMotor2Angle(upperTipAngle, lowerTiltAngle);
        }
    } else if (z12_m1 < 0.0 && z22_m1 < 0.0) {  // 2/4 motor angles are valid
        if (z11_m1 > 0.0 && z21_m1 > 0.0) {
            this->motor1Angle = linearInterpolation(lowerTiltAngle,
                                                    upperTiltAngle,
                                                    z11_m1, z21_m1,
                                                    this->gimbalTiltAngleRef);
            this->motor2Angle = linearInterpolation(lowerTiltAngle,
                                                    upperTiltAngle,
                                                    z11_m2, z21_m2,
                                                    this->gimbalTiltAngleRef);
        } else if (z11_m1 < 0.0) {  // 1/4 motor angels are valid
            this->motor1Angle = this->pullMotor1Angle(lowerTipAngle, upperTiltAngle);
            this->motor2Angle = this->pullMotor2Angle(lowerTipAngle, upperTiltAngle);
        } else {  // 1/4 motor angels are valid
            this->motor1Angle = this->pullMotor1Angle(lowerTipAngle, lowerTiltAngle);
            this->motor2Angle = this->pullMotor2Angle(lowerTipAngle, lowerTiltAngle);
        }
    } else if (z11_m1 < 0.0) {  // 3/4 motor angles are valid: use 3d triangle centroid method
        this->motor1Angle = (z12_m1 + z21_m1 + z22_m1) / 3.0;
        this->motor2Angle = (z12_m2 + z21_m2 + z22_m2) / 3.0;
    } else if (z12_m1 < 0.0) {
        this->motor1Angle = (z11_m1 + z21_m1 + z22_m1) / 3.0;
        this->motor2Angle = (z11_m2 + z21_m2 + z22_m2) / 3.0;
    } else if (z21_m1 < 0.0) {
        this->motor1Angle = (z11_m1 + z12_m1 + z22_m1) / 3.0;
        this->motor2Angle = (z11_m2 + z12_m2 + z22_m2) / 3.0;
    } else if (z22_m1 < 0.0) {
        this->motor1Angle = (z11_m1 + z12_m1 + z21_m1) / 3.0;
        this->motor2Angle = (z11_m2 + z12_m2 + z21_m2) / 3.0;
    }

}

/*! This method calls the linear interpolation function to interpolate the gimbal angles to motor angles using a fixed gimbal tip angle and bounded gimbal tilt angle.
The case where the gimbal angles are at the edge of the interpolation table is checked in this method and the appropriate motor angles are returned for this case.
If 2/2 motor angles are valid, linear interpolation is used to determine the motor angle
If 1/2 motor angles are valid, the motor angle is directly pulled as the valid value
 @return void
*/
void TwoAxisGimbalController::linearlyInterpolateMotorAnglesTipAngleFixed() {
    // Find the upper and lower interpolation table angle bounds for the bounded gimbal angle
    double lowerGimbalAngle = this->tableStepAngle * floor(this->gimbalTiltAngleRef / this->tableStepAngle);
    double upperGimbalAngle = this->tableStepAngle * ceil(this->gimbalTiltAngleRef / this->tableStepAngle);

    // Determine the bounding motor 1 angles
    double y1_m1 = this->pullMotor1Angle(this->gimbalTipAngleRef, lowerGimbalAngle);
    double y2_m1 = this->pullMotor1Angle(this->gimbalTipAngleRef, upperGimbalAngle);

    // Determine the bounding motor 2 angles
    double y1_m2 = this->pullMotor2Angle(this->gimbalTipAngleRef, lowerGimbalAngle);
    double y2_m2 = this->pullMotor2Angle(this->gimbalTipAngleRef, upperGimbalAngle);

    // Check that a valid gimbal attitude was requested
    assert (!(y1_m1 < 0.0 && y2_m1 < 0.0));

    // Linearly interpolate the motor angles if not on the edge of the interpolation table
    if (y1_m1 > 0.0 && y2_m1 > 0.0) {
        this->motor1Angle = linearInterpolation(lowerGimbalAngle,
                                                upperGimbalAngle,
                                                y1_m1,
                                                y2_m1,
                                                this->gimbalTiltAngleRef);
        this->motor2Angle = linearInterpolation(lowerGimbalAngle,
                                                upperGimbalAngle,
                                                y1_m2,
                                                y2_m2,
                                                this->gimbalTiltAngleRef);
    } else {
        if (y1_m1 < 0.0) {
            this->motor1Angle = y2_m1;
            this->motor2Angle = y2_m2;
        } else {
            this->motor1Angle = y1_m1;
            this->motor2Angle = y1_m2;
        }
    }
}

/*! This method calls the linear interpolation function to interpolate the gimbal angles to motor angles using a fixed gimbal tilt angle and bounded gimbal tip angle.
The case where the gimbal angles are at the edge of the interpolation table is checked in this method and the appropriate motor angles are returned for this case.
If 2/2 motor angles are valid, linear interpolation is used to determine the motor angle
If 1/2 motor angles are valid, the motor angle is directly pulled as the valid value
 @return void
*/
void TwoAxisGimbalController::linearlyInterpolateMotorAnglesTiltAngleFixed() {
    // Find the upper and lower interpolation table angle bounds for the bounded gimbal angle
    double lowerGimbalAngle = this->tableStepAngle * floor(this->gimbalTipAngleRef / this->tableStepAngle);
    double upperGimbalAngle = this->tableStepAngle * ceil(this->gimbalTipAngleRef / this->tableStepAngle);

    // Determine the bounding motor 1 angles
    double y1_m1 = this->pullMotor1Angle(lowerGimbalAngle, this->gimbalTiltAngleRef);
    double y2_m1 = this->pullMotor1Angle(upperGimbalAngle, this->gimbalTiltAngleRef);

    // Determine the bounding motor 2 angles
    double y1_m2 = this->pullMotor2Angle(lowerGimbalAngle, this->gimbalTiltAngleRef);
    double y2_m2 = this->pullMotor2Angle(upperGimbalAngle, this->gimbalTiltAngleRef);

    // Linearly interpolate the motor angles if not on the edge of the interpolation table
    if (y1_m1 > 0.0 && y2_m1 > 0.0) {
        this->motor1Angle = linearInterpolation(lowerGimbalAngle,
                                                upperGimbalAngle,
                                                y1_m1,
                                                y2_m1,
                                                this->gimbalTipAngleRef);
        this->motor2Angle = linearInterpolation(lowerGimbalAngle,
                                                upperGimbalAngle,
                                                y1_m2,
                                                y2_m2,
                                                this->gimbalTipAngleRef);
    } else {
        if (y1_m1 < 0.0) {
            this->motor1Angle = y2_m1;
            this->motor2Angle = y2_m2;
        } else {
            this->motor1Angle = y1_m1;
            this->motor2Angle = y1_m2;
        }
    }
}

/*! This method pulls a specific motor 1 angle from the motor 1 interpolation table given specific gimbal angles.
 @return double
 @param gimbalTipAngle [rad] Gimbal tip angle
 @param gimbalTiltAngle [rad] Gimbal tilt angle
*/
double TwoAxisGimbalController::pullMotor1Angle(double gimbalTipAngle, double gimbalTiltAngle) const {
    int gimbalTipIdx = 40 + gimbalTipAngle / this->tableStepAngle;
    int gimbalTiltIdx = 56 + gimbalTiltAngle / this->tableStepAngle;

    return this->gimbal_to_motor_1_angle[gimbalTiltIdx][gimbalTipIdx];
}

/*! This method pulls a specific motor 2 angle from the motor 2 interpolation table given specific gimbal angles.
 @return double
 @param gimbalTipAngle [rad] Gimbal tip angle
 @param gimbalTiltAngle [rad] Gimbal tilt angle
*/
double TwoAxisGimbalController::pullMotor2Angle(double gimbalTipAngle, double gimbalTiltAngle) const {
    int gimbalTipIdx = 40 + gimbalTipAngle / this->tableStepAngle;
    int gimbalTiltIdx = 56 + gimbalTiltAngle / this->tableStepAngle;

    return this->gimbal_to_motor_2_angle[gimbalTiltIdx][gimbalTipIdx];
}

/*! Method to load the gimbal-to-motor 1 angle lookup table.
 @return void
 @param pathToMotor1Table String for the file absolute path
*/
void TwoAxisGimbalController::loadGimbalToMotor1AngleLookupTable(std::string pathToMotor1Table) {
    std::ifstream file(pathToMotor1Table);
    if (!file.is_open()) {
        _bskLog(this->bskLogger, BSK_ERROR, "Gimbal-to-motor 1 angle interpolation table was not found.");
    }
    std::string line;
    int row = -1;
    while (getline(file, line) && row < 110) {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;
        while (getline(ss, cell, ',') && col < 80) {
            if (row >= 0) {
                if (cell.empty()) {
                    cell = "-1";
                }
                this->gimbal_to_motor_1_angle[row][col] = DEG2RAD * stod(cell);
            }
            col++;
        }
        row++;
    }
    file.close();
}

/*! Method to load the gimbal-to-motor 2 angle lookup table.
 @return void
 @param pathToMotor12able String for the file absolute path
*/
void TwoAxisGimbalController::loadGimbalToMotor2AngleLookupTable(std::string pathToMotor2Table) {
    std::ifstream file(pathToMotor2Table);
    if (!file.is_open()) {
        _bskLog(this->bskLogger, BSK_ERROR, "Gimbal-to-motor 2 angle interpolation table was not found.");
    }
    std::string line;
    int row = -1;
    while (getline(file, line) && row < 110) {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;
        while (getline(ss, cell, ',') && col < 80) {
            if (row >= 0) {
                if (cell.empty()) {
                    cell = "-1";
                }
                this->gimbal_to_motor_2_angle[row][col] = DEG2RAD * stod(cell);
            }
            col++;
        }
        row++;
    }
    file.close();
}
