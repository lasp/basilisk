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
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearInterpolation.hpp"
#include "architecture/utilities/bilinearInterpolation.hpp"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "architecture/utilities/macroDefinitions.h"
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>

/*! This method initializes the output messages for this module.
 @return void
 @param this The configuration data associated with this module
 @param moduleID The module identifier
 */
TwoAxisGimbalController::TwoAxisGimbalController(std::string pathToMotor1Table, std::string pathToMotor2Table) {
    // Read the gimbal-to-motor 1 angle data file
    std::ifstream file(pathToMotor1Table);
    if (!file.is_open()) {
        _bskLog(this->bskLogger, BSK_ERROR, "Gimbal-to-motor 1 angle interpolation table was not found.");
    }
    std::string line;
    int row = -1;
    while (getline(file, line) && row < 111) {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;
        while (getline(ss, cell, ',') && col < 80) {
            if (row >= 0) {
                if (cell == "" || cell == " ") {
                    cell = "-1";
                    std::cout<< cell << std::endl;
                    this->gimbal_to_motor_1_angle[row][col] = -1;
                }
                else {
                    std::cout<< cell << std::endl;
                    this->gimbal_to_motor_1_angle[row][col] = DEG2RAD * stod(cell);
                }

            }
            col++;
        }
        row++;
    }
    file.close();

    // Read the gimbal-to-motor 2 angle data file
    std::ifstream file2(pathToMotor2Table);
    if (!file2.is_open()) {
        _bskLog(this->bskLogger, BSK_ERROR, "Gimbal-to-motor 2 angle interpolation table was not found.");
    }
    row = -1;
    while (getline(file2, line) && row < 111) {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;
        while (getline(ss, cell, ',') && col < 80) {
            if (row >= 0) {
                if (cell == "") {
                    cell = "-1";
                }
                this->gimbal_to_motor_2_angle[row][col] = DEG2RAD * stod(cell);
            }
            col++;
        }
        row++;
    }
    file2.close();
}


/*! This method performs a complete reset of the module. The input messages are checked to ensure they are linked.
 @return void
 @param this The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/

void TwoAxisGimbalController::Reset(uint64_t callTime) {
    if (!this->twoAxisGimbalInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "twoAxisGimbalController.twoAxisGimbalInMsg wasn't connected.");
    }
}

/*! This method recieves the desired tip and tilt angles and outputs the motor angles.
The desired gimbal angles are then written to the output message to find the corresponding motor angles from the lookup table.
 @return void
 @param callTime [ns] The current time of simulation
*/
void TwoAxisGimbalController::UpdateState(uint64_t callTime) {

    // Read the input messages
    TwoAxisGimbalMsgPayload twoAxisGimbalIn = TwoAxisGimbalMsgPayload();
    if (this->twoAxisGimbalInMsg.isWritten()) {
        twoAxisGimbalIn = this->twoAxisGimbalInMsg();
        this->gimbalTipAngleRef = twoAxisGimbalIn.theta1;
        this->gimbalTiltAngleRef = twoAxisGimbalIn.theta2;
    }

    // Use the provided gimbal angles to determine the motor angles
    this->gimbalAnglesToMotorAngles();

    // Write the module output messages
    HingedRigidBodyMsgPayload motor1AngleOut = HingedRigidBodyMsgPayload();
    HingedRigidBodyMsgPayload motor2AngleOut = HingedRigidBodyMsgPayload();
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
        this->motor1Angle = this->pullMotor1Angle(this->gimbalTipAngleRef, this->gimbalTiltAngleRef);
        this->motor2Angle = this->pullMotor2Angle(this->gimbalTipAngleRef, this->gimbalTiltAngleRef);
    } else if (this->linearInterpolationRequired(this->gimbalTipAngleRef)) {
        this->linearlyInterpolateMotorAnglesTipAngleFixed();
    } else {
        this->linearlyInterpolateMotorAnglesTiltAngleFixed();
    }
}

/*! This method determines if bilinear interpolation is required to obtain the gimbal angles.
 @return bool
*/
bool TwoAxisGimbalController::bilinearInterpolationRequired() {
    int compare1a = this->gimbalTipAngleRef / this->tableStepAngle;
    double compare2a = this->gimbalTipAngleRef / this->tableStepAngle;
    double compare3a = compare2a - compare1a;

    int compare1b = this->gimbalTiltAngleRef / this->tableStepAngle;
    double compare2b = this->gimbalTiltAngleRef / this->tableStepAngle;
    double compare3b = compare2b - compare1b;

    return (!(compare3a < 1e-10) && !(compare3b < 1e-10));
}

/*! This method determines if linear interpolation is required to obtain the motor angles.
 @return bool
*/
bool TwoAxisGimbalController::linearInterpolationRequired(double gimbalAngle) {
    int compare1 = gimbalAngle / this->tableStepAngle;
    double compare2 = gimbalAngle / this->tableStepAngle;
    double compare3 = compare2 - compare1;

    return (compare3 < 1e-10);
}

/*! This method determines if no interpolation is required to obtain the motor angles.
 @return bool
*/
bool TwoAxisGimbalController::noInterpolationRequired() {
    int compare1a = this->gimbalTipAngleRef / this->tableStepAngle;
    double compare2a = this->gimbalTipAngleRef / this->tableStepAngle;
    double compare3a = compare2a - compare1a;

    int compare1b = this->gimbalTiltAngleRef / this->tableStepAngle;
    double compare2b = this->gimbalTiltAngleRef / this->tableStepAngle;
    double compare3b = compare2b - compare1b;

    return (compare3a < 1e-10 && compare3b < 1e-10);
}

void TwoAxisGimbalController::bilinearlyInterpolateMotorAngles() {
    // Find the upper and lower interpolation table angle bounds for the gimbal angles
    double lowerTipAngle = this->tableStepAngle * floor(this->gimbalTipAngleRef / this->tableStepAngle);
    double upperTipAngle = this->tableStepAngle * ceil(this->gimbalTipAngleRef / this->tableStepAngle);
    double lowerTiltAngle = this->tableStepAngle * floor(this->gimbalTiltAngleRef / this->tableStepAngle);
    double upperTiltAngle = this->tableStepAngle * ceil(this->gimbalTiltAngleRef / this->tableStepAngle);

    // Bilinearly interpolate the motor 1 angle
    double z11_m1 = this->pullMotor1Angle(lowerTipAngle, lowerTiltAngle);
    double z12_m1 = this->pullMotor1Angle(lowerTipAngle, upperTiltAngle);
    double z21_m1 = this->pullMotor1Angle(upperTipAngle, lowerTiltAngle);
    double z22_m1 = this->pullMotor1Angle(upperTipAngle, upperTiltAngle);

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
    } else if (z11_m1 < 0.0 && z21_m1 < 0.0) {
        if (z12_m1 > 0.0 && z22_m1 > 0.0) {
            this->motor1Angle = linearInterpolation(lowerTiltAngle, upperTiltAngle, z12_m1, z22_m1, this->gimbalTiltAngleRef);
        } else if (z12_m1 < 0.0) {
            this->motor1Angle = this->pullMotor1Angle(upperTipAngle, upperTiltAngle);
        } else {
            this->motor1Angle = this->pullMotor1Angle(lowerTipAngle, upperTiltAngle);
        }
    } else if (z12_m1 < 0.0 && z22_m1 < 0.0) {
        if (z11_m1 > 0.0 && z21_m1 > 0.0) {
            this->motor1Angle = linearInterpolation(lowerTiltAngle, upperTiltAngle, z11_m1, z21_m1, this->gimbalTiltAngleRef);
        } else if (z11_m1 < 0.0) {
            this->motor1Angle = this->pullMotor1Angle(upperTipAngle, lowerTiltAngle);
        } else {
            this->motor1Angle = this->pullMotor1Angle(lowerTipAngle, lowerTiltAngle);
        }
    } else if (z11_m1 < 0.0) {
        this->motor1Angle = this->trilinearInterpolation(z12_m1, z21_m1, z22_m1);
    } else if (z12_m1 < 0.0) {
        this->motor1Angle = this->trilinearInterpolation(z11_m1, z21_m1, z22_m1);
    } else if (z21_m1 < 0.0) {
        this->motor1Angle = this->trilinearInterpolation(z11_m1, z12_m1, z22_m1);
    } else if (z22_m1 < 0.0) {
        this->motor1Angle = this->trilinearInterpolation(z11_m1, z12_m1, z21_m1);
    }

    // Bilinearly interpolate the motor 2 angle
    double z11_m2 = this->pullMotor2Angle(lowerTipAngle, lowerTiltAngle);
    double z12_m2 = this->pullMotor2Angle(lowerTipAngle, upperTiltAngle);
    double z21_m2 = this->pullMotor2Angle(upperTipAngle, lowerTiltAngle);
    double z22_m2 = this->pullMotor2Angle(upperTipAngle, upperTiltAngle);

    if (z11_m2 > 0.0 && z12_m2 > 0.0 && z21_m2 > 0.0 && z22_m2 > 0.0) {
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
    } else if (z11_m2 < 0.0 && z21_m2 < 0.0) {
        if (z12_m2 > 0.0 && z22_m2 > 0.0) {
            this->motor2Angle = linearInterpolation(lowerTiltAngle, upperTiltAngle, z12_m2, z22_m2, this->gimbalTiltAngleRef);
        } else if (z12_m2 < 0.0) {
            this->pullMotor2Angle(upperTipAngle, upperTiltAngle);
        } else {
            this->pullMotor2Angle(lowerTipAngle, upperTiltAngle);
        }
    } else if (z12_m2 < 0.0 && z22_m2 < 0.0) {
        if (z11_m2 > 0.0 && z21_m2 > 0.0) {
            this->motor2Angle = linearInterpolation(lowerTiltAngle, upperTiltAngle, z11_m2, z21_m2, this->gimbalTiltAngleRef);
        } else if (z11_m2 < 0.0) {
            this->pullMotor2Angle(upperTipAngle, lowerTiltAngle);
        } else {
            this->pullMotor2Angle(lowerTipAngle, lowerTiltAngle);
        }
    } else if (z11_m2 < 0.0) {
        this->motor2Angle = this->trilinearInterpolation(z12_m2, z21_m2, z22_m2);
    } else if (z12_m2 < 0.0) {
        this->motor2Angle = this->trilinearInterpolation(z11_m2, z21_m2, z22_m2);
    } else if (z21_m2 < 0.0) {
        this->motor2Angle = this->trilinearInterpolation(z11_m2, z12_m2, z22_m2);
    } else if (z22_m2 < 0.0) {
        this->motor2Angle = this->trilinearInterpolation(z11_m2, z12_m2, z21_m2);
    }
}

double TwoAxisGimbalController::trilinearInterpolation(double z1, double z2, double z3) {
    return (z1 + z2 + z3) / 3;
}

void TwoAxisGimbalController::linearlyInterpolateMotorAnglesTipAngleFixed() {
    // Find the upper and lower interpolation table angle bounds for the bounded gimbal angle
    double lowerGimbalAngle = this->tableStepAngle * floor(this->gimbalTiltAngleRef / this->tableStepAngle);
    double upperGimbalAngle = this->tableStepAngle * ceil(this->gimbalTiltAngleRef / this->tableStepAngle);

    // Linearly interpolate the motor 1 angle
    double y1_m1 = this->pullMotor1Angle(this->gimbalTipAngleRef, lowerGimbalAngle);
    double y2_m1 = this->pullMotor1Angle(this->gimbalTipAngleRef, upperGimbalAngle);
    this->motor1Angle = linearInterpolation(lowerGimbalAngle, upperGimbalAngle, y1_m1, y2_m1, this->gimbalTiltAngleRef);

    // Linearly interpolate the motor 2
    double y1_m2 = this->pullMotor2Angle(this->gimbalTipAngleRef, lowerGimbalAngle);
    double y2_m2 = this->pullMotor2Angle(this->gimbalTipAngleRef, upperGimbalAngle);
    this->motor2Angle = linearInterpolation(lowerGimbalAngle, upperGimbalAngle, y1_m2, y2_m2, this->gimbalTiltAngleRef);
}

void TwoAxisGimbalController::linearlyInterpolateMotorAnglesTiltAngleFixed() {
    // Find the upper and lower interpolation table angle bounds for the bounded gimbal angle
    double lowerGimbalAngle = this->tableStepAngle * floor(this->gimbalTipAngleRef / this->tableStepAngle);
    double upperGimbalAngle = this->tableStepAngle * ceil(this->gimbalTipAngleRef / this->tableStepAngle);

    // Linearly interpolate the motor 1 angle
    double y1_m1 = this->pullMotor1Angle(lowerGimbalAngle, this->gimbalTiltAngleRef);
    double y2_m1 = this->pullMotor1Angle(upperGimbalAngle, this->gimbalTiltAngleRef);
    this->motor1Angle = linearInterpolation(lowerGimbalAngle, upperGimbalAngle, y1_m1, y2_m1, this->gimbalTipAngleRef);

    // Linearly interpolate the motor 2 angle
    double y1_m2 = this->pullMotor2Angle(lowerGimbalAngle, this->gimbalTiltAngleRef);
    double y2_m2 = this->pullMotor2Angle(upperGimbalAngle, this->gimbalTiltAngleRef);
    this->motor2Angle = linearInterpolation(lowerGimbalAngle, upperGimbalAngle, y1_m2, y2_m2, this->gimbalTipAngleRef);
}

/*! This method pulls a specific motor 1 angle from the motor 1 interpolation table given specific gimbal angles.
 @return double
 @param gimbalTipAngle [rad] Gimbal tip angle
 @param gimbalTiltAngle [rad] Gimbal tilt angle
*/
double TwoAxisGimbalController::pullMotor1Angle(double gimbalTipAngle, double gimbalTiltAngle) {
    int gimbalTipIdx = 40 + gimbalTipAngle / this->tableStepAngle;
    int gimbalTiltIdx = 56 + gimbalTiltAngle / this->tableStepAngle;
    return this->gimbal_to_motor_1_angle[gimbalTiltIdx][gimbalTipIdx];
}

/*! This method pulls a specific motor 2 angle from the motor 2 interpolation table given specific gimbal angles.
 @return double
 @param gimbalTipAngle [rad] Gimbal tip angle
 @param gimbalTiltAngle [rad] Gimbal tilt angle
*/
double TwoAxisGimbalController::pullMotor2Angle(double gimbalTipAngle, double gimbalTiltAngle) {
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
    while (getline(file, line) && row < 111) {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;
        while (getline(ss, cell, ',') && col < 80) {
            if (row >= 0) {
                if (cell == "") {
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
    while (getline(file, line) && row < 111) {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;
        while (getline(ss, cell, ',') && col < 80) {
            if (row >= 0) {
                if (cell == "") {
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
