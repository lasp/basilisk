/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

/* Import the module header file */
#include "sepGimbalLookUpTable.h"
#include "platformToMotor1.h"
#include "platformToMotor2.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <cstdio>
#include <stdbool.h>
#include <math.h>
#include <iostream>
#include <memory>
#include <fstream>
#include <sstream>
#include <vector>
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"
// #include "fswAlgorithms\effectorInterfaces\sepGimbalLookUpTable\platformToMotor1.h"
// #include "fswAlgorithms\effectorInterfaces\sepGimbalLookUpTable\platformToMotor2.h"

using namespace std;
#define MAX_ROWS 114
#define MAX_COLUMNS 82

#define WINDOWS  /* uncomment this line to use it for windows.*/
#ifdef WINDOWS

#include <direct.h>

#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

/*! This method initializes the output messages for this module.
 @return void
 @param this The configuration data associated with this module
 @param moduleID The module identifier
 */

void SepGimbalLookUpTable::SelfInit() {
    // Initialize the output messages
    HingedRigidBodyMsg_C_init(&this->motor1AngleOutMsg);
    HingedRigidBodyMsg_C_init(&this->motor2AngleOutMsg);
}

/*! This method performs a complete reset of the module. The input messages are checked to ensure they are linked.
 @return void
 @param this The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/
void SepGimbalLookUpTable::Reset(uint64_t callTime) {

    // Check if the required input message is linked
    if (!HingedRigidBodyMsg_C_isLinked(&this->desiredGimbalTipAngleInMsg)) {
        _bskLog(this->bskLogger, BSK_ERROR, "Error: desiredGimbalTipAngleInMsg wasn't connected.");
    }

    if (!HingedRigidBodyMsg_C_isLinked(&this->desiredGimbalTiltAngleInMsg)) {
        _bskLog(this->bskLogger, BSK_ERROR, "Error: desiredGimbalTiltAngleInMsg wasn't connected.");
    }

    this->motor1Table(motorA1);
    std::cout << "Number of rows in motorA1: " << motorA1.rows() << std::endl;
    std::cout << "Number of columns in motorA1: " << motorA1.cols() << std::endl;
    std::cout << "Value of motorA1(59, 62): " << motorA1(59, 62) << std::endl;
    this->motor2Table(motorA2);

    // Initialize the module parameters to zero
    this->motor1Angle = 0.0;
    this->motor2Angle = 0.0;
}
/*! This method recieves the desired tip and tilt angles and outputs the motor angles.
The desired gimbal angles are then written to the output message to find the corresponding motor angles from the lookup table.
 @return void
 @param callTime [ns] The current time of simulation
*/

void SepGimbalLookUpTable::UpdateState(uint64_t callTime) {

    // // Create the buffer messages
    // HingedRigidBodyMsgPayload desiredGimbalTipAngleIn;
    // HingedRigidBodyMsgPayload desiredGimbalTiltAngleIn;
    // HingedRigidBodyMsgPayload motor1AngleOut;
    // //HingedRigidBodyMsgPayload motor2AngleOut;

    // // Zero the output messages
    // motor1AngleOut = HingedRigidBodyMsg_C_zeroMsgPayload();
    // //motor2AngleOut = HingedRigidBodyMsg_C_zeroMsgPayload();

    this->readGimbalAnglesInputMsg();
    this->readMessage();
    std::cout << "Input Tip Angle Message:" <<" " <<this->tipAngle << " " << "Input Tilt Angle Message:" <<" " << this->tiltAngle << std::endl;
    
    //Functions of Motor 1
    this->findMatchingTiltAngleAndIndeciesOfMotor1(tiltAngle, tipAngle, motorA1);
    std::cout << "Nearest Tilt 1 M1: " << nearestTilt1_M1 << " (Index: " << nearestTilt1_Index_M1 << ")" << std::endl;
    std::cout << "Nearest Tilt 2 M1: " << nearestTilt2_M1 << " (Index: " << nearestTilt2_Index_M1 << ")" << std::endl;
    std::cout << "Nearest Tilt 3 M1: " << nearestTilt3_M1 << " (Index: " << nearestTilt3_Index_M1 << ")" << std::endl;

    this->findMatchingTipAngleAndIndeciesOfMotor1(tiltAngle,  tipAngle, motorA1);
    std::cout << "Nearest Tip 1 M1 : " << nearestTip1_M1 << " (Index: " << nearestTip1_Index_M1 << ")" << std::endl;
    std::cout << "Nearest Tip 2 M1 : " << nearestTip2_M1 << " (Index: " << nearestTip2_Index_M1 << ")" << std::endl;
    std::cout << "Nearest Tip 3 M1 : " << nearestTip3_M1 << " (Index: " << nearestTip3_Index_M1 << ")" << std::endl;

    std::cout << "Tilt Angle: " << tiltAngle << std::endl;
    this->findMotor1Angles(nearestTilt1_Index_M1 ,  nearestTilt2_Index_M1, nearestTip1_Index_M1, nearestTip2_Index_M1, motorA1);
    std::cout << "(z11 M1 ="<< " " << this->z11_M1 <<")"<< "  " <<  "(z12 M1 ="<<" " << this->z12_M1 <<")"<< " " << "(z21 M1 ="<<" " << this->z21_M1 <<")"<<" " << "(z22 M1 ="<<" "<< this->z22_M1<<")" <<" " << "(z33 M1 ="<<" "<< this->z33_M1<<")" << std::endl;
    std::cout << "Tip Angle: " << tipAngle << std::endl;
    double result1 = this->bilinearInterpolationMotor1(tiltAngle,  tipAngle,  z11_M1,  z12_M1,  z21_M1,  z22_M1, nearestTip1_M1, nearestTip2_M1, nearestTilt1_M1, nearestTilt2_M1);
    std::cout << "Motor1 using bilinear = " << result1 << std::endl;
    double result31 =this->triangularInterpolationMotor1( tiltAngle, tipAngle, z11_M1, z12_M1, z21_M1, z22_M1, z33_M1, nearestTip1_M1, nearestTip2_M1, nearestTip3_M1, nearestTilt1_M1, nearestTilt2_M1, nearestTilt3_M1);
    std::cout << "Motor1 using triangular = " << result31 << std::endl;

    //Functions of Motor 2
    this->findMatchingTiltAngleAndIndeciesOfMotor2(tiltAngle, tipAngle, motorA2);
    std::cout << "Nearest Tilt 1 M2: " << nearestTilt1_M2 << " (Index: " << nearestTilt1_Index_M2 << ")" << std::endl;
    std::cout << "Nearest Tilt 2 M2: " << nearestTilt2_M2 << " (Index: " << nearestTilt2_Index_M2 << ")" << std::endl;
    std::cout << "Nearest Tilt 3 M2: " << nearestTilt3_M2 << " (Index: " << nearestTilt3_Index_M2 << ")" << std::endl;

    this->findMatchingTipAngleAndIndeciesOfMotor2(tiltAngle,  tipAngle, motorA2);
    std::cout << "Nearest Tip 1 M2 : " << nearestTip1_M2 << " (Index: " << nearestTip1_Index_M2 << ")" << std::endl;
    std::cout << "Nearest Tip 2 M2 : " << nearestTip2_M2 << " (Index: " << nearestTip2_Index_M2 << ")" << std::endl;
    std::cout << "Nearest Tip 3 M2 : " << nearestTip3_M2 << " (Index: " << nearestTip3_Index_M2 << ")" << std::endl;

    this->findMotor2Angles(nearestTilt1_Index_M2 ,  nearestTilt2_Index_M2, nearestTip1_Index_M2, nearestTip2_Index_M2, motorA2);
    std::cout << "(z11 M2= "<< " " << this->z11_M2 << ")"<<"   " <<  "(z12 M2="<<" " << this->z12_M2 << ")"<<"  " << "(z21 M2="<<" " << this->z21_M2 <<")"<<"  " << "(z22 M2="<<" "<< this->z22_M2<<")" <<" " << "(z33 M2 ="<<" "<< this->z33_M2<<")" << std::endl;
    double result2 = this->bilinearInterpolationMotor2(tiltAngle,  tipAngle,  z11_M2,  z12_M2,  z21_M2,  z22_M2, nearestTip1_M2, nearestTip2_M2, nearestTilt1_M2, nearestTilt2_M2);
    std::cout << "Motor2 using bilinear == " << result2 << std::endl;
    double result32 = this->triangularInterpolationMotor2( tiltAngle, tipAngle, z11_M2, z12_M2, z21_M2, z22_M2, z33_M2, nearestTip1_M2, nearestTip2_M2, nearestTip3_M2, nearestTilt1_M2, nearestTilt2_M2, nearestTilt3_M2);
    std::cout << "Motor2 using triangular == " << result32 << std::endl;




    //this->writeOutputMotorMessages(motorAngle1);
}

void SepGimbalLookUpTable::motor1Table(Eigen::Matrix<double, 114, 82>& motorA1) {
    Eigen::Map<Eigen::Matrix<double, 114, 82, Eigen::RowMajor>> motorA1Map(motorAngle1Table);
    motorA1 = motorA1Map;
}

void SepGimbalLookUpTable::motor2Table(Eigen::Matrix<double, 114, 82>& motorA2) {
    Eigen::Map<Eigen::Matrix<double, 114, 82, Eigen::RowMajor>> motorA2Map(motorAngle2Table);
    motorA2 = motorA2Map;
}

void SepGimbalLookUpTable::readGimbalAnglesInputMsg() {
    // Read the gimbal tip angle input message
    HingedRigidBodyMsgPayload desiredGimbalTipAngleIn = HingedRigidBodyMsg_C_zeroMsgPayload();
    if (HingedRigidBodyMsg_C_isWritten(&this->desiredGimbalTipAngleInMsg)) {
        desiredGimbalTipAngleIn = HingedRigidBodyMsg_C_read(&this->desiredGimbalTipAngleInMsg);
        this->tipAngle = desiredGimbalTipAngleIn.theta;
    }

    // Read the gimbal tilt angle input message
    HingedRigidBodyMsgPayload desiredGimbalTiltAngleIn = HingedRigidBodyMsg_C_zeroMsgPayload();
    if (HingedRigidBodyMsg_C_isWritten(&this->desiredGimbalTiltAngleInMsg)) {
        desiredGimbalTiltAngleIn = HingedRigidBodyMsg_C_read(&this->desiredGimbalTiltAngleInMsg);
        this->tiltAngle = desiredGimbalTiltAngleIn.theta;
    }
}

void SepGimbalLookUpTable::readMessage() {
    // Read the gimbal tip angle input message
    HingedRigidBodyMsgPayload desiredGimbalTipAngleIn = HingedRigidBodyMsg_C_zeroMsgPayload();
    if (HingedRigidBodyMsg_C_isWritten(&this->desiredGimbalTipAngleInMsg)) {
    desiredGimbalTipAngleIn = HingedRigidBodyMsg_C_read(&this->desiredGimbalTipAngleInMsg);
    this->tipAngle = desiredGimbalTipAngleIn.theta;
    }

    // Read the gimbal tilt angle input message
    HingedRigidBodyMsgPayload desiredGimbalTiltAngleIn = HingedRigidBodyMsg_C_zeroMsgPayload();
    if (HingedRigidBodyMsg_C_isWritten(&this->desiredGimbalTiltAngleInMsg)) {
        desiredGimbalTiltAngleIn = HingedRigidBodyMsg_C_read(&this->desiredGimbalTiltAngleInMsg);
        this->tiltAngle = desiredGimbalTiltAngleIn.theta;
    }

}

// Function to find the nearest 2 rows with equal or nearest tilt angle value
void SepGimbalLookUpTable::findMatchingTiltAngleAndIndeciesOfMotor1(double tiltAngle, double tipAngle, Eigen::Matrix<double, 114, 82>& motorA1) {
    
    // Initialize variables
    double difference = DBL_MAX; 
    double minDifference1 = DBL_MAX; 
    double minDifference2 = DBL_MAX; 
    double minDifference3 = DBL_MAX; 
    this->nearestTilt1_M1 = 0.0;
    this->nearestTilt2_M1 = 0.0;
    this->nearestTilt3_M1 = 0.0;
    this->nearestTilt1_Index_M1 = 0;
    this->nearestTilt2_Index_M1 = 0;
    this->nearestTilt3_Index_M1 = 0;

    // Iterate through all columns to find the two nearest tip angles
    for (int rowIndex = 0; rowIndex < 82; ++rowIndex) {
    double currentTiltAngle = motorA1(rowIndex,0);
    double difference = fabs(tiltAngle - currentTiltAngle); 
    //std::cout << "Column Index: " << columnIndex << ", Current Tip Angle: " << currentTipAngle << ", Difference: " << difference << std::endl;
    
    // Update nearestTip1, nearestTip2, and nearestTip3 along with their indices
        if (difference < minDifference1) {
        minDifference3 = minDifference2;
        nearestTilt3_M1 = nearestTilt2_M1;
        nearestTilt3_Index_M1 = nearestTilt2_Index_M1;
        
        minDifference2 = minDifference1;
        nearestTilt2_M1 = nearestTilt1_M1;
        nearestTilt2_Index_M1 = nearestTilt1_Index_M1;
        
        minDifference1 = difference;
        nearestTilt1_M1 = currentTiltAngle;
        nearestTilt1_Index_M1 = rowIndex;
        } else if (difference < minDifference2) {
        minDifference3 = minDifference2;
        nearestTilt3_M1 = nearestTilt2_M1;
        nearestTilt3_Index_M1 = nearestTilt2_Index_M1;
        
        minDifference2 = difference;
        nearestTilt2_M1 = currentTiltAngle;
        nearestTilt2_Index_M1 = rowIndex;
        } else if (difference < minDifference3) {
        minDifference3 = difference;
        nearestTilt3_M1 = currentTiltAngle;
        nearestTilt3_Index_M1 = rowIndex;
        }
    }
    //std::cout << "final Nearest Tip 1: " << nearestTip1 << " (Index: " << nearestTip1_Index << ")" << std::endl;
    //std::cout << "final Nearest Tip 2: " << nearestTip2 << " (Index: " << nearestTip2_Index << ")" << std::endl;
};

// Function to find the nearest 2 rows with equal or nearest tilt angle value
void SepGimbalLookUpTable::findMatchingTipAngleAndIndeciesOfMotor1(double tiltAngle, double tipAngle, Eigen::Matrix<double, 114, 82>& motorA1) {
    double difference = DBL_MAX; 
    double minDifference1 = DBL_MAX; 
    double minDifference2 = DBL_MAX; 
    double minDifference3 = DBL_MAX;  
    this->nearestTip1_M1 = 0.0;
    this->nearestTip2_M1 = 0.0;
    this->nearestTip3_M1 = 0.0;
    this->nearestTip1_Index_M1 = 0;
    this->nearestTip2_Index_M1 = 0;
    this->nearestTip3_Index_M1 = 0;

    // Iterate through all columns to find the two nearest tip angles
    // Update nearestTip1, nearestTip2, and nearestTip3 along with their indices
    // Iterate through all columns to find the two nearest tip angles
    for (int columnIndex = 0; columnIndex < 82; ++columnIndex) {
        double currentTipAngle = motorA1(0, columnIndex);
        double difference = fabs(tipAngle - currentTipAngle); 
       
        if (difference < minDifference1) {
        minDifference3 = minDifference2;
        nearestTip3_M1 = nearestTip2_M1;
        nearestTip3_Index_M1 = nearestTip2_Index_M1;
        
        minDifference2 = minDifference1;
        nearestTip2_M1 = nearestTip1_M1;
        nearestTip2_Index_M1 = nearestTip1_Index_M1;
        
        minDifference1 = difference;
        nearestTip1_M1 = currentTipAngle;
        nearestTip1_Index_M1 = columnIndex;
        } else if (difference < minDifference2) {
        minDifference3 = minDifference2;
        nearestTip3_M1 = nearestTip2_M1;
        nearestTip3_Index_M1 = nearestTip2_Index_M1;
        
        minDifference2 = difference;
        nearestTip2_M1 = currentTipAngle;
        nearestTip2_Index_M1 = columnIndex;
        } else if (difference < minDifference3) {
        minDifference3 = difference;
        nearestTip3_M1 = currentTipAngle;
        nearestTip3_Index_M1 = columnIndex;
        }
    }
    //std::cout << "final Nearest Tip 1: " << nearestTip1 << " (Index: " << nearestTip1_Index << ")" << std::endl;
    //std::cout << "final Nearest Tip 2: " << nearestTip2 << " (Index: " << nearestTip2_Index << ")" << std::endl;
};

void SepGimbalLookUpTable::findMatchingTiltAngleAndIndeciesOfMotor2(double tiltAngle, double tipAngle, Eigen::Matrix<double, 114, 82>& motorA2) {
    
    // Initialize variables
    double difference = DBL_MAX;
    double minDifference1 = DBL_MAX;
    double minDifference2 = DBL_MAX; 
    double minDifference3 = DBL_MAX; 
    this->nearestTilt1_M2 = 0.0;
    this->nearestTilt2_M2 = 0.0;
    this->nearestTilt3_M2 = 0.0;
    this->nearestTilt1_Index_M2 = 0;
    this->nearestTilt2_Index_M2 = 0;
    this->nearestTilt3_Index_M2 = 0;

    // Iterate through all rows to find the two nearest tilt angles
    for (int rowIndex = 0; rowIndex < 114; ++rowIndex) {
    double currentTiltAngle = motorA2(rowIndex, 0);
    double difference = fabs(tiltAngle - currentTiltAngle);
    //std::cout << "Row Index: " << rowIndex << ", Current Tilt Angle: " << currentTiltAngle << ", Difference: " << difference << std::endl;

    // Update nearestTilt1, nearestTilt2, and nearestTilt3 along with their indices
    if (difference < minDifference1) {
        minDifference3 = minDifference2;
        nearestTilt3_M2 = nearestTilt2_M2;
        nearestTilt3_Index_M2 = nearestTilt2_Index_M2;
        
        minDifference2 = minDifference1;
        nearestTilt2_M2 = nearestTilt1_M2;
        nearestTilt2_Index_M2 = nearestTilt1_Index_M2;
        
        minDifference1 = difference;
        nearestTilt1_M2 = currentTiltAngle;
        nearestTilt1_Index_M2 = rowIndex;
    } else if (difference < minDifference2) {
        minDifference3 = minDifference2;
        nearestTilt3_M2 = nearestTilt2_M2;
        nearestTilt3_Index_M2 = nearestTilt2_Index_M2;
        
        minDifference2 = difference;
        nearestTilt2_M2 = currentTiltAngle;
        nearestTilt2_Index_M2 = rowIndex;
    } else if (difference < minDifference3) {
        minDifference3 = difference;
        nearestTilt3_M2 = currentTiltAngle;
        nearestTilt3_Index_M2 = rowIndex;
    }
}
    
    //std::cout << "final Nearest Tilt 1: " << nearestTilt1_M2 << " (Index: " << nearestTilt1_Index_M2 << ")" << std::endl;
    //std::cout << "final Nearest Tilt 2: " << nearestTilt2_M2 << " (Index: " << nearestTilt2_Index_M2 << ")" << std::endl;
}

// Function to find the nearest 2 rows with equal or nearest tilt angle value
void SepGimbalLookUpTable::findMatchingTipAngleAndIndeciesOfMotor2(double tiltAngle, double tipAngle, Eigen::Matrix<double, 114, 82>& motorA2) {
    double difference = DBL_MAX; 
    double minDifference1 = DBL_MAX; 
    double minDifference2 = DBL_MAX; 
    double minDifference3 = DBL_MAX; 
    this->nearestTip1_M2 = 0.0;
    this->nearestTip2_M2 = 0.0;
    this->nearestTip3_M2 = 0.0;
    this->nearestTip1_Index_M2 = 0;
    this->nearestTip2_Index_M2 = 0;
    this->nearestTip3_Index_M2 = 0;

    // Iterate through all columns to find the two nearest tip angles
    for (int columnIndex = 0; columnIndex < 82; ++columnIndex) {
    double currentTipAngle = motorA2(0, columnIndex);
    double difference = fabs(tipAngle - currentTipAngle); 
    //std::cout << "Column Index: " << columnIndex << ", Current Tip Angle: " << currentTipAngle << ", Difference: " << difference << std::endl;
    
    // Update nearestTip1, nearestTip2, and nearestTip3 along with their indices
    if (difference < minDifference1) {
        minDifference3 = minDifference2;
        nearestTip3_M2 = nearestTip2_M2;
        nearestTip3_Index_M2 = nearestTip2_Index_M2;
        
        minDifference2 = minDifference1;
        nearestTip2_M2 = nearestTip1_M2;
        nearestTip2_Index_M2 = nearestTip1_Index_M2;
        
        minDifference1 = difference;
        nearestTip1_M2 = currentTipAngle;
        nearestTip1_Index_M2 = columnIndex;
        } else if (difference < minDifference2) {
        minDifference3 = minDifference2;
        nearestTip3_M2 = nearestTip2_M2;
        nearestTip3_Index_M2 = nearestTip2_Index_M2;
        
        minDifference2 = difference;
        nearestTip2_M2 = currentTipAngle;
        nearestTip2_Index_M2 = columnIndex;
        } else if (difference < minDifference3) {
        minDifference3 = difference;
        nearestTip3_M2 = currentTipAngle;
        nearestTip3_Index_M2 = columnIndex;
        }
    }
    //std::cout << "final Nearest Tip 1: " << nearestTip1 << " (Index: " << nearestTip1_Index << ")" << std::endl;
    //std::cout << "final Nearest Tip 2: " << nearestTip2 << " (Index: " << nearestTip2_Index << ")" << std::endl;
}

void SepGimbalLookUpTable::findMotor1Angles(int nearestTilt1_Index_M1, int nearestTilt2_Index_M1, int nearestTip1_Index_M1, int nearestTip2_Index_M1, Eigen::Matrix<double, 114, 82>& motorA1) {
    // Useing the indices to extract the motor angles from the motorA1 matrix
    this->z11_M1 = motorA1(nearestTilt1_Index_M1, nearestTip1_Index_M1);
    this->z12_M1 = motorA1(nearestTilt1_Index_M1, nearestTip2_Index_M1);
    this->z21_M1 = motorA1(nearestTilt2_Index_M1, nearestTip1_Index_M1);
    this->z22_M1 = motorA1(nearestTilt2_Index_M1, nearestTip2_Index_M1);
    this->z33_M1 = motorA1(nearestTilt3_Index_M1, nearestTip3_Index_M1);
}

void SepGimbalLookUpTable::findMotor2Angles(int nearestTilt1_Index_M2, int nearestTilt2_Index_M2, int nearestTip1_Index_M2, int nearestTip2_Index_M2, Eigen::Matrix<double, 114, 82>& motorA2) {
    // Useing the indices to extract the motor angles from the motorA2 matrix
    this->z11_M2 = motorA2(nearestTilt1_Index_M2, nearestTip1_Index_M2 );
    this->z12_M2 = motorA2(nearestTilt1_Index_M2, nearestTip2_Index_M2);
    this->z21_M2 = motorA2(nearestTilt2_Index_M2, nearestTip1_Index_M2 );
    this->z22_M2 = motorA2(nearestTilt2_Index_M2, nearestTip2_Index_M2);
    this->z33_M2 = motorA2(nearestTilt3_Index_M1, nearestTip3_Index_M1);
}

double SepGimbalLookUpTable::bilinearInterpolationMotor1(double tiltAngle,
                                                            double tipAngle,
                                                            double z11_M1,
                                                            double z12_M1,
                                                            double z21_M1,
                                                            double z22_M1,
                                                            double nearestTip1_M1,
                                                            double nearestTip2_M1,
                                                            double nearestTilt1_M1,
                                                            double nearestTilt2_M1) {
    if (z11_M1 == -1 || z12_M1 == -1 || z21_M1 == -1 || z22_M1 == -1) {
        return triangularInterpolationMotor1(tiltAngle, tipAngle, z11_M1, z12_M1, z21_M1, z22_M1, z33_M1,
                                              nearestTip1_M1, nearestTip2_M1, nearestTip3_M1,
                                              nearestTilt1_M1, nearestTilt2_M1, nearestTilt3_M1);
    }

    double biLinearInterpolation_M1 = 0.0;
    double x2x1 = nearestTip2_M1 - nearestTip1_M1;     //x is tip y is tilt :)
    double y2y1 = nearestTilt2_M1 - nearestTilt1_M1;
    double x2x = nearestTip2_M1 - tipAngle;
    double y2y = nearestTilt2_M1 - tiltAngle;
    double yy1 = tiltAngle - nearestTilt1_M1;
    double xx1 = tipAngle - nearestTip1_M1;
    biLinearInterpolation_M1 =  (1.0 / (x2x1 * y2y1)) * (z11_M1 * x2x * y2y
                                                    + z21_M1 * xx1 * y2y
                                                    + z12_M1 * x2x * yy1
                                                    + z22_M1 * xx1 * yy1);
    double motorAngle1 = biLinearInterpolation_M1; 
    return motorAngle1; 
}

double SepGimbalLookUpTable::bilinearInterpolationMotor2(double tiltAngle,
                                                            double tipAngle,
                                                            double z11_M2,
                                                            double z12_M2,
                                                            double z21_M2,
                                                            double z22_M2,
                                                            double nearestTip1_M2,
                                                            double nearestTip2_M2,
                                                            double nearestTilt1_M2,
                                                            double nearestTilt2_M2) {
    if (z11_M2 == -1 || z12_M2 == -1 || z21_M2 == -1 || z22_M2 == -1) {
        return triangularInterpolationMotor2(tiltAngle, tipAngle, z11_M2, z12_M2, z21_M2, z22_M2, z33_M2,
                                              nearestTip1_M2, nearestTip2_M2, nearestTip3_M2,
                                              nearestTilt1_M2, nearestTilt2_M2, nearestTilt3_M2);
    }
    
    double biLinearInterpolation_M2 = 0.0;
    double x2x1 = nearestTip2_M2 - nearestTip1_M2;     //x is tip y is tilt :)
    double y2y1 = nearestTilt2_M2 - nearestTilt1_M2;
    double x2x = nearestTip2_M2 - tipAngle;
    double y2y = nearestTilt2_M2 - tiltAngle;
    double yy1 = tiltAngle - nearestTilt1_M2;
    double xx1 = tipAngle - nearestTip1_M2;
    biLinearInterpolation_M2 =  (1.0 / (x2x1 * y2y1)) * (z11_M2 * x2x * y2y
                                                    + z21_M2 * xx1 * y2y
                                                    + z12_M2 * x2x * yy1
                                                    + z22_M2 * xx1 * yy1);
    double motorAngle2 = biLinearInterpolation_M2; 
    return motorAngle2; 
}

double SepGimbalLookUpTable::triangularInterpolationMotor1(double tiltAngle,
                                                            double tipAngle,
                                                            double z11_M1,
                                                            double z12_M1,
                                                            double z21_M1,
                                                            double z22_M1,
                                                            double z33_M1,
                                                            double nearestTip1_M1,
                                                            double nearestTip2_M1,
                                                            double nearestTip3_M1,
                                                            double nearestTilt1_M1,
                                                            double nearestTilt2_M1,
                                                            double nearestTilt3_M1) {
    double denominator = (nearestTip2_M1 - nearestTip3_M1) * (nearestTilt1_M1 - nearestTilt3_M1 ) + (nearestTilt3_M1 - nearestTilt2_M1 ) * (nearestTip1_M1 - nearestTip3_M1);
    double barycentric1 = ((nearestTip2_M1 - nearestTip3_M1) * (tiltAngle - nearestTilt3_M1 ) + (nearestTilt3_M1 - nearestTilt2_M1 ) * (tipAngle - nearestTip3_M1)) / denominator;
    double barycentric2 = ((nearestTip3_M1 - nearestTip1_M1) * (tiltAngle - nearestTilt3_M1) + (nearestTilt1_M1 - nearestTilt3_M1) * (tipAngle- nearestTip3_M1)) / denominator;
    double barycentric3 = 1 - barycentric1- barycentric2;
    std::cout << "denominator M1 = " << denominator << std::endl;
    std::cout << "barycentric1 M1= " << barycentric1 << std::endl;
    std::cout << "barycentric2 M1= " << barycentric2 << std::endl;
    std::cout << "barycentric3 M1= " << barycentric3 << std::endl;

    // Interpolate using barycentric coordinates
    double interpolated_motor1_angle = barycentric1 * z11_M1 + barycentric2 * z22_M1 + barycentric3 * z33_M1;
    return interpolated_motor1_angle;
}

double SepGimbalLookUpTable::triangularInterpolationMotor2(double tiltAngle,
                                                            double tipAngle,
                                                            double z11_M2,
                                                            double z12_M2,
                                                            double z21_M2,
                                                            double z22_M2,
                                                            double z33_M2,
                                                            double nearestTip1_M2,
                                                            double nearestTip2_M2,
                                                            double nearestTip3_M2,
                                                            double nearestTilt1_M2,
                                                            double nearestTilt2_M2,
                                                            double nearestTilt3_M2) {
    double denominator = (nearestTip2_M2 - nearestTip3_M2) * (nearestTilt1_M2 - nearestTilt3_M2 ) + (nearestTilt3_M2 - nearestTilt2_M2 ) * (nearestTip1_M2 - nearestTip3_M2);
    double barycentric1 = ((nearestTip2_M2 - nearestTip3_M2) * (tiltAngle - nearestTilt3_M2 ) + (nearestTilt3_M2 - nearestTilt2_M2 ) * (tipAngle - nearestTip3_M2)) / denominator;
    double barycentric2 = ((nearestTip3_M2 - nearestTip1_M2) * (tiltAngle - nearestTilt3_M2) + (nearestTilt1_M2 - nearestTilt3_M2) * (tipAngle- nearestTip3_M2)) / denominator;
    double barycentric3 = 1 - barycentric1- barycentric2;
    std::cout << "denominator M2= " << denominator << std::endl;
    std::cout << "barycentric1 M2= " << barycentric1 << std::endl;
    std::cout << "barycentric2 M2= " << barycentric2 << std::endl;
    std::cout << "barycentric3 M2= " << barycentric3 << std::endl;


    // Interpolate using barycentric coordinates
    double interpolated_motor2_angle = barycentric1 * z11_M2 + barycentric2 * z22_M2 + barycentric3 * z33_M2;
    return interpolated_motor2_angle;
    std::cout << "interpolated_motor2_angle = " << interpolated_motor2_angle << std::endl;

}

// void SepGimbalLookUpTable:: writeOutputMotorMessages(double motorAngle1) {
//     // Assuming motor1AngleOut is a struct with a member theta
//     motor1AngleOut.theta = motorAngle1; 
//     HingedRigidBodyMsg_C_write(&motor1AngleOut, this->motor1AngleOutMsg);
// }
