
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

#ifndef SEPGIMBALLOOKUPTABLE_
#define SEPGIMBALLOOKUPTABLE_

#include <stdint.h>
#include <Eigen/Core>
#include "architecture/utilities/bskLogging.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "cMsgCInterface/HingedRigidBodyMsg_C.h"

#define MAX_ROWS 114
#define MAX_COLUMNS 82

/*! @brief Top level structure for the sub-module routines. */

class SepGimbalLookUpTable : public SysModel {
public:
    //Eigen::Matrix<double, 114 , 82> motorA1;

    BSKLogger* bskLogger;                                        //!< BSK Logging

    /* Messages */
    HingedRigidBodyMsg_C desiredGimbalTipAngleInMsg;             //!< Intput msg for the tip angle (theta)
    HingedRigidBodyMsg_C desiredGimbalTiltAngleInMsg;            //!< Intput msg for the tilt angle (theta)
    HingedRigidBodyMsg_C motor1AngleOutMsg;                      //!< Output msg for motor 1 angle (theta)
    HingedRigidBodyMsg_C motor2AngleOutMsg;                      //!< Output msg for motor 2 angle (theta)

    void SelfInit() override;                     //!< Method for module initialization
    void Reset(uint64_t callTime) override;     //!< Method for module reset
    void UpdateState(uint64_t callTime) override;    //!< Method for module time update

private:
    //Input Messgaes:
    double tiltAngle;
    double tipAngle;

    //Output Messgaes:
    double motor1Angle;
    double motor2Angle; 

    // Variables of Motor 1
    double z11_M1;
    double z12_M1;
    double z21_M1;
    double z22_M1;
    double z33_M1;

    double nearestTip1_M1;
    double nearestTip2_M1;
    double nearestTip3_M1;
    int nearestTip1_Index_M1;
    int nearestTip2_Index_M1;
    int nearestTip3_Index_M1;

    double nearestTilt1_M1;
    double nearestTilt2_M1;
    double nearestTilt3_M1;
    int nearestTilt1_Index_M1;
    int nearestTilt2_Index_M1;
    int nearestTilt3_Index_M1;

    // Variables of Motor 2
    double tiltAngle_M2;
    double tipAngle_M2;   
    double z11_M2;
    double z12_M2;
    double z21_M2;
    double z22_M2;
    double z33_M2;

    double nearestTip1_M2;
    double nearestTip2_M2;
    double nearestTip3_M2;
    int nearestTip1_Index_M2;
    int nearestTip2_Index_M2;
    int nearestTip3_Index_M2;

    double nearestTilt1_M2;
    double nearestTilt2_M2;
    double nearestTilt3_M2;
    int nearestTilt1_Index_M2;
    int nearestTilt2_Index_M2;
    int nearestTilt3_Index_M2;
    
    void readMessage();
    void readGimbalAnglesInputMsg();
    // void SepGimbalLookUpTable:: writeOutputMotorMessages(double motorAngle1);

    //Functions of Motor 1
    Eigen::Matrix<double, 114, 82> motorA1;
    void motor1Table(Eigen::Matrix<double, 114, 82>& motorA1); 
    void findMatchingTiltAngleAndIndeciesOfMotor1(double tiltAngle, double tipAngle, Eigen::Matrix<double, 114, 82>& motorA1);
    void findMatchingTipAngleAndIndeciesOfMotor1(double tiltAngle, double tipAngle, Eigen::Matrix<double, 114, 82>& motorA1);
    void findMotor1Angles(int nearestTilt1_Index_M1, int nearestTilt2_Index_M1, int nearestTip1_Index_M1, int nearestTip2_Index_M1,  Eigen::Matrix<double, 114, 82>& motorA1);
    double bilinearInterpolationMotor1(double tiltAngle, double tipAngle, double z11_M1, double z12_M1, double z21_M1, double z22_M1, double nearestTip1_M1, double nearestTip2_M1, double nearestTilt1_M1, double nearestTilt2_M1);
    double triangularInterpolationMotor1(double tiltAngle,double tipAngle,double z11_M1,double z12_M1,double z21_M1,double z22_M1,double z33_M1,double nearestTip1_M1,double nearestTip2_M1,double nearestTip3_M1,double nearestTilt1_M1,double nearestTilt2_M1,double nearestTilt3_M1);

    //Functions of Motor 2
    Eigen::Matrix<double, 114, 82> motorA2;
    void motor2Table(Eigen::Matrix<double, 114, 82>& motorA2);
    void findMatchingTiltAngleAndIndeciesOfMotor2(double tiltAngle, double tipAngle, Eigen::Matrix<double, 114, 82>& motorA2);
    void findMatchingTipAngleAndIndeciesOfMotor2(double tiltAngle, double tipAngle, Eigen::Matrix<double, 114, 82>& motorA2);
    void findMotor2Angles(int nearestTilt1_Index_M2, int nearestTilt2_Index_M2, int nearestTip1_Index_M2, int nearestTip2_Index_M2,  Eigen::Matrix<double, 114, 82>& motorA2);
    double bilinearInterpolationMotor2(double tiltAngle, double tipAngle, double z11_M2, double z12_M2, double z21_M2, double z22_M2, double nearestTip1_M2, double nearestTip2_M2, double nearestTilt1_M2, double nearestTilt2_M2);
    double triangularInterpolationMotor2(double tiltAngle,double tipAngle,double z11_M2,double z12_M2,double z21_M2,double z22_M2,double z33_M2,double nearestTip1_M2,double nearestTip2_M2,double nearestTip3_M2,double nearestTilt1_M2,double nearestTilt2_M2,double nearestTilt3_M2);
};

#endif /* SEPGIMBALLOOKUPTABLE_ */
