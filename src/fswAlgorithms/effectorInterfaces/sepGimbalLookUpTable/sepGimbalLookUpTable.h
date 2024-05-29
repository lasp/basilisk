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
#define MAX_COLUMNS 78


/*! @brief Top level structure for the sub-module routines. */

typedef struct {
    double rowNum;
    double columnNum;

    double desiredTipAngle;                                     //!< [deg] Desired platform tip angle
    double desiredTiltAngle;                                    //!< [deg] Desired platform tilt angle
    double motor1Angle;                                         //!< [deg] Desired motor angle 1
    double motor2Angle;                                         //!< [deg] Desired motor angle 2

}LookUpTableRowElements;

class SepGimbalLookUpTable : public SysModel {
public:
    char* fileName_1;                                           //!< CSV File for motor 1
    char* fileName_2;                                           //!< CSV File for motor 2

    LookUpTableRowElements* rows[MAX_ROWS];                     //!< Array to save csv file rows
    LookUpTableRowElements* selectedTiltRows[MAX_ROWS];         //!< Array to save rows with desired tilt angle
    LookUpTableRowElements* columns[MAX_COLUMNS];               //!< Array to save csv file rows
    LookUpTableRowElements* selectedTipColumns[MAX_COLUMNS];    //!< Array to save rows with desired tip angle

    BSKLogger* bskLogger;                                        //!< BSK Logging

    /* Messages */
    HingedRigidBodyMsg_C desiredGimbalTipAngleInMsg;             //!< Intput msg for the tip angle (theta)
    HingedRigidBodyMsg_C desiredGimbalTiltAngleInMsg;            //!< Intput msg for the tilt angle (theta)
    HingedRigidBodyMsg_C motor1AngleOutMsg;                      //!< Output msg for motor 1 angle (theta)
    HingedRigidBodyMsg_C motor2AngleOutMsg;                      //!< Output msg for motor 2 angle (theta)

    void SelfInit() override;                     //!< Method for module initialization
    void Reset(uint64_t callTime) override;     //!< Method for module reset
    void UpdateState(uint64_t callTime) override;    //!< Method for module time update

    // Function to read data from a CSV file
    static int readData1(const char *filename_1, LookUpTableRowElements* rows[], LookUpTableRowElements *columns[]);
    static int readData2(const char *filename_2, LookUpTableRowElements* rows[], LookUpTableRowElements *columns[]);

private:
    double motor1Angle{};
    double motor2Angle{};

    LookUpTableRowElements findMatchingTiltAngle(double desiredTiltAngle);
    LookUpTableRowElements findMatchingTipAngle(double desiredTipAngle);
    Eigen::Vector2d bilinearInterpolation(
                              double tipAngle,
                              double tiltAngle,
                              double z11,
                              double z12,
                              double z21,
                              double z22,
                              double x1,
                              double x2,
                              double y1,
                              double y2);
};

#endif /* SEPGIMBALLOOKUPTABLE_ */
