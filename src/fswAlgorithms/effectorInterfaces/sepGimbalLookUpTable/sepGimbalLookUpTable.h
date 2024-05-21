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

#ifndef _SEPGIMBALLOOKUPTABLE_
#define _SEPGIMBALLOOKUPTABLE_

#include <stdint.h>
#include <stdbool.h>
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/HingedRigidBodyMsg_C.h"
#define MAX_ROWS 4091

/*! @brief Top level structure for the sub-module routines. */

typedef struct {
    double rowNum;
    double desiredTipAngle;                                     //!< [deg] Desired platform tip angle
    double desiredTiltAngle;                                    //!< [deg] Desired platform tilt angle
    double motor1Angle;                                         //!< [deg] Desired motor angle 1
    double motor2Angle;                                         //!< [deg] Desired motor angle 2

}LookUpTableRowElements;

typedef struct {
    double rowNum;
    double inputTipAngle;                                       //!< [deg] Desired platform tip angle
    double inputTiltAngle;                                      //!< [deg] Desired platform tilt angle
    double motor1Angle;                                         //!< [deg] Desired motor angle 1
    double motor2Angle;                                         //!< [deg] Desired motor angle 2
    double norm;                                                //!< [m] norm distance value
    int arrayLength;
    double findMax;

    char* fileName;                                             //!< CSV File 
    LookUpTableRowElements* rows[MAX_ROWS];                     //!< Array to save csv file rows
    LookUpTableRowElements* selectedTipRows[MAX_ROWS];          //!< Array to save rows with desired tip angle 
    LookUpTableRowElements* smallestDistances[MAX_ROWS];        //!< Array to save the rows with shortest distances from desired tip and tilt angles
    LookUpTableRowElements* smallestIndex[MAX_ROWS];            //!< Array to save the index rows with shortest distances
    

    BSKLogger* bskLogger;                                        //!< BSK Logging

    /* Messages */
    HingedRigidBodyMsg_C desiredGimbalTipAngleInMsg;              //!< Intput msg for the tip angle (theta)
    HingedRigidBodyMsg_C desiredGimbalTiltAngleInMsg;             //!< Intput msg for the tilt angle (theta)
    HingedRigidBodyMsg_C motor1AngleOutMsg;                       //!< Output msg for motor 1 angle (theta)
    HingedRigidBodyMsg_C motor2AngleOutMsg;                       //!< Output msg for motor 2 angle (theta)
    
}SepGimbalLookUpTableConfig;



#ifdef __cplusplus
extern "C" {
#endif
    void SelfInit_sepGimbalLookUpTable(SepGimbalLookUpTableConfig *configData, int64_t moduleID);                     //!< Method for module initialization
    void Reset_sepGimbalLookUpTable(SepGimbalLookUpTableConfig *configData, uint64_t callTime, int64_t moduleID);     //!< Method for module reset
    void Update_sepGimbalLookUpTable(SepGimbalLookUpTableConfig *configData, uint64_t callTime, int64_t moduleID);    //!< Method for module time update

    // Function to read data from a CSV file
    int readData(const char *filename, LookUpTableRowElements* rows[]);
    int findMatchingTipAngle(LookUpTableRowElements *rows[], double inputTipAngle, LookUpTableRowElements **selectedTipRows);
    int findMatchingTiltAngle(LookUpTableRowElements *matchingRowsTip[], int numMatchingRowsTip, double inputTiltAngle, LookUpTableRowElements **selectedTiltRows);

#ifdef __cplusplus
}
#endif

#endif
