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

#ifndef _SEPGIMBAL_
#define _SEPGIMBAL_

#include <stdint.h>
#include <stdbool.h>
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/BodyHeadingMsg_C.h"
#include "cMsgCInterface/HingedRigidBodyMsg_C.h"

/*! @brief Top level structure for the sub-module routines. */
typedef struct {

    /* Private variables */
    double desiredThrustUnitVector[3];                             //!< [deg] Desired thrust unit vector
    double Tip_angle;                                              //!< [deg] Desired tip angle
    double Tilt_angle;                                             //!< [deg] Desired tilt angle


    bool firstCall;

    double previousWrittenTime;                                //!< [ns] Time the desired theta was given

    // uint64_t previousWrittenTime;                               //!< [ns] Time the desired theta was given (might not need it)
    BSKLogger* bskLogger;                                          //!< BSK Logging

    /* Messages */
    BodyHeadingMsg_C desiredThrustMountFrameInMsg;                 //!< Intput msg for the desired thrust unit vector in the mount frame
    HingedRigidBodyMsg_C desiredGimbalTipAngleOutMsg;              //!< Output msg for the tip angle (theta)
    HingedRigidBodyMsg_C desiredGimbalTiltAngleOutMsg;             //!< Output msg for the tilt angle (theta)

}SepGimbalConfig;

#ifdef __cplusplus
extern "C" {
#endif
    void SelfInit_sepGimbal(SepGimbalConfig *configData, int64_t moduleID);                     //!< Method for module initialization
    void Reset_sepGimbal(SepGimbalConfig *configData, uint64_t callTime, int64_t moduleID);     //!< Method for module reset
    void Update_sepGimbal(SepGimbalConfig *configData, uint64_t callTime, int64_t moduleID);    //!< Method for module time update
#ifdef __cplusplus
}
#endif

#endif