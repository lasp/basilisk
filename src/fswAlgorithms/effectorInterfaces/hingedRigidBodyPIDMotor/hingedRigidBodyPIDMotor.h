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

#ifndef _HINGED_RIGID_BODY_PID_MOTOR_
#define _HINGED_RIGID_BODY_PID_MOTOR_

#include <stdint.h>
#include "architecture/utilities/bskLogging.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "msgPayloadDef/HingedRigidBodyMsgPayload.h"
#include "msgPayloadDef/ArrayMotorTorqueMsgPayload.h"

/*! @brief Top level structure for the sub-module routines. */
class HingedRigidBodyPIDMotor : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;

    /*! declare these user-defined input parameters */
    double K;                 //!< proportional gain
    double P;                 //!< derivative gain
    double I;                 //!< integral gain

    /*! declare these variables for internal computations */
    uint64_t priorTime;       //!< prior function call time for trapezoid integration
    double   priorThetaError; //!< theta error at prior function call
    double   intError;        //!< integral error

    /* declare module IO interfaces */
    ReadFunctor<HingedRigidBodyMsgPayload>   hingedRigidBodyInMsg;      //!< input spinning body message
    ReadFunctor<HingedRigidBodyMsgPayload>   hingedRigidBodyRefInMsg;   //!< output msg containing spinning body target angle and angle rate
    Message<ArrayMotorTorqueMsgPayload>  motorTorqueOutMsg;         //!< output msg containing the motor torque to the array drive

    BSKLogger bskLogger={};                             //!< BSK Logging

};

#endif
