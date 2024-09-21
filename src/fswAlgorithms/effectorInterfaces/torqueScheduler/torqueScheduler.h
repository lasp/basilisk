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

#ifndef _TORQUE_SCHEDULER_
#define _TORQUE_SCHEDULER_

#include <stdint.h>
#include "architecture/utilities/bskLogging.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "msgPayloadDef/ArrayMotorTorqueMsgPayload.h"
#include "msgPayloadDef/ArrayEffectorLockMsgPayload.h"


/*! @brief Top level structure for the sub-module routines. */
class TorqueScheduler : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;

    /* declare these user-defined inputs */
    int    lockFlag;                               //!< flag to control the scheduler logic
    double tSwitch;                                //!< [s] time span after t0 at which controller switches to second angle

    /* declare this quantity that is a module internal variable */
    uint64_t t0;                                   //!< [ns] epoch time where module is reset

    /* declare module IO interfaces */
    ReadFunctor<ArrayMotorTorqueMsgPayload>  motorTorque1InMsg;      //!< input motor torque message #1
    ReadFunctor<ArrayMotorTorqueMsgPayload>  motorTorque2InMsg;      //!< input motor torque message #1
    Message<ArrayMotorTorqueMsgPayload>  motorTorqueOutMsg;      //!< output msg containing the motor torque to the array drive
    Message<ArrayEffectorLockMsgPayload> effectorLockOutMsg;     //!< output msg containing the flag to actuate or lock the motor

    BSKLogger bskLogger={};                          //!< BSK Logging

};

#endif
