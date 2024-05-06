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

#ifndef _TWOAXISGIMBAL_
#define _TWOAXISGIMBAL_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "cMsgCInterface/HingedRigidBodyMsg_C.h"
#include "cMsgCInterface/MotorStepCommandMsg_C.h"
#include "cMsgCInterface/StepperMotorMsg_C.h"
#include "cMsgCInterface/TwoAxisGimbalMsg_C.h"
#include "cMsgCInterface/PrescribedRotationMsg_C.h"

/*! @brief Two Axis Gimbal Class */
class TwoAxisGimbal: public SysModel {
public:
    TwoAxisGimbal() = default;                                                      //!< Constructor
    ~TwoAxisGimbal() = default;                                                     //!< Destructor

    void Reset(uint64_t CurrentSimNanos) override;                                  //!< Reset member function
    void UpdateState(uint64_t CurrentSimNanos) override;                            //!< Update member function

    ReadFunctor<HingedRigidBodyMsgPayload> motor1InitStateInMsg;                    //!< Input msg for stepper motor 1 initial state
    ReadFunctor<HingedRigidBodyMsgPayload> motor2InitStateInMsg;                    //!< Input msg for stepper motor 2 initial state
    ReadFunctor<MotorStepCommandMsgPayload> motor1StepCmdInMsg;                     //!< Input msg for stepper motor 1 steps commanded
    ReadFunctor<MotorStepCommandMsgPayload> motor2StepCmdInMsg;                     //!< Input msg for stepper motor 2 steps commanded
    ReadFunctor<StepperMotorMsgPayload> motor1StateInMsg;                           //!< Input msg for stepper motor 1 state information
    ReadFunctor<StepperMotorMsgPayload> motor2StateInMsg;                           //!< Input msg for stepper motor 2 state information
    Message<TwoAxisGimbalMsgPayload> twoAxisGimbalOutMsg;                           //!< Output msg for the gimbal tip and tilt angles
    Message<PrescribedRotationMsgPayload> prescribedRotationOutMsg;                 //!< Output msg for the hub-relative prescribed rotational states

    BSKLogger *bskLogger;                                                           //!< BSK Logging

private:
};

#endif
