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

#ifndef _SOLAR_ARRAY_REFERENCE_
#define _SOLAR_ARRAY_REFERENCE_

#include <stdint.h>
#include "architecture/utilities/bskLogging.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "msgPayloadDef/NavAttMsgPayload.h"
#include "msgPayloadDef/AttRefMsgPayload.h"
#include "msgPayloadDef/HingedRigidBodyMsgPayload.h"


enum attitudeFrame{
    referenceFrame = 0,
    bodyFrame = 1
};

/*! @brief Top level structure for the sub-module routines. */
class SolarArrayReference : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;

    /* declare these user-defined quantities */
    double a1Hat_B[3];              //!< solar array drive axis in body frame coordinates
    double a2Hat_B[3];              //!< solar array surface normal at zero rotation
    int attitudeFrame;              //!< flag = 1: compute theta reference based on current attitude instead of attitude reference

    /* declare these variables for internal computations */
    int                 count;                    //!< counter variable for finite differences
    uint64_t            priorT;                   //!< prior call time for finite differences
    double              priorThetaR;              //!< prior output msg for finite differences

    /* declare module IO interfaces */
    ReadFunctor<NavAttMsgPayload>            attNavInMsg;                  //!< input msg measured attitude
    ReadFunctor<AttRefMsgPayload>            attRefInMsg;                  //!< input attitude reference message
    ReadFunctor<HingedRigidBodyMsgPayload>   hingedRigidBodyInMsg;         //!< input hinged rigid body message
    Message<HingedRigidBodyMsgPayload>   hingedRigidBodyRefOutMsg;     //!< output msg containing hinged rigid body target angle and angle rate

    BSKLogger bskLogger={};                         //!< BSK Logging

};

#endif
