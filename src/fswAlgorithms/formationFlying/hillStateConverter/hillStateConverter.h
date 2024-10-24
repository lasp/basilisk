/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _HILL_STATE_CONVERTER_H_
#define _HILL_STATE_CONVERTER_H_

//  Standard lib imports
#include <stdint.h>

//  Support imports
#include "architecture/utilities/bskLogging.h"

//  Message type imports
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/HillRelStateMsgPayload.h"



/*! @brief Top level structure for the sub-module routines. */
class HillStateConverter : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;

    /* declare module IO interfaces */
    Message<HillRelStateMsgPayload> hillStateOutMsg; //!< Output message containing relative state of deputy to chief in chief hill coordinates
    ReadFunctor<NavTransMsgPayload> chiefStateInMsg; //!< Input message containing chief inertial translational state estimate
    ReadFunctor<NavTransMsgPayload> depStateInMsg; //!< Input message containing deputy inertial translational state estimate

    BSKLogger bskLogger={};                           //!< BSK Logging
};

#endif
