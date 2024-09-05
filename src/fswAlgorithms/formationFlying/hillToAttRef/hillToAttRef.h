/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _HILL_TO_ATT_H
#define _HILL_TO_ATT_H

#include <stdint.h>
#include <string.h>

#include "architecture/utilities/bskLogging.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/HillRelStateMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"




/*! @brief Top level structure for the sub-module routines. */
class HillToAttRef : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;
    AttRefMsgPayload relativeToInertialMRP(double relativeAtt[3], double sigma_XN[3]);

    /* declare module IO interfaces */
    ReadFunctor<HillRelStateMsgPayload> hillStateInMsg;               //!< Provides state relative to chief
    ReadFunctor<AttRefMsgPayload> attRefInMsg;                        //!< (Optional) Provides basis for relative attitude
    ReadFunctor<NavAttMsgPayload> attNavInMsg;                        //!< (Optional) Provides basis for relative attitude
    Message<AttRefMsgPayload> attRefOutMsg;                       //!< Provides the attitude reference output message.
    BSKLogger bskLogger={};                           //!< BSK Logging

    double gainMatrix[3][6]; //!< User-configured gain matrix that maps from hill states to relative attitudes.
    double relMRPMin;        //!< Minimum value for the relative MRP components; user-configurable.
    double relMRPMax;        //!< Maximum value for the relative MRP components; user-configurable.
};

#endif

