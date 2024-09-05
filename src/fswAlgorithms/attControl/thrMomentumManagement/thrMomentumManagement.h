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

#ifndef _THR_MOMENTUM_MANAGEMENT_H_
#define _THR_MOMENTUM_MANAGEMENT_H_

#include <stdint.h>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/RWArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"

/*! @brief Module configuration message definition. */
class ThrMomentumManagement : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;

    /* declare module private variables */
    int initRequest;                                    //!< [-] status flag of the momentum dumping management
    RWArrayConfigMsgPayload rwConfigParams;             //!< [-] struct to store message containing RW config parameters in body B frame

    /* declare module public variables */
    double hs_min;                                      //!< [Nms]  minimum RW cluster momentum for dumping

    /* declare module IO interfaces */
    Message<CmdTorqueBodyMsgPayload> deltaHOutMsg;                    //!< The name of the output message
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;                         //!< [] The name for the reaction wheel speeds message
    ReadFunctor<RWArrayConfigMsgPayload> rwConfigDataInMsg;               //!< [-] The name of the RWA configuration message

    BSKLogger bskLogger = {};                             //!< BSK Logging

};

#endif
