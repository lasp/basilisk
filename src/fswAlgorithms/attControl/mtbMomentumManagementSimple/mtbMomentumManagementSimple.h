/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef MTBMOMENTUMMANAGEMENTSIMPLE_H
#define MTBMOMENTUMMANAGEMENTSIMPLE_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "msgPayloadDef/RWSpeedMsgPayload.h"
#include "msgPayloadDef/RWArrayConfigMsgPayload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgPayload.h"
#include <stdio.h>
#include "architecture/utilities/macroDefinitions.h"
#include <stdint.h>

/*! @brief Top level structure for the sub-module routines. */
class MtbMomentumManagementSimple : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;

    /* Configs.*/
    double Kp;                                  //!<[1/s]  momentum feedback gain

    /* Inputs.*/
    ReadFunctor<RWArrayConfigMsgPayload> rwParamsInMsg;           //!< input message containing RW parameters
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;                 //!< input message containingRW speeds

    /* Outputs.*/
    Message<CmdTorqueBodyMsgPayload> tauMtbRequestOutMsg;     //!< output message containing control torque in the Body frame

    /* Other. */
    RWArrayConfigMsgPayload rwConfigParams;     //!< configuration for RW's
    double Gs[3 * MAX_EFF_CNT];                 //!< transformation from the wheelspace to the Body frame
    BSKLogger bskLogger = {};                   //!< BSK Logging
};

#endif
