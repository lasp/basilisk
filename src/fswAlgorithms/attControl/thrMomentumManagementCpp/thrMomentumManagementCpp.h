/*
 ISC License

 Copyright (c) 2024, Laboratory of Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef _THR_MOMENTUM_MANAGEMENT_CPP_H_
#define _THR_MOMENTUM_MANAGEMENT_CPP_H_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

#include "architecture/msgPayloadDefC/RWArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"

#include "Eigen/Core"

/*! @brief Module configuration message definition. */
class ThrMomentumManagementCpp: public SysModel {
public:
    void Reset(uint64_t currentSimNanos) override;              //!< Reset function
    void UpdateState(uint64_t currentSimNanos) override;        //!< Update function

    double hs_min = 0;                                      //!< [Nms]  minimum RW cluster momentum for dumping
    Eigen::Vector3d hd_B;
    RWArrayConfigMsgPayload rwConfigParams;             //!< [-] struct to store message containing RW config parameters in body B frame

    Message<CmdTorqueBodyMsgPayload> deltaHOutMsg = {};                    //!< The name of the output message
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;                         //!< [] The name for the reaction wheel speeds message
    ReadFunctor<RWArrayConfigMsgPayload> rwConfigDataInMsg;               //!< [-] The name of the RWA configuration message

    BSKLogger *bskLogger;                             //!< BSK Logging
private:
    int initRequest = 1;                                    //!< [-] status flag of the momentum dumping management
};

#endif
