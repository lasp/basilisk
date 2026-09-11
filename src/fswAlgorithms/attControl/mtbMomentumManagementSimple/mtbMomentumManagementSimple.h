// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef MTBMOMENTUMMANAGEMENTSIMPLE_H
#define MTBMOMENTUMMANAGEMENTSIMPLE_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>

#include <mission/parameters.h>
#include <stdint.h>

/*! @brief Top level structure for the sub-module routines. */
class MtbMomentumManagementSimple : public SysModel {
public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* Configs.*/
    double Kp;  //!<[1/s]  momentum feedback gain

    /* Inputs.*/
    ReadFunctor<RWArrayConfigMsgPayload> rwParamsInMsg;  //!< input message containing RW parameters
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;        //!< input message containingRW speeds

    /* Outputs.*/
    Message<CmdTorqueBodyMsgPayload>
        tauMtbRequestOutMsg;  //!< output message containing control torque in the Body frame

    /* Other. */
    RWArrayConfigMsgPayload rwConfigParams;  //!< configuration for RW's
    double Gs[3 * RW_EFF_CNT];               //!< transformation from the wheelspace to the Body frame
    BSKLogger bskLogger = {};                //!< BSK Logging
};

#endif
