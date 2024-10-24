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


#ifndef DIPOLEMAPPING_H
#define DIPOLEMAPPING_H

#include <stdint.h>
#include "architecture/utilities/bskLogging.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/DipoleRequestBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/MTBCmdMsgPayload.h"
#include "architecture/msgPayloadDefC/MTBArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/TAMSensorBodyMsgPayload.h"

/*! @brief Top level structure for the sub-module routines. */
class DipoleMapping : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;

    /* Configs.*/
    double steeringMatrix[MAX_EFF_CNT * 3];             //!< matrix for mapping body frame dipole request to individual torque bar dipoles

    /* Inputs. */
    ReadFunctor<MTBArrayConfigMsgPayload> mtbArrayConfigParamsInMsg;      //!< input message containing configuration parameters for all the torque bars on the vehicle
    ReadFunctor<DipoleRequestBodyMsgPayload> dipoleRequestBodyInMsg;      //!< [A-m2] input message containing the requested body frame dipole

    /* Outputs. */
    Message<MTBCmdMsgPayload> dipoleRequestMtbOutMsg;                 //!< [A-m2] output message containing the individual dipole requests for each torque bar on the vehicle

    /* Other. */
    MTBArrayConfigMsgPayload mtbArrayConfigParams;      //!< configuration parameters for all the torque bars used on the vehicle
    BSKLogger bskLogger={};                               //!< BSK Logging
};

#endif
