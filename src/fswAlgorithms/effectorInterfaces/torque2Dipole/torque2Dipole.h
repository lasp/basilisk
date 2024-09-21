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


#ifndef TORQUE2DIPOLE_H
#define TORQUE2DIPOLE_H

#include <stdint.h>
#include "architecture/utilities/bskLogging.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "msgPayloadDef/TAMSensorBodyMsgPayload.h"
#include "msgPayloadDef/DipoleRequestBodyMsgPayload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgPayload.h"

/*! @brief Top level structure for the sub-module routines. */
class Torque2Dipole : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;

    /* Inputs.*/
    ReadFunctor<TAMSensorBodyMsgPayload> tamSensorBodyInMsg;          //!< [Tesla] input message for magnetic field sensor data in the Body frame
    ReadFunctor<CmdTorqueBodyMsgPayload> tauRequestInMsg;             //!< [N-m] input message containing control torque in the Body frame

    /* Outputs.*/
    Message<DipoleRequestBodyMsgPayload> dipoleRequestOutMsg;     //!< [A-m2] output message containing dipole request in the Body frame

    /* Other. */
    BSKLogger bskLogger={};                           //!< BSK Logging
};

#endif
