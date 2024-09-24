/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#ifndef SCANNINGINSTRUMENTCONTROLLER_H
#define SCANNINGINSTRUMENTCONTROLLER_H

#include <stdint.h>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AccessMsgPayload.h"
#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/DeviceStatusMsgPayload.h"
#include "architecture/msgPayloadDefC/DeviceCmdMsgPayload.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief Module to perform continuous instrument control
 */
class ScanningInstrumentController : public SysModel {
public:
    void UpdateState(uint64_t callTime) override;
    void Reset(uint64_t callTime) override;

    double attErrTolerance; //!< Normalized MRP attitude error tolerance
    unsigned int useRateTolerance; //!< Flag to enable rate error tolerance
    double rateErrTolerance; //!< Rate error tolerance in rad/s
    unsigned int controllerStatus;  //!< dictates whether or not the controller should be running

    /* declare module IO interfaces */
    ReadFunctor<AccessMsgPayload> accessInMsg;  //!< Ground location access
    ReadFunctor<AttGuidMsgPayload> attGuidInMsg;  //!< Attitude guidance input message
    ReadFunctor<DeviceStatusMsgPayload> deviceStatusInMsg;  //!< Device status input message
    Message<DeviceCmdMsgPayload> deviceCmdOutMsg;  //!< Device status output message

    BSKLogger bskLogger={};  //!< BSK Logging
};

#endif
