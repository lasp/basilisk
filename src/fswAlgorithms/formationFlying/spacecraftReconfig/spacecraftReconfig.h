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

#ifndef _SPACECRAFT_RECONFIG_H_
#define _SPACECRAFT_RECONFIG_H_

#include <stdint.h>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "msgPayloadDef/NavTransMsgPayload.h"
#include "msgPayloadDef/THRArrayConfigMsgPayload.h"
#include "msgPayloadDef/AttRefMsgPayload.h"
#include "msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h"
#include "msgPayloadDef/VehicleConfigMsgPayload.h"
#include "msgPayloadDef/ReconfigBurnArrayInfoMsgPayload.h"

#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/orbitalMotion.h"

/*! @brief Data structure for the MRP feedback attitude control routine. */
class SpacecraftReconfig : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;
    void UpdateManeuver(NavTransMsgPayload chiefTransMsgBuffer,
                         NavTransMsgPayload deputyTransMsgBuffer,
                         AttRefMsgPayload attRefInMsgBuffer,
                         THRArrayConfigMsgPayload thrustConfigMsgBuffer,
                         VehicleConfigMsgPayload vehicleConfigMsgBuffer,
                         AttRefMsgPayload *attRefOutMsgBuffer,
                         THRArrayOnTimeCmdMsgPayload *thrustOnMsgBuffer,
                         uint64_t callTime);

    void ScheduleDV(ClassicElements oe_c,
                    ClassicElements oe_d,
                    THRArrayConfigMsgPayload thrustConfigMsgBuffer,
                    VehicleConfigMsgPayload vehicleConfigMsgBuffer);

    /* declare module IO interfaces */
    // in
    ReadFunctor<NavTransMsgPayload> chiefTransInMsg;                      //!< chief orbit input msg
    ReadFunctor<NavTransMsgPayload> deputyTransInMsg;                     //!< deputy orbit input msg
    ReadFunctor<THRArrayConfigMsgPayload> thrustConfigInMsg;              //!< THR configuration input msg
    ReadFunctor<AttRefMsgPayload> attRefInMsg;                            //!< nominal attitude reference input msg
    ReadFunctor<VehicleConfigMsgPayload> vehicleConfigInMsg;              //!< deputy vehicle config msg

    // out
    Message<AttRefMsgPayload> attRefOutMsg;                           //!< attitude reference output msg
    Message<THRArrayOnTimeCmdMsgPayload> onTimeOutMsg;                //!< THR on-time output msg
    Message<ReconfigBurnArrayInfoMsgPayload> burnArrayInfoOutMsg;     //!< array of burn info output msg

    double mu;  //!< [m^3/s^2] gravity constant of planet being orbited
    double attControlTime; //!< [s] attitude control margin time (time necessary to change sc's attitude)
    double targetClassicOED[6]; //!< target classic orital element difference, SMA should be normalized
    double resetPeriod; //!< [s] burn scheduling reset period
    double tCurrent; //!< [s] timer
    uint64_t prevCallTime; //!< [ns]
    uint8_t thrustOnFlag; //!< thrust control
    int    attRefInIsLinked;        //!< flag if the attitude reference input message is linked
    ReconfigBurnArrayInfoMsgPayload burnArrayInfoOutMsgBuffer;    //!< msg buffer for burn array info


    BSKLogger bskLogger={};                             //!< BSK Logging
};

#endif
