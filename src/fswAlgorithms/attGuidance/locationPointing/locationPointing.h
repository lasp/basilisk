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


#ifndef LOCATIONPOINTING_H
#define LOCATIONPOINTING_H

#include <stdint.h>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "msgPayloadDef/NavAttMsgPayload.h"
#include "msgPayloadDef/NavTransMsgPayload.h"
#include "msgPayloadDef/GroundStateMsgPayload.h"
#include "msgPayloadDef/AttGuidMsgPayload.h"
#include "msgPayloadDef/AttRefMsgPayload.h"
#include "msgPayloadDef/EphemerisMsgPayload.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief This module is used to generate the attitude reference message in order to have a spacecraft point at a location on the ground
 */
class LocationPointing : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;

    /* user configurable variables */
    double pHat_B[3];           /*!< body fixed vector that is to be aimed at a location */
    double smallAngle;          /*!< rad An angle value that specifies what is near 0 or 180 degrees */
    int useBoresightRateDamping; /*!< [int] flag to use rate damping about the sensor boresight */

    /* private variables */
    double sigma_BR_old[3];     /*!< Older sigma_BR value, stored for finite diff*/
    uint64_t time_old;          /*!< [ns] prior time value */
    double init;                /*!< moudle initialization counter */
    double eHat180_B[3];        /*!< -- Eigen axis to use if commanded axis is 180 from pHat */

    /* declare module IO interfaces */
    ReadFunctor<NavAttMsgPayload> scAttInMsg;                 //!< input msg with inertial spacecraft attitude states
    ReadFunctor<NavTransMsgPayload> scTransInMsg;             //!< input msg with inertial spacecraft position states
    ReadFunctor<GroundStateMsgPayload> locationInMsg;         //!< input msg with location relative to planet
    ReadFunctor<EphemerisMsgPayload> celBodyInMsg;            //!< input celestial body message
    ReadFunctor<NavTransMsgPayload> scTargetInMsg;            //!< input msg with inertial target spacecraft position states
    Message<AttGuidMsgPayload> attGuidOutMsg;             //!< attitude guidance output message
    Message<AttRefMsgPayload> attRefOutMsg;               //!< attitude reference output message

    BSKLogger bskLogger={};  //!< BSK Logging
};

#endif
