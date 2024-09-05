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

#ifndef _ATT_TRACKING_ERROR_
#define _ATT_TRACKING_ERROR_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"

#include <stdint.h>
#include "architecture/utilities/bskLogging.h"



/*!@brief Data structure for module to compute the attitude tracking error between the spacecraft attitude and the reference.
 */
class AttTrackingError : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;

    double sigma_R0R[3];                        //!< MRP from corrected reference frame to original reference frame R0. This is the same as [BcB] going from primary body frame B to the corrected body frame Bc
    Message<AttGuidMsgPayload> attGuidOutMsg;              //!< output msg of attitude guidance
    ReadFunctor<NavAttMsgPayload> attNavInMsg;                 //!< input msg measured attitude
    ReadFunctor<AttRefMsgPayload> attRefInMsg;                 //!< input msg of reference attitude
    BSKLogger bskLogger={};                       //!< BSK Logging
};

#endif
