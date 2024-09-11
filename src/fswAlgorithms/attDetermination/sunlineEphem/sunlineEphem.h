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

#ifndef _SUNLINE_EPHEM_FSW_MSG_H_
#define _SUNLINE_EPHEM_FSW_MSG_H_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"

#include "architecture/utilities/bskLogging.h"
#include <stdint.h>



/*! @brief Top level structure for the sub-module routines. */
class SunlineEphem : public SysModel {
public:
    void UpdateState(uint64_t callTime) override;

    /* declare module IO interfaces */
    Message<NavAttMsgPayload> navStateOutMsg;                     /*!< The name of the output message*/
    ReadFunctor<EphemerisMsgPayload> sunPositionInMsg;           //!< The name of the sun ephemeris input message
    ReadFunctor<NavTransMsgPayload> scPositionInMsg;             //!< The name of the spacecraft ephemeris input message
    ReadFunctor<NavAttMsgPayload> scAttitudeInMsg;               //!< The name of the spacecraft attitude input message

    BSKLogger bskLogger={}; //!< BSK Logging

};

#endif
