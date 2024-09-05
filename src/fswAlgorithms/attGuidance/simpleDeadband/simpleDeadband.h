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

#ifndef _SIMPLE_DEADBAND_
#define _SIMPLE_DEADBAND_

#include <stdint.h>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/utilities/bskLogging.h"



/*! @brief Top level structure for the sub-module routines. */
class SimpleDeadband : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;
    void applyDBLogic_simpleDeadband();

    /* declare module private variables */
    double innerAttThresh;              /*!< inner limit for sigma (attitude) errors */
    double outerAttThresh;              /*!< outer limit for sigma (attitude) errors */
    double innerRateThresh;             /*!< inner limit for omega (rate) errors */
    double outerRateThresh;             /*!< outer limit for omega (rate) errors */
    uint32_t wasControlOff;             /*!< boolean variable to keep track of the last Control status (ON/OFF) */
    double attError;                    /*!< current scalar attitude error */
    double rateError;                   /*!< current scalar rate error */
    
    /* declare module IO interfaces */
    Message<AttGuidMsgPayload> attGuidOutMsg;    /*!< The name of the output message*/
    ReadFunctor<AttGuidMsgPayload> guidInMsg;        /*!< The name of the guidance reference Input message */

    AttGuidMsgPayload attGuidOut;                       /*!< copy of the output message */
    BSKLogger bskLogger={};                             //!< BSK Logging

};

#endif
