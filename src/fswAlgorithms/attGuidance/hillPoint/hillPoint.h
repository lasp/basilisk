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

#ifndef _HILL_POINT_
#define _HILL_POINT_

#include <stdint.h>

/* Required module input messages */
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"

#include "architecture/utilities/bskLogging.h"




/*!@brief Data structure for module to compute the Hill-frame pointing navigation solution.
 */
class HillPoint : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;
    
    /* declare module IO interfaces */
    Message<AttRefMsgPayload> attRefOutMsg;               //!<        The name of the output message
    ReadFunctor<NavTransMsgPayload> transNavInMsg;            //!<        The name of the incoming attitude command
    ReadFunctor<EphemerisMsgPayload> celBodyInMsg;            //!<        The name of the celestial body message

    int planetMsgIsLinked;                  //!<        flag if the planet message is linked

    BSKLogger bskLogger={};                             //!< BSK Logging

};

#endif
