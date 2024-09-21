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

#ifndef _EPHEM_DIFFERENCE_H_
#define _EPHEM_DIFFERENCE_H_

#define MAX_NUM_CHANGE_BODIES 10

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "msgPayloadDef/EphemerisMsgPayload.h"

#include "architecture/utilities/bskLogging.h"


/*! @brief Container with paired input/output message names and IDs */
typedef struct{
    ReadFunctor<EphemerisMsgPayload> ephInMsg;  //!< [-] Input name for the ephemeris message
    Message<EphemerisMsgPayload> ephOutMsg; //!< [-] The name converted output message
}EphemChangeConfig;

/*! @brief Container holding ephemDifference module variables */
class EphemDifference : public SysModel {
public:
    void UpdateState(uint64_t callTime) override;
    void Reset(uint64_t callTime) override;

    ReadFunctor<EphemerisMsgPayload> ephBaseInMsg; //!< base ephemeris input message name
    EphemChangeConfig changeBodies[MAX_NUM_CHANGE_BODIES]; //!< [-] The list of bodies to change out

    uint32_t ephBdyCount; //!< [-] The number of ephemeris bodies we are changing

    BSKLogger bskLogger{}; //!< BSK Logging
};

#endif
