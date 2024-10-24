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

#ifndef _DV_ATT_EFFECT_H_
#define _DV_ATT_EFFECT_H_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayOnTimeCmdMsgPayload.h"

#include "fswAlgorithms/effectorInterfaces/_GeneralModuleFiles/thrustGroupData.h"
#include "architecture/utilities/bskLogging.h"
#include <stdint.h>
#include <stdlib.h>




#define MAX_NUM_THR_GROUPS 4


/*! @brief effective thruster pair structure */
typedef struct {
    double onTime;              /*!< s   The requested on time for this thruster*/
    uint32_t thrustIndex;       /*!< -  The actual thruster index associated with on-time*/
}effPairs;

void computeSingleThrustBlock(ThrustGroupData *thrData,
                              uint64_t callTime,
                              CmdTorqueBodyMsgPayload *contrReq,
                              int64_t moduleID);

/*! @brief module configuration message */
class DvAttEffect : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;
    ReadFunctor<CmdTorqueBodyMsgPayload> cmdTorqueBodyInMsg; /*!< - The name of the Input message*/

    uint32_t numThrGroups;   /*!< - Count on the number of thrusters groups available*/
    ThrustGroupData thrGroups[MAX_NUM_THR_GROUPS]; /*!< - Thruster grouping container*/
    BSKLogger bskLogger={};   //!< BSK Logging
};


#endif
