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

#ifndef _DV_EXECUTE_GUIDANCE_H_
#define _DV_EXECUTE_GUIDANCE_H_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayOnTimeCmdMsgPayload.h"
#include "architecture/msgPayloadDefC/DvBurnCmdMsgPayload.h"
#include "architecture/msgPayloadDefC/DvExecutionDataMsgPayload.h"

#include "architecture/utilities/bskLogging.h"
#include <stdint.h>



/*! @brief Top level structure for the execution of a Delta-V maneuver */
class DvExecuteGuidance : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;
    ReadFunctor<NavTransMsgPayload> navDataInMsg; /*!< [-] navigation input message that includes dv accumulation info */
    ReadFunctor<DvBurnCmdMsgPayload> burnDataInMsg;/*!< [-] commanded burn input message */
    Message<THRArrayOnTimeCmdMsgPayload> thrCmdOutMsg; /*!< [-] thruster command on time output message */
    Message<DvExecutionDataMsgPayload> burnExecOutMsg; /*!< [-] burn execution output message */
    double dvInit[3];        /*!< [m/s] DV reading off the accelerometers at burn start*/
    uint32_t burnExecuting;  /*!< [-] Flag indicating whether the burn is in progress or not*/
    uint32_t burnComplete;   /*!< [-] Flag indicating that burn has completed successfully*/
    double burnTime;          /*!< [s] Burn time to be used for telemetry*/
    uint64_t prevCallTime;   /*!< [-] Call time register for computing total burn time*/
    double minTime;           /*!< [s] Minimum count of burn time allowed to elapse*/
    double maxTime;           /*!< [s] Maximum count of burn time allowed to elapse*/
    double defaultControlPeriod; /*!< [s] Default control period used for first call*/

    BSKLogger bskLogger={};   //!< BSK Logging
};

#endif
