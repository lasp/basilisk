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

#ifndef _DV_ACCUMULATION_H_
#define _DV_ACCUMULATION_H_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/AccDataMsgPayload.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief Top level structure for the CSS sensor interface system.  Contains all parameters for the
 CSS interface*/
class DVAccumulation : public SysModel {
public:
    void UpdateState(uint64_t callTime) override;
    void Reset(uint64_t callTime) override;

    Message<NavTransMsgPayload> dvAcumOutMsg; //!< accumulated DV output message
    ReadFunctor<AccDataMsgPayload> accPktInMsg; //!< [-] input accelerometer message
    
    uint32_t msgCount;      //!< [-] The total number of messages read from inputs
    uint32_t dvInitialized; //!< [-] Flag indicating whether DV has been started completely
    uint64_t previousTime;  //!< [ns] The clock time associated with the previous run of algorithm
    double vehAccumDV_B[3];    //!< [m/s] The accumulated Delta_V in body frame components

    BSKLogger bskLogger{};   //!< BSK Logging
};

void dvAccumulation_QuickSort (AccPktDataMsgPayload *A, int start, int end);

#endif
