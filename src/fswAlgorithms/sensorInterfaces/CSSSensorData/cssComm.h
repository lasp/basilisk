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

#ifndef _CSS_COMM_H_
#define _CSS_COMM_H_


#define MAX_NUM_CHEBY_POLYS 32

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/CSSArraySensorMsgPayload.h"

#include "architecture/utilities/bskLogging.h"



/*! @brief Top level structure for the CSS sensor interface system.  Contains all parameters for the
 CSS interface*/
class CSSComm : public SysModel {
public:
    void UpdateState(uint64_t callTime) override;
    void Reset(uint64_t callTime) override;

    uint32_t  numSensors;   //!< The number of sensors we are processing
    ReadFunctor<CSSArraySensorMsgPayload> sensorListInMsg; //!< input message that contains CSS data
    Message<CSSArraySensorMsgPayload> cssArrayOutMsg; //!< output message of corrected CSS data

    double maxSensorValue; //!< Scale factor to go from sensor values to cosine
    uint32_t chebyCount; //!< Count on the number of chebyshev polynominals we have
    double kellyCheby[MAX_NUM_CHEBY_POLYS]; //!< Chebyshev polynominals to fit output to cosine
    BSKLogger bskLogger={};                             //!< BSK Logging
};

#endif
