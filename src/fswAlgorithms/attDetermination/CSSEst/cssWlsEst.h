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

#ifndef _CSS_WLS_EST_H_
#define _CSS_WLS_EST_H_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/CSSConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/CSSUnitConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/CSSArraySensorMsgPayload.h"
#include "architecture/msgPayloadDefC/SunlineFilterMsgPayload.h"

#include <stdint.h>


/*! @brief Top level structure for the CSS weighted least squares estimator.
 Used to estimate the sun state in the vehicle body frame*/
class CssWlsEst {
public:
    void Reset(uint64_t callTime);
    void UpdateState(uint64_t callTime);

    CSSArraySensorMsgPayload cssDataInMsgPayload;                   //!< The name of the CSS sensor input message
    CSSConfigMsgPayload cssConfigInMsgPayload;                      //!< The name of the CSS configuration input message
    NavAttMsgPayload navStateOutMsgPayload;                         //!< The name of the navigation output message containing the estimated states
    SunlineFilterMsgPayload cssWlsFiltResOutMsgPayload;             //!< The name of the CSS filter data out message

    uint32_t numActiveCss;                              //!< [-] Number of currently active CSS sensors
    uint32_t useWeights;                                //!< Flag indicating whether or not to use weights for least squares
    uint32_t priorSignalAvailable;                      //!< Flag indicating if a recent prior heading estimate is available
    double dOld[3];                                     //!< The prior sun heading estimate
    double sensorUseThresh;                             //!< Threshold below which we discount sensors
    uint64_t priorTime;                                 //!< [ns] Last time the attitude control is called
};

#endif
