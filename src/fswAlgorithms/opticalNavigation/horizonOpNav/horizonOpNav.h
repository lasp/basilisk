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

#ifndef _HORIZON_OPNAV_H_
#define _HORIZON_OPNAV_H_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/OpNavLimbMsgPayload.h"
#include "architecture/msgPayloadDefC/CameraConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/OpNavMsgPayload.h"

#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/bskLogging.h"


/*! @brief The configuration structure for the horizon OpNav module.*/
class HorizonOpNav : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;

    Message<OpNavMsgPayload> opNavOutMsg; //!< [-] output navigation message for relative position
    ReadFunctor<CameraConfigMsgPayload> cameraConfigInMsg; //!< camera config input message
    ReadFunctor<NavAttMsgPayload> attInMsg; //!< attitude input message
    ReadFunctor<OpNavLimbMsgPayload> limbInMsg; //!< limb input message

    int32_t planetTarget; //!< The planet targeted (None = 0, Earth = 1, Mars = 2, Jupiter = 3 are allowed)
    double noiseSF;   //!< A scale factor to control measurement noise

    BSKLogger bskLogger={};                             //!< BSK Logging
};

void BackSub(double *R, double *inVec, int32_t nRow, double *n);
void QRDecomp(double *inMat, int32_t nRow, double *Q , double *R);

#endif
