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

#ifndef _FAULT_DETECTION_H_
#define _FAULT_DETECTION_H_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/OpNavMsgPayload.h"
#include "architecture/msgPayloadDefC/CameraConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"

#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/rigidBodyKinematics.h"

/*! @brief Module data structure */
class FaultDetection : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;

    Message<OpNavMsgPayload> opNavOutMsg; //!< [-] output navigation message for relative position
    ReadFunctor<NavAttMsgPayload> attInMsg; //!< attitude input message
    ReadFunctor<OpNavMsgPayload> navMeasPrimaryInMsg; //!< first measurement input message
    ReadFunctor<OpNavMsgPayload> navMeasSecondaryInMsg; //!< second measurement input message
    ReadFunctor<CameraConfigMsgPayload> cameraConfigInMsg; //!< camera config inut message

    int32_t planetTarget; //!< The planet targeted (None = 0, Earth = 1, Mars = 2, Jupiter = 3 are allowed)
    double faultMode; //!< What fault mode to go in: 0 is dissimilar (use the primary measurement and compare with secondary), 1 merges the measurements if they are both valid and similar.
    double sigmaFault; //!< What is the sigma multiplication factor when comparing measurements

    // added for bsk
    BSKLogger bskLogger={};                               //!< BSK Logging

};


#endif
