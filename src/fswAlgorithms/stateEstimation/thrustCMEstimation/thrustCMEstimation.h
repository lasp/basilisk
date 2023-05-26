/*
 ISC License
 
 Copyright (c) 2023, Laboratory  for Atmospheric and Space Physics, University of Colorado at Boulder
 
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


/*! @brief Top level structure for the thrust CM estimation kalman filter.
 Used to estimate the spacecraft's center of mass position with respect to the B frame.
 */

#ifndef THRUSTCMESTIMATION_H
#define THRUSTCMESTIMATION_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/msgPayloadDefC/THRConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"

#include <string.h>
#include <array>
#include <math.h>

class ThrustCMEstimation: public SysModel {
public:
    ThrustCMEstimation();
    ~ThrustCMEstimation() override;
    void Reset(uint64_t CurrentSimNanos) override;
    void UpdateState(uint64_t CurrentSimNanos) override;

    /* declare these user-defined quantities */
    double epsilon;

private:


public:
    ReadFunctor<THRConfigMsgPayload>      thrusterConfigInMsg;
    ReadFunctor<CmdTorqueBodyMsgPayload>  cmdTorqueInMsg;
    ReadFunctor<AttGuidMsgPayload>        attGuidInMsg;
    ReadFunctor<A
    Message<VehicleConfigMsgPayload>      vehConfigInMsg;

    double measNoiseScaling = 1; //!< [s] Scale factor that can be applied on the measurement noise to over/under weight

    double attError;

private:
    // THRConfigMsgPayload      thrConfigMsgBuffer;  //!< Message buffer

    BSKLogger bskLogger; //!< -- BSK Logging
};

#endif
