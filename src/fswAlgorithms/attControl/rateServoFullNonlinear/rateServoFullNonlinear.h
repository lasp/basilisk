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

#ifndef _RATE_SERVO_FULL_NONLINEAR_
#define _RATE_SERVO_FULL_NONLINEAR_


#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/RWArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/RWAvailabilityMsgPayload.h"
#include "architecture/msgPayloadDefC/RateCmdMsgPayload.h"
#include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"

#include <stdint.h>



/*! @brief The configuration structure for the rateServoFullNonlinear module.  */
class RateServoFullNonlinear : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;

    /* declare module public variables */
    double P;                           //!< [N*m*s]   Rate error feedback gain applied
    double Ki;                          //!< [N*m]     Integration feedback error on rate error
    double knownTorquePntB_B[3];        //!< [N*m]     known external torque in body frame vector components
    double integralLimit;               //!< [N*m]     Integration limit to avoid wind-up issue

    /* declare module private variables */
    uint64_t priorTime;                 //!< [ns]      Last time the attitude control is called
    double z[3];                        //!< [rad]     integral state of delta_omega
    double ISCPntB_B[9];                //!< [kg m^2] Spacecraft Inertia
    RWArrayConfigMsgPayload rwConfigParams; //!< [-] struct to store message containing RW config parameters in body B frame

    /* declare module IO interfaces */
    Message<CmdTorqueBodyMsgPayload> cmdTorqueOutMsg;             //!< commanded torque output message
    ReadFunctor<AttGuidMsgPayload> guidInMsg;                         //!< attitude guidance input message
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;              //!< vehicle configuration input message
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;                     //!< (optional) RW speed input message
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg;               //!< (optional) RW availability input message
    ReadFunctor<RWArrayConfigMsgPayload> rwParamsInMsg;               //!< (optional) RW configuration parameter input message
    ReadFunctor<RateCmdMsgPayload> rateSteeringInMsg;                 //!< commanded rate input message

    BSKLogger bskLogger = {};                           //!< BSK Logging

};

#endif
