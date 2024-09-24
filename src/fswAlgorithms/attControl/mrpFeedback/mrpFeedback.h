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

#ifndef _MRP_FEEDBACK_CONTROL_H_
#define _MRP_FEEDBACK_CONTROL_H_

#include <stdint.h>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
#include "architecture/msgPayloadDefC/RWAvailabilityMsgPayload.h"
#include "architecture/msgPayloadDefC/RWArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"


/*! @brief Data configuration structure for the MRP feedback attitude control routine. */
class MrpFeedback : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;

    double K;                           //!< [rad/sec] Proportional gain applied to MRP errors
    double P;                           //!< [N*m*s]   Rate error feedback gain applied
    double Ki;                          //!< [N*m]     Integration feedback error on rate error
    double integralLimit;               //!< [N*m]     Integration limit to avoid wind-up issue
    int    controlLawType;              //!<           Flag to choose between the two control laws available
    uint64_t priorTime;                 //!< [ns]      Last time the attitude control is called
    double z[3];                        //!< [rad]     integral state of delta_omega
    double int_sigma[3];                //!< [s]       integral of the MPR attitude error
    double knownTorquePntB_B[3];        //!< [N*m]     known external torque in body frame vector components

    double ISCPntB_B[9];                //!< [kg m^2]  Spacecraft Inertia
    RWArrayConfigMsgPayload rwConfigParams; //!< [-] struct to store message containing RW config parameters in body B frame

    /* declare module IO interfaces */
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;                         //!< RW speed input message (Optional)
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg;                   //!< RW availability input message (Optional)
    ReadFunctor<RWArrayConfigMsgPayload> rwParamsInMsg;                   //!< RW parameter input message.  (Optional)
    Message<CmdTorqueBodyMsgPayload> cmdTorqueOutMsg;                 //!< commanded spacecraft external control torque output message
    Message<CmdTorqueBodyMsgPayload> intFeedbackTorqueOutMsg;         //!< commanded integral feedback control torque output message
    ReadFunctor<AttGuidMsgPayload> guidInMsg;                             //!< attitude guidance input message
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;                  //!< vehicle configuration input message

    BSKLogger bskLogger = {};                               //!< BSK Logging
};

#endif
