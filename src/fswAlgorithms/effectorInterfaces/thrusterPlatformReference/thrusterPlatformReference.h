/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _THRUSTER_PLATFORM_REFERENCE_
#define _THRUSTER_PLATFORM_REFERENCE_

#include <stdint.h>
#include "architecture/utilities/bskLogging.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/RWArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/BodyHeadingMsgPayload.h"
#include "architecture/msgPayloadDefC/THRConfigMsgPayload.h"


enum momentumDumping{
    Yes = 0,
    No  = 1
};

/*! @brief Top level structure for the sub-module routines. */
class ThrusterPlatformReference : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;

    /*! declare these user-defined quantities */
    double sigma_MB[3];                                   //!< orientation of the M frame w.r.t. the B frame
    double r_BM_M[3];                                     //!< position of B frame origin w.r.t. M frame origin, in M frame coordinates
    double r_FM_F[3];                                     //!< position of F frame origin w.r.t. M frame origin, in F frame coordinates

    double K;                                             //!< momentum dumping proportional gain [1/s]
    double Ki;                                            //!< momentum dumping integral gain [1]

    double theta1Max;                                     //!< absolute bound on tip angle [rad]
    double theta2Max;                                     //!< absolute bound on tilt angle [rad]

    /*! declare variables for internal module calculations */
    RWArrayConfigMsgPayload   rwConfigParams;             //!< struct to store message containing RW config parameters in body B frame
    int                       momentumDumping;            //!< flag that assesses whether RW information is provided to perform momentum dumping
    double                    hsInt_M[3];                 //!< integral of RW momentum
    double                    priorHs_M[3];               //!< prior RW momentum
    uint64_t                  priorTime;                  //!< prior call time

    /*! declare module IO interfaces */
    ReadFunctor<VehicleConfigMsgPayload>        vehConfigInMsg;             //!< input msg vehicle configuration msg (needed for CM location)
    ReadFunctor<THRConfigMsgPayload>            thrusterConfigFInMsg;       //!< input thruster configuration msg
    ReadFunctor<RWSpeedMsgPayload>              rwSpeedsInMsg;              //!< input reaction wheel speeds message
    ReadFunctor<RWArrayConfigMsgPayload>        rwConfigDataInMsg;          //!< input RWA configuration message
    Message<HingedRigidBodyMsgPayload>      hingedRigidBodyRef1OutMsg;  //!< output msg containing theta1 reference and thetaDot1 reference
    Message<HingedRigidBodyMsgPayload>      hingedRigidBodyRef2OutMsg;  //!< output msg containing theta2 reference and thetaDot2 reference
    Message<BodyHeadingMsgPayload>          bodyHeadingOutMsg;          //!< output msg containing the thrust heading in body frame coordinates
    Message<CmdTorqueBodyMsgPayload>        thrusterTorqueOutMsg;       //!< output msg containing the opposite of the thruster torque to be compensated by RW's
    Message<THRConfigMsgPayload>            thrusterConfigBOutMsg;      //!< output msg containing the thruster configuration infor in B-frame

    BSKLogger bskLogger={};                                 //!< BSK Logging

};

#endif
