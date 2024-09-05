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

#ifndef _ET_SPHERICAL_CONTROL_H_
#define _ET_SPHERICAL_CONTROL_H_

#include <stdint.h>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdForceInertialMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdForceBodyMsgPayload.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief Top level structure for the sub-module routines. */
class EtSphericalControl : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;
    void calcRelativeMotionControl(NavTransMsgPayload servicerTransInMsgBuffer,
                                    NavTransMsgPayload debrisTransInMsgBuffer, NavAttMsgPayload servicerAttInMsgBuffer,
                                    VehicleConfigMsgPayload servicerVehicleConfigInMsgBuffer,
                                    VehicleConfigMsgPayload debrisVehicleConfigInMsgBuffer,
                                    CmdForceInertialMsgPayload eForceInMsgBuffer,
                                    CmdForceInertialMsgPayload *forceInertialOutMsgBuffer,
                                    CmdForceBodyMsgPayload *forceBodyOutMsgBuffer);
    // declare module IO interfaces
    ReadFunctor<NavTransMsgPayload> servicerTransInMsg;                   //!< servicer orbit input message
    ReadFunctor<NavTransMsgPayload> debrisTransInMsg;                     //!< debris orbit input message
    ReadFunctor<NavAttMsgPayload> servicerAttInMsg;                       //!< servicer attitude input message
    ReadFunctor<VehicleConfigMsgPayload> servicerVehicleConfigInMsg;      //!< servicer vehicle configuration (mass) input message
    ReadFunctor<VehicleConfigMsgPayload> debrisVehicleConfigInMsg;        //!< debris vehicle configuration (mass) input message
    ReadFunctor<CmdForceInertialMsgPayload> eForceInMsg;                  //!< servicer electrostatic force input message
    Message<CmdForceInertialMsgPayload> forceInertialOutMsg;          //!< servicer inertial frame control force output message
    Message<CmdForceBodyMsgPayload> forceBodyOutMsg;                  //!< servicer body frame control force output message

    double mu;                                          //!< [m^3/s^2]  gravitational parameter
    double L_r;                                         //!< [m]  reference separation distance
    double theta_r;                                     //!< [rad]  reference in-plane rotation angle
    double phi_r;                                       //!< [rad]  reference out-of-plane rotation angle
    double K[9];                                        //!< 3x3 symmetric positive definite feedback gain matrix [K]
    double P[9];                                        //!< 3x3 symmetric positive definite feedback gain matrix [P]
    BSKLogger bskLogger={};                               //!< BSK Logging
};

#endif
