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

#ifndef _MRP_ROTATION_
#define _MRP_ROTATION_

#include <stdint.h>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AttStateMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"

#include "architecture/utilities/bskLogging.h"


/*! @brief Top level structure for the sub-module routines. */
class MrpRotation : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;
    void checkRasterCommands();
    void computeTimeStep(uint64_t callTime);
    void computeMRPRotationReference(double sigma_R0N[3],
                                     double omega_R0N_N[3],
                                     double domega_R0N_N[3],
                                     AttRefMsgPayload *attRefOut);
    /* Declare module public variables */
    double mrpSet[3];                           //!< [-] current MRP attitude coordinate set with respect to the input reference
    double omega_RR0_R[3];                      //!< [rad/s] angular velocity vector relative to input reference
    /* Declare module private variables */
    double cmdSet[3];                           //!< [] msg commanded initial MRP sigma_RR0 set with respect to input reference
    double cmdRates[3];                         //!< [rad/s] msg commanded constant angular velocity vector omega_RR0_R
    double priorCmdSet[3];                      //!< [] prior commanded MRP set
    double priorCmdRates[3];                    //!< [rad/s] prior commanded angular velocity vector
    uint64_t priorTime;                         //!< [ns] last time the guidance module is called
    double dt;                                  //!< [s] integration time-step

    /* Declare module IO interfaces */
    Message<AttRefMsgPayload> attRefOutMsg;                   //!< The name of the output message containing the Reference
    ReadFunctor<AttRefMsgPayload> attRefInMsg;                    //!< The name of the guidance reference input message
    ReadFunctor<AttStateMsgPayload>  desiredAttInMsg;             //!< The name of the incoming message containing the desired EA set

    BSKLogger bskLogger={};                             //!< BSK Logging
};

#endif
