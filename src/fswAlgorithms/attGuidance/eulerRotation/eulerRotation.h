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

#ifndef _EULER_ROTATION_
#define _EULER_ROTATION_

#include <stdint.h>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AttStateMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"

#include "architecture/utilities/bskLogging.h"




/*! @brief Top level structure for the sub-module routines. */
class EulerRotation : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;


    void checkRasterCommands();
    void computeTimeStep(uint64_t callTime);
    void computeEulerRotationReference(double sigma_R0N[3],
                                       double omega_R0N_N[3],
                                       double domega_R0N_N[3],
                                       AttRefMsgPayload *attRefOut);

    /* Declare module public variables */
    double angleSet[3];                         //!< [-] current euler angle 321 set R/R0  with respect to the input reference
    double angleRates[3];                       //!< [rad/s] euler angle 321 rates

    /* Declare module private variables */
    double cmdSet[3];                           //!< [] msg commanded initial Euler angle 321 set with respect to input reference
    double cmdRates[3];                         //!< [rad/s] msg commanded constant 321 Euler angle rates
    double priorCmdSet[3];                      //!< [] prior commanded 321 Euler angle set
    double priorCmdRates[3];                    //!< [rad/s] prior commanded 321 Euler angle rates
    uint64_t priorTime;                         //!< [ns] last time the guidance module is called
    double dt;                                  //!< [s] integration time-step
    
    /* Declare module IO interfaces */
    Message<AttRefMsgPayload> attRefOutMsg;                   //!< The name of the output message containing the Reference
    ReadFunctor<AttRefMsgPayload> attRefInMsg;                    //!< The name of the guidance reference input message
    ReadFunctor<AttStateMsgPayload>  desiredAttInMsg;             //!< The name of the incoming message containing the desired EA set

    BSKLogger bskLogger={};                             //!< BSK Logging
};

#endif
