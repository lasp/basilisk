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

#ifndef _INERTIAL3D_SPIN_
#define _INERTIAL3D_SPIN_

#include <stdint.h>
#include "architecture/utilities/bskLogging.h"

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"

/*!@brief module configuration structure definition.
 */
class Inertial3DSpin : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;
    void computeReference_inertial3DSpin(double omega_R0N_N[3],
                                         double domega_R0N_N[3],
                                         double dt);
    /* declare module private variables */
    double sigma_RN[3];                              /*!< MPR of reference frame relative to inertial N frame */
    double omega_RR0_R0[3];                          /*!< [r/s] constant angular velocity spin vector of the spinning R frame relative to the input frame R0 */
    uint64_t priorTime;                              /*!< [ns] last time the guidance module is called */
    /* declare module IO interfaces */
    Message<AttRefMsgPayload> attRefOutMsg;                     //!< reference attitude output message
    ReadFunctor<AttRefMsgPayload> attRefInMsg;                      //!< (optional) reference attitude input message

    AttRefMsgPayload attRefOutBuffer;                    //!< [-] structure for the output data
    BSKLogger bskLogger={};                             //!< BSK Logging
};

#endif
