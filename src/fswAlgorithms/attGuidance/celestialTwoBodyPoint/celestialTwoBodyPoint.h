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

#ifndef _CELESTIAL_BODY_POINT_H_
#define _CELESTIAL_BODY_POINT_H_

#include <stdint.h>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"

#include "architecture/utilities/bskLogging.h"



/*!@brief Data structure for module to compute the two-body celestial pointing navigation solution.
 */
class CelestialTwoBodyPoint : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;
    void parseInputMessages();
    void computeCelestialTwoBodyPoint(uint64_t callTime);
    /* Declare module private variables */
    double singularityThresh;       //!< (r) Threshold for when to fix constraint axis*/
    double R_P1B_N[3];              //!< [m] planet 1 position vector relative to inertial frame, in N-frame components
    double R_P2B_N[3];              //!< [m] planet 2 position vector relative to inertial frame, in N-frame components
    double v_P1B_N[3];              //!< [m/s] planet 1 velocity vector relative to inertial frame, in N-frame components
    double v_P2B_N[3];              //!< [m/s] planet 2 velocity vector relative to inertial frame, in N-frame components
    double a_P1B_N[3];              //!< [m/s^2] planet 1 acceleration vector relative to inertial frame, in N-frame components
    double a_P2B_N[3];              //!< [m/s^2] planet 2 acceleration vector relative to inertial frame, in N-frame components


    /* Declare module IO interfaces */
    Message<AttRefMsgPayload> attRefOutMsg;                       //!< The name of the output message*/
    ReadFunctor<EphemerisMsgPayload> celBodyInMsg;                    //!< The name of the celestial body message*/
    ReadFunctor<EphemerisMsgPayload> secCelBodyInMsg;                 //!< The name of the secondary body to constrain point*/
    ReadFunctor<NavTransMsgPayload> transNavInMsg;                    //!< The name of the incoming attitude command*/

    int secCelBodyIsLinked;                         //!< flag to indicate if the optional 2nd celestial body message is linked

    /* Output attitude reference data to send */
    AttRefMsgPayload attRefOut;                     //!< (-) copy of output reference frame message

    BSKLogger bskLogger={};                             //!< BSK Logging
};

#endif
