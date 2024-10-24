/*
 ISC License

 Copyright (c) 2024 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef _HILL_POINT_CPP_H_
#define _HILL_POINT_CPP_H_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"
#include <Eigen/Core>

/*! @brief Hill Point attitude guidance class. */
class HillPointCpp: public SysModel {
public:

    HillPointCpp() = default;                                   //!< Constructor
    ~HillPointCpp() override = default;                         //!< Destructor

    void Reset(uint64_t currentSimNanos) override;              //!< Reset function
    void UpdateState(uint64_t currentSimNanos) override;        //!< Update function

    ReadFunctor<NavTransMsgPayload> transNavInMsg;                     //!< The name of the incoming attitude command
    ReadFunctor<EphemerisMsgPayload> celBodyInMsg;            //!< The name of the celestial body message
    Message<AttRefMsgPayload> attRefOutMsg;               //!< The name of the output message

    BSKLogger *bskLogger;                   //!< BSK Logging

private:
    int planetMsgIsLinked;                  //!< flag if the planet message is linked

    static void computeHillPointingReference(Eigen::Vector3d r_BN_N,
                                             Eigen::Vector3d v_BN_N,
                                             Eigen::Vector3d celBdyPositionVector,
                                             Eigen::Vector3d celBdyVelocityVector,
                                             AttRefMsgPayload *attRefOut);
};

#endif
