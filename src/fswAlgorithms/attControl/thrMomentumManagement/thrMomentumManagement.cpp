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
/*
    Thruster RW Momentum Management

 */

#include "fswAlgorithms/attControl/thrMomentumManagement/thrMomentumManagement.h"
#include "architecture/utilities/linearAlgebra.h"
#include <string.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrMomentumManagement::Reset(uint64_t callTime)
{
    // check if the required input messages are included
    if (!this->rwConfigDataInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrMomentumManagement.rwConfigDataInMsg wasn't connected.");
    }
    if (!this->rwSpeedsInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrMomentumManagement.rwSpeedsInMsg wasn't connected.");
    }

    /*! - read in the RW configuration message */
    this->rwConfigParams = this->rwConfigDataInMsg();

    /*! - reset the momentum dumping request flag */
    this->initRequest = 1;
}

/*! The RW momentum level is assessed to determine if a momentum dumping maneuver is required.
 This checking only happens once after the reset function is called.  To run this again afterwards,
 the reset function must be called again.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrMomentumManagement::UpdateState(uint64_t callTime)
{
    RWSpeedMsgPayload   rwSpeedMsg;         /* Reaction wheel speed estimate message */
    CmdTorqueBodyMsgPayload controlOutMsg = {};  /* Control torque output message */
    double              hs;                 /* net RW cluster angular momentum magnitude */
    double              hs_B[3];            /* RW angular momentum */
    double              vec3[3];            /* temp vector */
    double              Delta_H_B[3];       /* [Nms]  net desired angular momentum change */
    int i;

    /*! - check if a momentum dumping check has been requested */
    if (this->initRequest == 1) {

        /*! - Read the input messages */
        rwSpeedMsg = this->rwSpeedsInMsg();

        /*! - compute net RW momentum magnitude */
        v3SetZero(hs_B);
        for (i=0;i<this->rwConfigParams.numRW;i++) {
            v3Scale(this->rwConfigParams.JsList[i]*rwSpeedMsg.wheelSpeeds[i],&this->rwConfigParams.GsMatrix_B[i*3],vec3);
            v3Add(hs_B, vec3, hs_B);
        }
        hs = v3Norm(hs_B);

        /*! - check if momentum dumping is required */
        if (hs < this->hs_min) {
            /* Momentum dumping not required */
            v3SetZero(Delta_H_B);
        } else {
            v3Scale(-(hs - this->hs_min)/hs, hs_B, Delta_H_B);
        }
        this->initRequest = 0;


        /*! - write out the output message */
        v3Copy(Delta_H_B, controlOutMsg.torqueRequestBody);

        this->deltaHOutMsg.write(&controlOutMsg, moduleID, callTime);

    }

    return;
}
