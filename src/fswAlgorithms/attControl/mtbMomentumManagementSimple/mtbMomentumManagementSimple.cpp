/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "mtbMomentumManagementSimple.h"
#include "architecture/utilities/linearAlgebra.h"
#include <stdio.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.
 @return void
 @param callTime [ns] time the method is called
*/
void MtbMomentumManagementSimple::Reset(uint64_t callTime)
{
    /*
     * Check if the required input messages are connected.
     */
    if (!this->rwParamsInMsg.isLinked()){
        this->bskLogger.bskLog(BSK_ERROR, "Error: mtbMomentumManagement.rwParamsInMsg is not connected.");
    }
    if (!this->rwSpeedsInMsg.isLinked()){
        this->bskLogger.bskLog(BSK_ERROR, "Error: mtbMomentumManagement.rwSpeedsInMsg is not connected.");
    }

    /*! - Read in the reaction wheels input configuration message. This gives us the transformation from
         from the wheel space to the Body frame through GsMatrix_B.*/
    this->rwConfigParams = this->rwParamsInMsg();

    /*
     * Compute the transpose of GsMatrix_B, which is an array of the spin
     * axis of the reaction wheels. By transposing it we get the transformation
     * from the wheel space to the Body frame, Gs.
     */
    mTranspose(this->rwConfigParams.GsMatrix_B, this->rwConfigParams.numRW, 3, this->Gs);

    /*
     * Sanity check configs.
     */
    if (this->Kp < 0.0)
        this->bskLogger.bskLog(BSK_ERROR, "Error: k < 0.0");

    return;
}


/*! This routine calculate the current desired torque in the Body frame to meet the momentum target.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MtbMomentumManagementSimple::UpdateState(uint64_t callTime)
{
    /*
     * Initialize local variables.
     */
    double hWheels_B[3] = {0.0, 0.0, 0.0};                      // the net momentum of the reaction wheels in the body frame
    double hWheels_W[MAX_EFF_CNT];                              // array of individual wheel momentum values
    vSetZero(hWheels_W, this->rwConfigParams.numRW);

    /*
     * Read the input message and initialize output message.
     */
    RWSpeedMsgPayload rwSpeedsInMsgBuffer = this->rwSpeedsInMsg();
    CmdTorqueBodyMsgPayload tauMtbRequestOutMsgBuffer = {};

    /*! - Compute wheel momentum in Body frame components by calculating it first in the wheel frame and then
         transforming it from the wheel space into the body frame using Gs.*/
    vElementwiseMult(rwSpeedsInMsgBuffer.wheelSpeeds, this->rwConfigParams.numRW, this->rwConfigParams.JsList, hWheels_W);
    mMultV(this->Gs, 3, this->rwConfigParams.numRW, hWheels_W, hWheels_B);

    /*! - Compute the feedback torque command by multiplying the wheel momentum in the Body frame by the proportional
         momentum gain Kp. Note that this module is currently targeting a wheel momentum in the Body frame of zero and
         hWheels_B is the momentum feedback error and needs to be multiplied by a negative sign.*/
    v3Scale(-this->Kp, hWheels_B, tauMtbRequestOutMsgBuffer.torqueRequestBody);

    /*! - Write the output message. This is the torque we are requesting the torque bars to produce in the Body frame.
         Note that depending on the torque rod/magentic field geometry, torque rod saturation limts, unknown alignments,
         and imperfect sensor readings, this torque may not be perfectly produced.*/
    this->tauMtbRequestOutMsg.write(&tauMtbRequestOutMsgBuffer, moduleID, callTime);
}
