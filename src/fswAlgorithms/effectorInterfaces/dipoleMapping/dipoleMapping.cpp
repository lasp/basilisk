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


#include "fswAlgorithms/effectorInterfaces/dipoleMapping/dipoleMapping.h"
#include "architecture/utilities/linearAlgebra.h"

/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.
 @return void
 @param callTime [ns] time the method is called
*/
void DipoleMapping::Reset(uint64_t callTime)
{
    /*
     * Check if the required input messages are connected.
     */
    if (!this->dipoleRequestBodyInMsg.isLinked()){
        this->bskLogger.bskLog(BSK_ERROR, "Error: dipoleMapping.dipoleRequestBodyInMsg is not connected.");
    }
    if (!this->mtbArrayConfigParamsInMsg.isLinked()){
        this->bskLogger.bskLog(BSK_ERROR, "Error: mtbMomentumManagement.mtbArrayConfigParamsInMsg is not connected.");
    }

    /*! - Read in the torque rod input configuration message. This gives us the number of torque rods
         being used on the vehicle.*/
    this->mtbArrayConfigParams = this->mtbArrayConfigParamsInMsg();
}


/*! This method computes takes a requested Body frame dipole into individual torque rod dipole commands using a
    psuedoinverse taking into account saturation limits of the torque rods.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void DipoleMapping::UpdateState(uint64_t callTime)
{
    /*
     * Initialize local variables.
     */
    int j = 0;  // counter used in loop over magnetic torque rods

    /*
     * Read the input messages and initialize output message.
     */
    DipoleRequestBodyMsgPayload dipoleRequestBodyInMsgBuffer = this->dipoleRequestBodyInMsg();
    MTBCmdMsgPayload dipoleRequestMtbOutMsgBuffer = {};

    /*! - Map the requested Body frame dipole request to individual torque rod dipoles.*/
    mMultV(this->steeringMatrix, this->mtbArrayConfigParams.numMTB, 3, dipoleRequestBodyInMsgBuffer.dipole_B, dipoleRequestMtbOutMsgBuffer.mtbDipoleCmds);

    /*! - Saturate the dipole commands if necesarry.*/
    for (j = 0; j < this->mtbArrayConfigParams.numMTB; j++)
    {
        if (dipoleRequestMtbOutMsgBuffer.mtbDipoleCmds[j] > this->mtbArrayConfigParams.maxMtbDipoles[j])
            dipoleRequestMtbOutMsgBuffer.mtbDipoleCmds[j] = this->mtbArrayConfigParams.maxMtbDipoles[j];

        if (dipoleRequestMtbOutMsgBuffer.mtbDipoleCmds[j] < -this->mtbArrayConfigParams.maxMtbDipoles[j])
            dipoleRequestMtbOutMsgBuffer.mtbDipoleCmds[j] = -this->mtbArrayConfigParams.maxMtbDipoles[j];
    }

    /*! - Write output message. Thiis is the individual torque rod dipoel comands.*/
    this->dipoleRequestMtbOutMsg.write(&dipoleRequestMtbOutMsgBuffer, this->moduleID, callTime);
}
