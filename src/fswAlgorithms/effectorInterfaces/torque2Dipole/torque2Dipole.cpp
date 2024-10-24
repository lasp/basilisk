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


#include "fswAlgorithms/effectorInterfaces/torque2Dipole/torque2Dipole.h"
#include "architecture/utilities/linearAlgebra.h"

/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.
 @return void
 @param callTime [ns] time the method is called
*/
void Torque2Dipole::Reset(uint64_t callTime)
{
    /*
     * Check if the required input messages are connected.
     */
    if (!this->tamSensorBodyInMsg.isLinked()){
        this->bskLogger.bskLog(BSK_ERROR, "Error: torque2Dipole.tamSensorBodyInMsg is not connected.");
    }
    if (!this->tauRequestInMsg.isLinked()){
        this->bskLogger.bskLog(BSK_ERROR, "Error: torque2Dipole.tauRequestInMsg is not connected.");
    }
}


/*! This method transforms the requested torque from the torque rods into a Body frame requested dipole from the torque rods.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void Torque2Dipole::UpdateState(uint64_t callTime)
{
    /*
     * Initialize local variables.
     */
    double bFieldNormSqrd = 0.0;        // the norm squared of the local magnetic field vector

    /*
     * Read the input messages and initialize output message.
     */
    TAMSensorBodyMsgPayload tamSensorBodyInMsgBuffer = this->tamSensorBodyInMsg();
    CmdTorqueBodyMsgPayload tauRequestInMsgBuffer = this->tauRequestInMsg();
    DipoleRequestBodyMsgPayload dipoleRequestOutMsgBuffer = {};

    /*! - Transform the requested Body frame torque into a requested Body frame dipole protecting against a bogus
         magnetic field value. */
    bFieldNormSqrd = v3Dot(tamSensorBodyInMsgBuffer.tam_B, tamSensorBodyInMsgBuffer.tam_B);
    if (bFieldNormSqrd > DB0_EPS)
    {
        v3Cross(tamSensorBodyInMsgBuffer.tam_B, tauRequestInMsgBuffer.torqueRequestBody, dipoleRequestOutMsgBuffer.dipole_B);
        v3Scale(1 / bFieldNormSqrd, dipoleRequestOutMsgBuffer.dipole_B, dipoleRequestOutMsgBuffer.dipole_B);
    }

    /*! - Write output message. This is the Body frame requested dipole from the torque rods.*/
    this->dipoleRequestOutMsg.write(&dipoleRequestOutMsgBuffer, this->moduleID, callTime);
}
