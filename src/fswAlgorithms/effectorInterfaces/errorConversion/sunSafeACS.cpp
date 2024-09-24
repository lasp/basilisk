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

#include "fswAlgorithms/effectorInterfaces/errorConversion/sunSafeACS.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <string.h>

/*! This method resets the module.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SunSafeACS::Reset(uint64_t callTime)
{
    // check if the required input messages are included
    if (!this->cmdTorqueBodyInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: sunSafeACS.cmdTorqueBodyInMsg wasn't connected.");
    }
}


/*! This method takes the estimated body-observed sun vector and computes the
 current attitude/attitude rate errors to pass on to control.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SunSafeACS::UpdateState(uint64_t callTime)
{
    CmdTorqueBodyMsgPayload cntrRequest;

    /*! - Read the input parsed CSS sensor data message*/
    cntrRequest = this->cmdTorqueBodyInMsg();
    computeSingleThrustBlock(&(this->thrData), callTime,
                             &cntrRequest, moduleID);

    return;
}
