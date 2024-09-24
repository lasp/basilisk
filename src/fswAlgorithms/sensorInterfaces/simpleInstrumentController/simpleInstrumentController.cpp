/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "fswAlgorithms/sensorInterfaces/simpleInstrumentController/simpleInstrumentController.h"
#include "architecture/utilities/linearAlgebra.h"
#include <stdio.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime [ns] time the method is called
*/
void SimpleInstrumentController::Reset(uint64_t callTime)
{
    // check if the required message has not been connected
    if (!this->locationAccessInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: simpleInstrumentController.locationAccessInMsg wasn't connected.");
    }
    if (!this->attGuidInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: simpleInstrumentController.attGuidInMsg wasn't connected.");
    }

    // reset the imaged variable to zero
    this->imaged = 0;
    this->controllerStatus = 1;
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void SimpleInstrumentController::UpdateState(uint64_t callTime)
{
    double sigma_BR_norm; //!< Norm of sigma_BR
    double omega_BR_norm; //!< Norm of omega_BR

    /* Local copies of the msg buffers*/
    DeviceStatusMsgPayload deviceStatusInMsgBuffer = {}; //!< local copy of input message buffer

    // zero output buffer
    DeviceCmdMsgPayload deviceCmdOutMsgBuffer = {};

    // read in the input messages
    AccessMsgPayload accessInMsgBuffer = this->locationAccessInMsg();
    AttGuidMsgPayload attGuidInMsgBuffer = this->attGuidInMsg();

    // read in the device cmd message if it is connected
    if (this->deviceStatusInMsg.isLinked()) {
        deviceStatusInMsgBuffer = this->deviceStatusInMsg();
        this->controllerStatus = deviceStatusInMsgBuffer.deviceStatus;
    }

    // Compute the norms of the attitude and rate errors
    sigma_BR_norm = v3Norm(attGuidInMsgBuffer.sigma_BR);
    omega_BR_norm = v3Norm(attGuidInMsgBuffer.omega_BR_B);

    // If the controller is active
    if (this->controllerStatus) {
        // If the target has not been imaged
        if (!this->imaged) {
            /* If the attitude error is less than the tolerance, the groundLocation is accessible, and (if enabled) the rate
            error is less than the tolerance, turn on the instrument and set the imaged indicator to 1*/
            if ((sigma_BR_norm <= this->attErrTolerance)
                && (!this->useRateTolerance || (omega_BR_norm <= this->rateErrTolerance)) // Check rate tolerance if useRateTolerance enabled
                && (accessInMsgBuffer.hasAccess))
            {
                deviceCmdOutMsgBuffer.deviceCmd = 1;
                this->imaged = 1;
                // Otherwise, turn off the instrument
            }
        }
    }

    // write to the output messages
    this->deviceCmdOutMsg.write(&deviceCmdOutMsgBuffer, this->moduleID, callTime);

    return;
}
