/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#include "fswAlgorithms/sensorInterfaces/scanningInstrumentController/scanningInstrumentController.h"
#include "architecture/utilities/linearAlgebra.h"

/*! This method checks if required input messages (accessInMsg and attGuidInMsg) are connected.
 @return void
 @param this The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void ScanningInstrumentController::Reset(uint64_t callTime)
{
    // check if the required message has not been connected
    if (!this->accessInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: scanningInstrumentController.accessInMsg was not connected.");
    }
    if (!this->attGuidInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: scanningInstrumentController.attGuidInMsg was not connected.");
    }
}

/*! This method checks the status of the device and if there is access to target, as well if the magnitude of the attitude
error and attitude rate are within the tolerance. If so, the instrument is turned on, otherwise it is turned off.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void ScanningInstrumentController::UpdateState(uint64_t callTime)
{
    double sigma_BR_norm; //!< Norm of sigma_BR
    double omega_BR_norm; //!< Norm of omega_BR

    // always zero the output message buffers before assigning values
    DeviceCmdMsgPayload deviceCmdOutMsgBuffer = {};

    // read in the input messages
    AccessMsgPayload accessInMsgBuffer = this->accessInMsg();
    AttGuidMsgPayload attGuidInMsgBuffer = this->attGuidInMsg();

    // Read in the device status message if it is linked
    DeviceStatusMsgPayload deviceStatusInMsgBuffer = {};
    if (this->deviceStatusInMsg.isLinked()) {
        deviceStatusInMsgBuffer = this->deviceStatusInMsg();
        this->controllerStatus = deviceStatusInMsgBuffer.deviceStatus;
    }

    // Compute the norms of the attitude and rate errors
    sigma_BR_norm = v3Norm(attGuidInMsgBuffer.sigma_BR);
    omega_BR_norm = v3Norm(attGuidInMsgBuffer.omega_BR_B);

    // If the controller is active
    if (this->controllerStatus) {
        /* If the attitude error is less than the tolerance, the groundLocation is accessible, and (if enabled) the rate
        error is less than the tolerance, turn on the instrument and set the imaged indicator to 1*/
        if ((sigma_BR_norm <= this->attErrTolerance)
            && (!this->useRateTolerance || (omega_BR_norm <= this->rateErrTolerance)) // Check rate tolerance if useRateTolerance enabled
            && (accessInMsgBuffer.hasAccess))
        {
            deviceCmdOutMsgBuffer.deviceCmd = 1;
        }
    }

    // write to the output messages
    this->deviceCmdOutMsg.write(&deviceCmdOutMsgBuffer, this->moduleID, callTime);
}
