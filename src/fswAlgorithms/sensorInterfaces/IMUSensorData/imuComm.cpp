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

#include "fswAlgorithms/sensorInterfaces/IMUSensorData/imuComm.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"

/*! This method resets the module.
 @return void
 @param configData The configuration data associated with the OD filter
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void ImuComm::Reset(uint64_t callTime)
{
    // check if the required message has not been connected
    if (!this->imuComInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: imuComm.imuComInMsg wasn't connected.");
    }
}

/*! This method takes the raw sensor data from the coarse sun sensors and
 converts that information to the format used by the IMU nav.
 @return void
 @param configData The configuration data associated with the IMU interface
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void ImuComm::UpdateState(uint64_t callTime)
{
    // read imu com msg
    IMUSensorMsgPayload LocalInput = this->imuComInMsg();
    IMUSensorBodyMsgPayload outMsgBuffer = {};

    m33MultV3(RECAST3X3 this->dcm_BP, LocalInput.DVFramePlatform, outMsgBuffer.DVFrameBody);
    m33MultV3(RECAST3X3 this->dcm_BP, LocalInput.AccelPlatform, outMsgBuffer.AccelBody);
    m33MultV3(RECAST3X3 this->dcm_BP, LocalInput.DRFramePlatform, outMsgBuffer.DRFrameBody);
    m33MultV3(RECAST3X3 this->dcm_BP, LocalInput.AngVelPlatform, outMsgBuffer.AngVelBody);

    this->imuSensorOutMsg.write(&outMsgBuffer, moduleID, callTime);

    return;
}
