/*
 ISC License

 Copyright (c) 2019, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "fswAlgorithms/sensorInterfaces/TAMSensorData/tamComm.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include <math.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void TamComm::Reset(uint64_t callTime)
{
    // check if the required message has not been connected
    if (!this->tamInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: tamComm.tamInMsg wasn't connected.");
    }

    if (fabs(m33Determinant(RECAST3X3 this->dcm_BS) - 1.0) > 1e-10) {
        this->bskLogger.bskLog(BSK_WARNING, "dcm_BS is set to zero values.");
    }

    return;
}

/*! This method takes the sensor data from the magnetometers and
 converts that information to the format used by the TAM nav.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void TamComm::UpdateState(uint64_t callTime)
{
    // read input msg
    TAMSensorMsgPayload localInput = this->tamInMsg();

    m33MultV3(RECAST3X3 this->dcm_BS, localInput.tam_S,
              this->tamLocalOutput.tam_B);

    /*! - Write aggregate output into output message */
    this->tamOutMsg.write(&tamLocalOutput, moduleID, callTime);

    return;
}
