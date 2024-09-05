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

#include "fswAlgorithms/transDetermination/ephemDifference/ephemDifference.h"
#include "architecture/utilities/linearAlgebra.h"

/*! @brief This method resets the module.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void EphemDifference::Reset(uint64_t callTime)
{
    // check if the required message has not been connected
    if (!this->ephBaseInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: ephemDifference.ephBaseInMsg wasn't connected.");
    }

    this->ephBdyCount = 0;
    for(int i = 0; i < MAX_NUM_CHANGE_BODIES; i++)
    {
        if (this->changeBodies[i].ephInMsg.isLinked()) {
            this->ephBdyCount++;
        } else {
            break;
        }
    }

    if (this->ephBdyCount == 0) {
        this->bskLogger.bskLog(BSK_WARNING,
                                "Your outgoing ephemeris message count is zero. "
                                "Be sure to specify desired output messages.");
    }
}

/*! @brief This method recomputes the body positions and velocities relative to
    the base body ephemeris and writes out updated ephemeris position and velocity
    for each body.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void EphemDifference::UpdateState(uint64_t callTime)
{
    // read input msg
    EphemerisMsgPayload tmpBaseEphem = this->ephBaseInMsg();

    for(uint32_t i = 0; i < this->ephBdyCount; i++)
    {
        auto tmpEphStore = this->changeBodies[i].ephInMsg();

        v3Subtract(tmpEphStore.r_BdyZero_N,
                   tmpBaseEphem.r_BdyZero_N,
                   tmpEphStore.r_BdyZero_N);
        v3Subtract(tmpEphStore.v_BdyZero_N,
                   tmpBaseEphem.v_BdyZero_N,
                   tmpEphStore.v_BdyZero_N);

        this->changeBodies[i].ephOutMsg.write(&tmpEphStore, moduleID, callTime);
    }
}
