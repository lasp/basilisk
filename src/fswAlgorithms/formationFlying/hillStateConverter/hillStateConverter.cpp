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

/* modify the path to reflect the new module names */
#include "hillStateConverter.h"

// Internal utilities
#include "architecture/utilities/orbitalMotion.h"

/*! This message checks to see that both of the input translational state messages were connected; if not, it errors.
 @return void
 @param callTime [ns] time the method is called
*/
void HillStateConverter::Reset(uint64_t callTime)
{
    // check if the required input messages are included
    if (!this->chiefStateInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: hillStateConverter.chiefStateInMsg wasn't connected.");
    }
    if (!this->depStateInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: hillStateConverter.depStateInMsg wasn't connected.");
    }
}

/*! Computes the relative state of the deputy vs the chief in chief Hill-frame coordinates and writes an output message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void HillStateConverter::UpdateState(uint64_t callTime)
{
    /*! - Read the input messages */
    NavTransMsgPayload chiefStateIn;
    NavTransMsgPayload depStateIn;
    HillRelStateMsgPayload hillStateOut = {};
    chiefStateIn = this->chiefStateInMsg();
    depStateIn = this->depStateInMsg();

    /*! - Add the module specific code */
    rv2hill(chiefStateIn.r_BN_N, chiefStateIn.v_BN_N,
            depStateIn.r_BN_N,  depStateIn.v_BN_N,
            hillStateOut.r_DC_H, hillStateOut.v_DC_H);

    /*! - write the module output message */
    this->hillStateOutMsg.write(&hillStateOut, this->moduleID, callTime);

    return;
}
