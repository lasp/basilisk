/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "rateDamp.h"

/*! This method is used to reset the module.
 @return void
 */
void RateDamp::Reset(uint64_t CurrentSimNanos)
{
    assert(this->attNavInMsg.isLinked());
}


/*! This method is the main carrier for the computation of the control torque.
 @return void
 @param CurrentSimNanos The current simulation time for system
 */
void RateDamp::UpdateState(uint64_t CurrentSimNanos)
{
    /*! Read input attitude navigation msg */
    NavAttMsgPayload attNavInBuffer = this->attNavInMsg();

    /*! Create and populate cmd torque buffer message */
    CmdTorqueBodyMsgPayload cmdTorqueOutBuffer;
    for (int i=0; i<3; ++i) {
        cmdTorqueOutBuffer.torqueRequestBody[i] = -this->P * attNavInBuffer.omega_BN_B[i];
    }

    /*! Write output messages */
    this->cmdTorqueOutMsg.write(&cmdTorqueOutBuffer, this->moduleID, CurrentSimNanos);
}

/*! Set the module rate feedback gain
    @param double P
    @return void
    */
void RateDamp::setRateGain(const double p) {
    this->P = p;
}

/*! Get the module rate feedback gain
    @param double measurementNoiseScale
    @return void
    */
double RateDamp::getRateGain() const {
    return this->P;
}
