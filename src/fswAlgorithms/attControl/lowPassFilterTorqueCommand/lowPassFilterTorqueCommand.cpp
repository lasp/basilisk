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
/*
    Control Torque Low Pass Filter Module

 */

/* modify the path to reflect the new module names */
#include "fswAlgorithms/attControl/lowPassFilterTorqueCommand/lowPassFilterTorqueCommand.h"
#include "architecture/utilities/linearAlgebra.h"
#include "fswAlgorithms/fswUtilities/fswDefinitions.h"
#include "math.h"


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void LowPassFilterTorqueCommand::Reset(uint64_t callTime)
{
    int i;

    this->reset  = BOOL_TRUE;         /* reset the first run flag */

    // check if the required input message is included
    if (!this->cmdTorqueInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: lowPassFilterTorqueCommand.cmdTorqueInMsg wasn't connected.");
    }

    for (i=0;i<NUM_LPF;i++) {
        v3SetZero(this->Lr[i]);
        v3SetZero(this->LrF[i]);
    }
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void LowPassFilterTorqueCommand::UpdateState(uint64_t callTime)
{
    double      v3[3];                      /*!<      3d vector sub-result */
    int         i;
    CmdTorqueBodyMsgPayload controlOut = {};             /*!< -- Control output message */

    /* - Read the input messages */
    CmdTorqueBodyMsgPayload msgBuffer = this->cmdTorqueInMsg();
    v3Copy(msgBuffer.torqueRequestBody, this->Lr[0]);

    /*
        check if the filter states must be reset
     */
    if (this->reset) {
        /* populate the filter history with 1st input */
        for (i=1;i<NUM_LPF;i++) {
            v3Copy(this->Lr[0], this->Lr[i]);
        }

        /* zero the history of filtered outputs */
        for (i=0;i<NUM_LPF;i++) {
            v3SetZero(this->LrF[i]);
        }

        /* compute h times the prewarped critical filter frequency */
        this->hw = tan(this->wc * this->h / 2.0)*2.0;

        /* determine 1st order low-pass filter coefficients */
        this->a[0] = 2.0 + this->hw;
        this->a[1] = 2.0 - this->hw;
        this->b[0] = this->hw;
        this->b[1] = this->hw;

        /* turn off first run flag */
        this->reset = BOOL_FALSE;

    }

    /*
        regular filter run
     */

    v3SetZero(this->LrF[0]);
    for (i=0;i<NUM_LPF;i++) {
        v3Scale(this->b[i], this->Lr[i], v3);
        v3Add(v3, this->LrF[0], this->LrF[0]);
    }
    for (i=1;i<NUM_LPF;i++) {
        v3Scale(this->a[i], this->LrF[i], v3);
        v3Add(v3, this->LrF[0], this->LrF[0]);
    }
    v3Scale(1.0/this->a[0], this->LrF[0], this->LrF[0]);


    /* reset the filter state history */
    for (i=1;i<NUM_LPF;i++) {
        v3Copy(this->Lr[NUM_LPF-1-i],  this->Lr[NUM_LPF-i]);
        v3Copy(this->LrF[NUM_LPF-1-i], this->LrF[NUM_LPF-i]);
    }

    /*
        store the output message
     */
    v3Copy(this->LrF[0], controlOut.torqueRequestBody);
    this->cmdTorqueOutMsg.write(&controlOut, this->moduleID, callTime);

    return;
}
