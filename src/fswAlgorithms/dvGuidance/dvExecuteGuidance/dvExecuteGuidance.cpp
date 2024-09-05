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

#include "fswAlgorithms/dvGuidance/dvExecuteGuidance/dvExecuteGuidance.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"
#include <string.h>

/*! @brief This resets the module.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void DvExecuteGuidance::Reset(uint64_t callTime)
{
    // check if the required input messages are included
    if (!this->navDataInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: dvExecuteGuidance.navDataInMsg wasn't connected.");
    }
    if (!this->burnDataInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: dvExecuteGuidance.burnDataInMsg wasn't connected.");
    }
    this->prevCallTime = 0;

    /*! - use default value of 2 seconds for control period of first call if not specified.
     * Control period (FSW rate) is computed dynamically for any subsequent calls.
     */
    this->defaultControlPeriod = (0.0 == this->defaultControlPeriod) ?
                                        2.0 : this->defaultControlPeriod;
}




/*! This method takes its own internal variables and creates an output attitude
    command to use for burn execution.  It also flags whether the burn should
    be happening or not.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void DvExecuteGuidance::UpdateState(uint64_t callTime)
{
    double burnAccum[3];
    double dvExecuteMag;
    double burnDt;
    double dvMag;

    NavTransMsgPayload navData;
    DvBurnCmdMsgPayload localBurnData;
    DvExecutionDataMsgPayload localExeData;
    THRArrayOnTimeCmdMsgPayload effCmd;

    // read in messages
    navData = this->navDataInMsg();
    localBurnData = this->burnDataInMsg();

    /*! - The first time update() is called there is no information on the time step.
     *    Use control period (FSW time step) as burn time delta-t */
	if(this->prevCallTime == 0) {
        burnDt = this->defaultControlPeriod;
	} else {
        /*! - compute burn time delta-t (control time period) */
        burnDt = (double) ((int64_t) callTime - (int64_t) this->prevCallTime)*NANO2SEC;
    }
    this->prevCallTime = callTime;
    v3SetZero(burnAccum);
    if((this->burnExecuting == 0 && callTime >= localBurnData.burnStartTime)
       && this->burnComplete != 1)
    {
        this->burnExecuting = 1;
        v3Copy(navData.vehAccumDV, this->dvInit);
        this->burnComplete = 0;
    }

    if(this->burnExecuting)
    {
        this->burnTime += burnDt;
    }

    v3Subtract(navData.vehAccumDV, this->dvInit, burnAccum);

    dvMag = v3Norm(localBurnData.dvInrtlCmd);
    dvExecuteMag = v3Norm(burnAccum);
    this->burnComplete = this->burnComplete == 1 || dvExecuteMag >= dvMag;
    this->burnComplete &= this->burnTime > this->minTime;
    this->burnComplete |= (this->maxTime != 0.0 && this->burnTime > this->maxTime);
    this->burnExecuting = this->burnComplete != 1 && this->burnExecuting == 1;

    if(this->burnComplete || this->burnExecuting != 1)
    {
        effCmd = {};
        this->thrCmdOutMsg.write(&effCmd, this->moduleID, callTime);
    }

    localExeData = {};
    localExeData.burnComplete = this->burnComplete;
    localExeData.burnExecuting = this->burnExecuting;
    this->burnExecOutMsg.write(&localExeData, this->moduleID, callTime);

    return;
}
