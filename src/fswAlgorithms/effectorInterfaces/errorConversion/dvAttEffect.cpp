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

#include "fswAlgorithms/effectorInterfaces/errorConversion/dvAttEffect.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <string.h>

void effectorVSort(effPairs *Input, effPairs *Output, size_t dim);

/*! This method resets the module.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void DvAttEffect::Reset(uint64_t callTime)
{
    // check if the required input messages are included
    if (!this->cmdTorqueBodyInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: dvAttEffect.cmdTorqueBodyInMsg wasn't connected.");
    }

    for(uint32_t i=0; i<this->numThrGroups; i=i+1)
    {
        this->thrGroups[i].cmdRequests = {};
        this->thrGroups[i].thrOnTimeOutMsg.write(&this->thrGroups[i].cmdRequests, this->moduleID, callTime);
    }

}

/*! This method takes the estimated body-observed sun vector and computes the
 current attitude/attitude rate errors to pass on to control.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void DvAttEffect::UpdateState(uint64_t callTime)
{
    uint32_t i;
    CmdTorqueBodyMsgPayload cntrRequest;

    /*! - Read the input requested torque from the feedback controller*/
    cntrRequest = this->cmdTorqueBodyInMsg();

    for(i=0; i<this->numThrGroups; i=i+1)
    {
        computeSingleThrustBlock(&(this->thrGroups[i]), callTime,
            &cntrRequest, moduleID);
    }

    return;
}

void computeSingleThrustBlock(ThrustGroupData *thrData,
                              uint64_t callTime,
                              CmdTorqueBodyMsgPayload *contrReq,
                              int64_t moduleID)
{
    double unSortOnTime[MAX_EFF_CNT];
    effPairs unSortPairs[MAX_EFF_CNT];
    effPairs sortPairs[MAX_EFF_CNT];
    uint32_t i;
    double localRequest[3];

    v3Copy(contrReq->torqueRequestBody, localRequest);      /* to generate a positive torque onto the spacecraft */
    mMultV(thrData->thrOnMap, thrData->numEffectors, 3,
           localRequest, unSortOnTime);

    for(i=0; i<thrData->numEffectors; i=i+1)
    {
        unSortOnTime[i] = unSortOnTime[i] + thrData->nomThrustOn;
    }

    for(i=0; i<thrData->numEffectors; i=i+1)
    {
        if(unSortOnTime[i] < thrData->minThrustRequest)
        {
            unSortOnTime[i] = 0.0;
        }
    }

    for(i=0; i<thrData->numEffectors; i++)
    {
        unSortPairs[i].onTime = unSortOnTime[i];
        unSortPairs[i].thrustIndex = i;
    }
    effectorVSort(unSortPairs, sortPairs, thrData->numEffectors);
    thrData->cmdRequests = {};
    for(i=0; i<thrData->maxNumCmds; i=i+1)
    {
        thrData->cmdRequests.OnTimeRequest[sortPairs[i].thrustIndex] =
        sortPairs[i].onTime;
    }
    thrData->thrOnTimeOutMsg.write(&thrData->cmdRequests, moduleID, callTime);
}

void effectorVSort(effPairs *Input, effPairs *Output, size_t dim)
{
    size_t i, j;
    int Swapped;
    Swapped = 1;
    memcpy(Output, Input, dim*sizeof(effPairs));
    for(i=0; i<dim && Swapped > 0; i++)
    {
        Swapped = 0;
        for(j=0; j<dim-1; j++)
        {
            if(Output[j].onTime<Output[j+1].onTime)
            {
                double tempOn = Output[j+1].onTime;
                uint32_t tempIndex = Output[j+1].thrustIndex;
                Output[j+1].onTime = Output[j].onTime;
                Output[j+1].thrustIndex = Output[j].thrustIndex;
                Output[j].onTime = tempOn;
                Output[j].thrustIndex = tempIndex;
                Swapped = 1;
            }
        }
    }
}
