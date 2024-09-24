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
    FSW MODULE thrMomentumDumping

 */

#include "fswAlgorithms/effectorInterfaces/thrMomentumDumping/thrMomentumDumping.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/linearAlgebra.h"
#include <string.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrMomentumDumping::Reset(uint64_t callTime)
{
    THRArrayConfigMsgPayload    localThrusterData;     /* local copy of the thruster data message */
    CmdTorqueBodyMsgPayload     DeltaHInMsg;
    int                         i;

    /*! - reset the prior time flag state.  If set to zero, the control time step is not evaluated on the
     first function call */
    this->priorTime = 0;

    // check if the required input messages are included
    if (!this->thrusterConfInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrMomentumDumping.thrusterConfInMsg wasn't connected.");
    }
    if (!this->deltaHInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrMomentumDumping.deltaHInMsg wasn't connected.");
    }
    if (!this->thrusterImpulseInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrMomentumDumping.thrusterImpulseInMsg wasn't connected.");
    }

    /*! - read in number of thrusters installed and maximum thrust values */
    localThrusterData = this->thrusterConfInMsg();
    this->numThrusters = localThrusterData.numThrusters;
    for (i=0;i<this->numThrusters;i++) {
        this->thrMaxForce[i] = localThrusterData.thrusters[i].maxThrust;
    }

    /*! - reset dumping counter */
    this->thrDumpingCounter = 0;

    /*! - zero out thruster on time array */
    mSetZero(this->thrOnTimeRemaining, 1, MAX_EFF_CNT);

    /*! - set the time tag of the last Delta_p message */
    if (this->deltaHInMsg.isWritten()) {
        DeltaHInMsg = this->deltaHInMsg();
        /* prior message has been written, copy its time tag as the last prior message */
        this->lastDeltaHInMsgTime = this->deltaHInMsg.timeWritten();
    } else {
        this->lastDeltaHInMsgTime = 0;
    }
    mSetZero(this->Delta_p, 1, MAX_EFF_CNT);

    /*! - perform sanity check that the module maxCounterValue value is set to a positive value */
    if (this->maxCounterValue < 1) {
        this->bskLogger.bskLog(BSK_WARNING,"The maxCounterValue flag must be set to a positive value.");
    }

}

/*! This method reads in the requested thruster impulse message.  If it is a new message then a fresh
 thruster firing cycle is setup to achieve the desired RW momentum dumping.  The the same message is read
 in, then the thrust continue to periodically fire to achieve the net thruster impuleses requested.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrMomentumDumping::UpdateState(uint64_t callTime)
{
    double              dt;                             /* [s]    control update period */
    double              *Delta_P_input;                 /* []     pointer to vector of requested net thruster impulses */
    double              *tOnOut;                        /*        pointer to vector of requested thruster on times per dumping cycle */
    THRArrayOnTimeCmdMsgPayload thrOnTimeOut = {};           /* []     output message container */
    THRArrayCmdForceMsgPayload thrusterImpulseInMsg;    /* []     thruster inpulse input message */
    CmdTorqueBodyMsgPayload  DeltaHInMsg;               /* []     commanded Delta_H input message */
    uint64_t            timeOfDeltaHMsg;
    int                 i;

    /*! - zero the output array of on-time values */
    tOnOut = thrOnTimeOut.OnTimeRequest;

    /*! - check if this is the first call after reset.  If yes, write zero output message and exit */
    if (this->priorTime != 0) {       /* don't compute dt if this is the first call after a reset */

        /* - compute control update time */
        dt = (callTime - this->priorTime)*NANO2SEC;
        if (dt < 0.0) {dt = 0.0;}             /* ensure no negative numbers are used */

        /*! - Read the requester thruster impulse input message */
        thrusterImpulseInMsg = this->thrusterImpulseInMsg();
        Delta_P_input = thrusterImpulseInMsg.thrForce;

        /*! - check if the thruster impulse input message time tag is identical to current values (continue
         with current momentum dumping) */
        DeltaHInMsg = this->deltaHInMsg();
        timeOfDeltaHMsg = this->deltaHInMsg.timeWritten();
        if (this->lastDeltaHInMsgTime == timeOfDeltaHMsg){
            /* identical net thruster impulse request case, continue with existing RW momentum dumping */
            if (this->thrDumpingCounter <= 0) {
                /* time to fire thrusters again */
                mCopy(this->thrOnTimeRemaining, 1, this->numThrusters, tOnOut);
                /* subtract next control period from remaining impulse time */
                for (i=0;i<this->numThrusters;i++) {
                    if (this->thrOnTimeRemaining[i] >0.0){
                        this->thrOnTimeRemaining[i] -= dt;
                    }
                }
                /* reset the dumping counter */
                this->thrDumpingCounter = this->maxCounterValue;
            } else {
                /* no thrusters are firing, giving RWs time to settle attitude */
                this->thrDumpingCounter -= 1;
            }


        } else {
            /* new net thruster impulse request case */
            this->lastDeltaHInMsgTime = timeOfDeltaHMsg;
            mCopy(Delta_P_input, 1, this->numThrusters, this->Delta_p); /* store current Delta_p */
            for (i=0;i<this->numThrusters;i++) {
                /* compute net time required to implement requested thruster impulse */
                this->thrOnTimeRemaining[i] = this->Delta_p[i]/this->thrMaxForce[i];
            }
            /* set thruster on time to requested impulse time */
            mCopy(this->thrOnTimeRemaining, 1, this->numThrusters, tOnOut);
            /* reset the dumping counter */
            this->thrDumpingCounter = this->maxCounterValue;
            /* subtract next control period from remaining impulse time */
            for (i=0;i<this->numThrusters;i++) {
                this->thrOnTimeRemaining[i] -= dt;
            }
        }

        /*! - check for negative, saturated firing times or negative remaining times */
        for (i=0;i<this->numThrusters;i++) {
            /* if thruster on time is less than the minimum firing time, set thrust time command to zero */
            if (tOnOut[i] < this->thrMinFireTime){
                tOnOut[i] = 0.0;
            }
            /* if the thruster time remainder is negative, zero out the remainder */
            if (this->thrOnTimeRemaining[i] < 0.0){
                this->thrOnTimeRemaining[i] = 0.0;
            }
            /* if the thruster on time is larger than the control period, set it equal to control period */
            if (tOnOut[i] > dt){
                tOnOut[i] = dt;
            }
        }
    }

    this->priorTime = callTime;

    /*! - write out the output message */
    this->thrusterOnTimeOutMsg.write(&thrOnTimeOut, this->moduleID, callTime);

    return;
}
