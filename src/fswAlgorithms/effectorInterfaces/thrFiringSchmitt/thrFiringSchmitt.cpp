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
    Thrust Firing Schmitt

 */

#include "fswAlgorithms/effectorInterfaces/thrFiringSchmitt/thrFiringSchmitt.h"
#include "architecture/utilities/macroDefinitions.h"
#include <string.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringSchmitt::Reset(uint64_t callTime)
{
	THRArrayConfigMsgPayload   localThrusterData;     /* local copy of the thruster data message */
	int 				i;

	this->prevCallTime = 0;

	// check if the required input messages are included
	if (!this->thrConfInMsg.isLinked()) {
		this->bskLogger.bskLog(BSK_ERROR, "Error: thrFiringSchmitt.thrConfInMsg wasn't connected.");
	}
    if (!this->thrForceInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrFiringSchmitt.thrForceInMsg wasn't connected.");
    }

	/*! - Zero and read in the support messages */
    localThrusterData = this->thrConfInMsg();

    /*! - store the number of installed thrusters */
	this->numThrusters = localThrusterData.numThrusters;

    /*! - loop over all thrusters and for each copy over maximum thrust, set last state to off */
	for(i=0; i<this->numThrusters; i++) {
		this->maxThrust[i] = localThrusterData.thrusters[i].maxThrust;
		this->lastThrustState[i] = BOOL_FALSE;
	}
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringSchmitt::UpdateState(uint64_t callTime)
{
	int 				i;
	double 				level;					/* [-] duty cycle fraction */
	double				controlPeriod;			/* [s] control period */
	double				onTime[MAX_EFF_CNT];	/* [s] array of commanded on time for thrusters */
    THRArrayCmdForceMsgPayload thrForceIn;          /* -- copy of the thruster force input message */
    THRArrayOnTimeCmdMsgPayload thrOnTimeOut = {};       /* -- copy of the thruster on-time output message */

    /*! - the first time update() is called there is no information on the time step.  Here
     return either all thrusters off or on depending on the baseThrustState state */
	if(this->prevCallTime == 0) {
		this->prevCallTime = callTime;

		for(i = 0; i < this->numThrusters; i++) {
			thrOnTimeOut.OnTimeRequest[i] = (double)(this->baseThrustState) * 2.0;
		}

        this->onTimeOutMsg.write(&thrOnTimeOut, this->moduleID, callTime);
		return;
	}

    /*! - compute control time period Delta_t */
	controlPeriod = ((double)(callTime - this->prevCallTime)) * NANO2SEC;
	this->prevCallTime = callTime;

    /*! - read the input thruster force message */
    thrForceIn = this->thrForceInMsg();

    /*! - Loop through thrusters */
	for(i = 0; i < this->numThrusters; i++) {

        /*! - Correct for off-pulsing if necessary.  Here the requested force is negative, and the maximum thrust
         needs to be added.  If not control force is requested in off-pulsing mode, then the thruster force should
         be set to the maximum thrust value */
		if (this->baseThrustState == 1) {
			thrForceIn.thrForce[i] += this->maxThrust[i];
		}

        /*! - Do not allow thrust requests less than zero */
		if (thrForceIn.thrForce[i] < 0.0) {
			thrForceIn.thrForce[i] = 0.0;
		}
        /*! - Compute T_on from thrust request, max thrust, and control period */
		onTime[i] = thrForceIn.thrForce[i]/this->maxThrust[i]*controlPeriod;

        /*! - Apply Schmitt trigger logic */
		if (onTime[i] < this->thrMinFireTime) {
			/*! - Request is less than minimum fire time */
			level = onTime[i]/this->thrMinFireTime;
			if (level >= this->level_on) {
				this->lastThrustState[i] = BOOL_TRUE;
				onTime[i] = this->thrMinFireTime;
			} else if (level <= this->level_off) {
				this->lastThrustState[i] = BOOL_FALSE;
				onTime[i] = 0.0;
			} else if (this->lastThrustState[i] == BOOL_TRUE) {
				onTime[i] = this->thrMinFireTime;
			} else {
				onTime[i] = 0.0;
			}
		} else if (onTime[i] >= controlPeriod) {
            /*! - Request is greater than control period then oversaturate onTime */
			this->lastThrustState[i] = BOOL_TRUE;
			onTime[i] = 1.1*controlPeriod; // oversaturate to avoid numerical error
		} else {
			/*! - Request is greater than minimum fire time and less than control period */
			this->lastThrustState[i] = BOOL_TRUE;
		}

		/*! Set the output data */
		thrOnTimeOut.OnTimeRequest[i] = onTime[i];
	}

    this->onTimeOutMsg.write(&thrOnTimeOut, this->moduleID, callTime);

	return;

}
