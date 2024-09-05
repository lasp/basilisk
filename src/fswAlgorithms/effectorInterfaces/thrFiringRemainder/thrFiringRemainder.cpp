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
    Thrust Firing Remainder

 */

#include "fswAlgorithms/effectorInterfaces/thrFiringRemainder/thrFiringRemainder.h"
#include "architecture/utilities/macroDefinitions.h"


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringRemainder::Reset(uint64_t callTime)
{
	THRArrayConfigMsgPayload   localThrusterData;     /* local copy of the thruster data message */
	int 				i;

	this->prevCallTime = 0;

	// check if the required input messages are included
	if (!this->thrConfInMsg.isLinked()) {
		this->bskLogger.bskLog(BSK_ERROR, "Error: thrFiringRemainder.thrConfInMsg wasn't connected.");
	}
    if (!this->thrForceInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrFiringRemainder.thrForceInMsg wasn't connected.");
    }

	/*! - read in the support messages */
    localThrusterData = this->thrConfInMsg();

    /*! - store the number of installed thrusters */
	this->numThrusters = localThrusterData.numThrusters;

    /*! - loop over all thrusters and for each copy over maximum thrust, zero the impulse remainder */
	for(i=0; i<this->numThrusters; i++) {
		this->maxThrust[i] = localThrusterData.thrusters[i].maxThrust;
		this->pulseRemainder[i] = 0.0;
	}

    /*! - use default value of 2 seconds for control period of first call if not specified.
     * Control period (FSW rate) is computed dynamically for any subsequent calls.
     */
    this->defaultControlPeriod = (0.0 == this->defaultControlPeriod) ?
                                        2.0 : this->defaultControlPeriod;
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringRemainder::UpdateState(uint64_t callTime)
{
	int 				i;
	double				controlPeriod;			/* [s] control period */
	double				onTime[MAX_EFF_CNT];	/* [s] array of commanded on time for thrusters */
    THRArrayCmdForceMsgPayload thrForceIn;          /* [-] copy of the thruster force input message */
    THRArrayOnTimeCmdMsgPayload thrOnTimeOut = {};       /* [-] copy of the thruster on-time output message */

    /*! - The first time update() is called there is no information on the time step.
     *    Pick 2 seconds for the control period */
	if(this->prevCallTime == 0) {
        controlPeriod = this->defaultControlPeriod;
	} else {
        /*! - compute control time period Delta_t */
        controlPeriod = ((double)(callTime - this->prevCallTime)) * NANO2SEC;
    }

	this->prevCallTime = callTime;

	/*! - Read the input thruster force message */
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
		/*! - Add in remainder from the last control step */
		onTime[i] += this->pulseRemainder[i]*this->thrMinFireTime;
		/*! - Set pulse remainder to zero. Remainder now stored in onTime */
		this->pulseRemainder[i] = 0.0;

		/* Pulse remainder logic */
		if(onTime[i] < this->thrMinFireTime) {
			/*! - If request is less than minimum pulse time zero onTime an store remainder */
			this->pulseRemainder[i] = onTime[i]/this->thrMinFireTime;
			onTime[i] = 0.0;
		} else if (onTime[i] >= controlPeriod) {
			/*! - If request is greater than control period then oversaturate onTime */
			onTime[i] = 1.1*controlPeriod;
		}

		/*! - Set the output data for each thruster */
		thrOnTimeOut.OnTimeRequest[i] = onTime[i];

	}

    /*! - write the moduel output message */
    this->onTimeOutMsg.write(&thrOnTimeOut, this->moduleID, callTime);

	return;

}
