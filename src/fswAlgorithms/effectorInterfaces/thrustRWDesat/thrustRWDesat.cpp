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

#include "fswAlgorithms/effectorInterfaces/thrustRWDesat/thrustRWDesat.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <string.h>

void ThrustRWDesat::Reset(uint64_t callTime)
{
    RWConstellationMsgPayload localRWData;
    THRArrayConfigMsgPayload localThrustData;
    VehicleConfigMsgPayload localConfigData;
    int i;
    double momentArm[3];
    double thrustDat_B[3];

	// check if the required input messages are included
	if (!this->rwConfigInMsg.isLinked()) {
		this->bskLogger.bskLog(BSK_ERROR, "Error: thrustRWDesat.rwConfigInMsg wasn't connected.");
	}
	if (!this->vecConfigInMsg.isLinked()) {
		this->bskLogger.bskLog(BSK_ERROR, "Error: thrustRWDesat.vecConfigInMsg wasn't connected.");
	}
	if (!this->thrConfigInMsg.isLinked()) {
		this->bskLogger.bskLog(BSK_ERROR, "Error: thrustRWDesat.thrConfigInMsg wasn't connected.");
	}
    if (!this->rwSpeedInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrustRWDesat.rwSpeedInMsg wasn't connected.");
    }

    /*! - Read input messages */
    localRWData = this->rwConfigInMsg();
    localConfigData = this->vecConfigInMsg();
    localThrustData = this->thrConfigInMsg();

    /*! - Transform from structure S to body B frame */
    this->numRWAs = localRWData.numRW;
    for(i=0; i<this->numRWAs; i=i+1)
    {
        v3Copy(localRWData.reactionWheels[i].gsHat_B, &this->rwAlignMap[i*3]);
    }

    this->numThrusters = localThrustData.numThrusters;
    for(i=0; i<this->numThrusters; i=i+1)
    {
        v3Copy(localThrustData.thrusters[i].tHatThrust_B, &this->thrAlignMap[i*3]);
        v3Copy(localThrustData.thrusters[i].rThrust_B, thrustDat_B);
        v3Subtract(thrustDat_B, localConfigData.CoM_B, momentArm);
        v3Copy(localThrustData.thrusters[i].tHatThrust_B, thrustDat_B);
        v3Cross(momentArm, thrustDat_B, &(this->thrTorqueMap[i*3]));
    }

    this->previousFiring = 0;
    v3SetZero(this->accumulatedImp);
    this->totalAccumFiring = 0.0;

}

/*! This method takes in the current oberved reaction wheel angular velocities.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrustRWDesat::UpdateState(uint64_t callTime)
{
    int32_t i;
	int32_t selectedThruster;     /* Thruster index to fire */
    RWSpeedMsgPayload rwSpeeds;   /* Local reaction wheel speeds */
	double observedSpeedVec[3];   /* The total summed speed of RWAs*/
	double singleSpeedVec[3];     /* The speed vector for a single wheel*/
	double bestMatch;             /* The current best thruster/wheel matching*/
	double currentMatch;          /* Assessment of the current match */
    double fireValue;             /* Amount of time to fire the jet for */
	THRArrayOnTimeCmdMsgPayload outputData = {};    /* Local output firings */

    /*! - If we haven't met the cooldown threshold, do nothing */
	if ((callTime - this->previousFiring)*1.0E-9 <
		this->thrFiringPeriod)
	{
		return;
	}

    /*! - Read the input rwheel speeds from the reaction wheels*/
    rwSpeeds = this->rwSpeedInMsg();

    /*! - Accumulate the total momentum vector we want to apply (subtract speed vectors)*/
	v3SetZero(observedSpeedVec);
	for (i = 0; i < this->numRWAs; i++)
	{
		v3Scale(rwSpeeds.wheelSpeeds[i], &(this->rwAlignMap[i * 3]),
			singleSpeedVec);
		v3Subtract(observedSpeedVec, singleSpeedVec, observedSpeedVec);
	}

	/*! - If we are within the specified threshold for the momentum, stop desaturation.*/
	if (v3Norm(observedSpeedVec) < this->DMThresh)
	{
		return;
	}

    /*! - Iterate through the list of thrusters and find the "best" match for the
          observed momentum vector that does not continue to perturb the velocity
          in the same direction as previous aggregate firings.  Only do this once we have
		  removed the specified momentum accuracy from the current direction.*/
	selectedThruster = -1;
	bestMatch = 0.0;
	if (v3Dot(this->currDMDir, observedSpeedVec) <= this->DMThresh)
	{
		for (i = 0; i < this->numThrusters; i++)
		{

			fireValue = v3Dot(observedSpeedVec,
				&(this->thrTorqueMap[i * 3]));
			currentMatch = v3Dot(this->accumulatedImp,
				&(this->thrAlignMap[i * 3]));
			if (fireValue - currentMatch > bestMatch && fireValue > 0.0)
			{
				selectedThruster = i;
				bestMatch = fireValue - currentMatch;
			}
		}
		if (selectedThruster >= 0)
		{
			v3Normalize(&this->thrTorqueMap[selectedThruster * 3],
				this->currDMDir);
		}
	}

    /*! - Zero out the thruster commands prior to setting the selected thruster.
          Only apply thruster firing if the best match is non-zero.  Find the thruster
		  that best matches the current specified direction.
    */
	selectedThruster = -1;
	bestMatch = 0.0;
	for (i = 0; i < this->numThrusters; i++)
	{

		fireValue = v3Dot(this->currDMDir,
			&(this->thrTorqueMap[i * 3]));
		currentMatch = v3Dot(this->accumulatedImp,
			&(this->thrAlignMap[i * 3]));
		if (fireValue - currentMatch > bestMatch && fireValue > 0.0)
		{
			selectedThruster = i;
			bestMatch = fireValue - currentMatch;
		}
	}
    /*! - If we have a valid match:
          - Set firing based on the best counter to the observed momentum.
          - Saturate based on the maximum allowable firing
          - Accumulate impulse and the total firing
          - Set the previous call time value for cooldown check */
	if (bestMatch > 0.0)
	{
		outputData.OnTimeRequest[selectedThruster] = v3Dot(this->currDMDir,
			&(this->thrTorqueMap[selectedThruster * 3]));
		outputData.OnTimeRequest[selectedThruster] =
			outputData.OnTimeRequest[selectedThruster] > this->maxFiring ?
			this->maxFiring : outputData.OnTimeRequest[selectedThruster];
		this->previousFiring = callTime;
		this->totalAccumFiring += outputData.OnTimeRequest[selectedThruster];
        v3Scale(outputData.OnTimeRequest[selectedThruster],
                &(this->thrAlignMap[selectedThruster * 3]), singleSpeedVec);
        v3Add(this->accumulatedImp, singleSpeedVec,
            this->accumulatedImp);
	}

    /*! - Write the output message to the thruster system */
    this->thrCmdOutMsg.write(&outputData, this->moduleID, callTime);

    return;
}
