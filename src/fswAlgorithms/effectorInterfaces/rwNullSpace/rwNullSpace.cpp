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

#include "fswAlgorithms/effectorInterfaces/rwNullSpace/rwNullSpace.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"

/*! @brief This resets the module to original states by reading in the RW configuration messages and recreating any module specific variables.  The output message is reset to zero.
    @return void
    @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwNullSpace::Reset(uint64_t callTime)
{
    double GsMatrix[3*MAX_EFF_CNT];                 /* [-]  [Gs] projection matrix where gs_hat_B RW spin axis form each colum */
    double GsTranspose[3 * MAX_EFF_CNT];            /* [-]  [Gs]^T */
    double GsInvHalf[3 * 3];                        /* [-]  ([Gs][Gs]^T)^-1 */
    double identMatrix[MAX_EFF_CNT*MAX_EFF_CNT];    /* [-]  [I_NxN] identity matrix */
    double GsTemp[MAX_EFF_CNT*MAX_EFF_CNT];         /* [-]  temp matrix */
    RWConstellationMsgPayload localRWData;          /*      local copy of RW configuration data */

    // check if the required input messages are included
    if (!this->rwConfigInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: rwNullSpace.rwConfigInMsg wasn't connected.");
    }
    if (!this->rwMotorTorqueInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: rwNullSpace.rwMotorTorqueInMsg wasn't connected.");
    }
    if (!this->rwSpeedsInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: rwNullSpace.rwSpeedsInMsg wasn't connected.");
    }

    /* read in the RW spin axis headings */
    localRWData = this->rwConfigInMsg();

    /* create the 3xN [Gs] RW spin axis projection matrix */
    this->numWheels = (uint32_t) localRWData.numRW;
    if (this->numWheels > MAX_EFF_CNT) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: rwNullSpace.numWheels is larger that max effector count.");
    }
    for(uint32_t i=0; i<this->numWheels; i=i+1)
    {
        for(int j=0; j<3; j=j+1)
        {
            GsMatrix[j*(int) this->numWheels+i] = localRWData.reactionWheels[i].gsHat_B[j];
        }
    }

    /* find the [tau] null space projection matrix [tau]= ([I] - [Gs]^T.([Gs].[Gs]^T) */
    mTranspose(GsMatrix, 3, this->numWheels, GsTranspose);            /* find [Gs]^T */
    mMultM(GsMatrix, 3, this->numWheels, GsTranspose,
           this->numWheels, 3, GsInvHalf);                            /* find [Gs].[Gs]^T */
    m33Inverse(RECAST3X3 GsInvHalf, RECAST3X3 GsInvHalf);                   /* find ([Gs].[Gs]^T)^-1 */
    mMultM(GsInvHalf, 3, 3, GsMatrix, 3, this->numWheels,
           this->tau);                                                /* find ([Gs].[Gs]^T)^-1.[Gs] */
    mMultM(GsTranspose, this->numWheels, 3, this->tau, 3,
           this->numWheels, GsTemp);                                  /* find [Gs]^T.([Gs].[Gs]^T)^-1.[Gs] */
    mSetIdentity(identMatrix, this->numWheels, this->numWheels);
    mSubtract(identMatrix, this->numWheels, this->numWheels,    /* find ([I] - [Gs]^T.([Gs].[Gs]^T)^-1.[Gs]) */
              GsTemp, this->tau);

}

/*! This method takes the input reaction wheel commands as well as the observed
    reaction wheel speeds and balances the commands so that the overall vehicle
	momentum is minimized.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwNullSpace::UpdateState(uint64_t callTime)
{
    ArrayMotorTorqueMsgPayload cntrRequest;        /* [Nm]  array of the RW motor torque solution vector from the control module */
    RWSpeedMsgPayload rwSpeeds;                    /* [r/s] array of RW speeds */
    RWSpeedMsgPayload rwDesiredSpeeds = {};             /* [r/s] array of RW speeds */
	ArrayMotorTorqueMsgPayload finalControl = {};       /* [Nm]  array of final RW motor torques containing both
                                                       the control and null motion torques */
	double dVector[MAX_EFF_CNT];                   /* [Nm]  null motion wheel speed control array */
    double DeltaOmega[MAX_EFF_CNT];                /* [r/s] difference in RW speeds */

    /* Read the input RW commands to get the raw RW requests*/
    cntrRequest = this->rwMotorTorqueInMsg();
    /* Read the RW speeds*/
    rwSpeeds = this->rwSpeedsInMsg();

    if (this->rwDesiredSpeedsInMsg.isLinked()) {
        rwDesiredSpeeds = this->rwDesiredSpeedsInMsg();
    }

    /* compute the wheel speed control vector d = -K.DeltaOmega */
    vSubtract(rwSpeeds.wheelSpeeds, this->numWheels, rwDesiredSpeeds.wheelSpeeds, DeltaOmega);
	vScale(-this->OmegaGain, DeltaOmega, this->numWheels, dVector);

    /* compute the RW null space motor torque solution to reduce the wheel speeds */
	mMultV(this->tau, this->numWheels, this->numWheels,
		dVector, finalControl.motorTorque);

    /* add the null motion RW torque solution to the RW feedback control torque solution */
	vAdd(finalControl.motorTorque, this->numWheels,
		cntrRequest.motorTorque, finalControl.motorTorque);

    /* write the final RW torque solution to the output message */
    this->rwMotorTorqueOutMsg.write(&finalControl, this->moduleID, callTime);

    return;
}
