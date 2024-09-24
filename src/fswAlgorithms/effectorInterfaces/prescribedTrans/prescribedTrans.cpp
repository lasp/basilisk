/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

/*! Import the module header file */
#include "prescribedTrans.h"

/*! Import other required files */
#include <math.h>
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values. This method also checks
 if the module input message is linked.
 @return void
 @param callTime [ns] Time the method is called
*/
void PrescribedTrans::Reset(uint64_t callTime)
{
    // Check if the input message is connected
    if (!this->linearTranslationRigidBodyInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: prescribedTrans.linearTranslationRigidBodyInMsg wasn't connected.");
    }

    // Set the initial time
    this->tInit = 0.0;

    // Set the initial convergence to true to enter the correct loop in the Update() method on the first pass
    this->convergence = true;
}

/*! This method uses the given initial and reference attitudes to compute the required attitude maneuver as
a function of time. The profiled translational trajectory is updated in time and written to the module's prescribed
motion output message.
 @return void
 @param callTime [ns] Time the method is called
*/
void PrescribedTrans::UpdateState(uint64_t callTime)
{
    // Create the buffer messages
    LinearTranslationRigidBodyMsgPayload linearTranslationRigidBodyIn = {};
    PrescribedTranslationMsgPayload prescribedTranslationOut = {};

    if (this->linearTranslationRigidBodyInMsg.isWritten())
    {
        linearTranslationRigidBodyIn = this->linearTranslationRigidBodyInMsg();
    }

    // This loop is entered when a new maneuver is requested after all previous maneuvers are completed
    if (this->linearTranslationRigidBodyInMsg.timeWritten() <= callTime && this->convergence)
    {
        // Store the initial information
        this->tInit = callTime * NANO2SEC;
        this->scalarPosInit = v3Norm(this->r_FM_M);
        this->scalarVelInit = v3Norm(this->rPrime_FM_M);

        // Store the reference information
        this->scalarPosRef = linearTranslationRigidBodyIn.rho;
        this->scalarVelRef = 0.0;

        // Define temporal information
        double convTime = sqrt(((0.5 * fabs(this->scalarPosRef - this->scalarPosInit)) * 8) / this->scalarAccelMax);
        this->tf = this->tInit + convTime;
        this->ts = this->tInit + convTime / 2;

        // Define the parabolic constants for the maneuver
        this->a = 0.5 * (this->scalarPosRef - this->scalarPosInit) / ((this->ts - this->tInit) * (this->ts - this->tInit));
        this->b = -0.5 * (this->scalarPosRef - this->scalarPosInit) / ((this->ts - this->tf) * (this->ts - this->tf));

        // Set the convergence to false to execute the maneuver
        this->convergence = false;
    }

    // Store the current simulation time
    double t = callTime * NANO2SEC;

    // Define scalar prescribed states
    double scalarAccel;
    double scalarVel;
    double scalarPos;

    // Compute the prescribed scalar states: scalarAccel, scalarVel, and scalarPos
    if ((t < this->ts || t == this->ts) && this->tf - this->tInit != 0)  // Entered during the first half of the maneuver
    {
        scalarAccel = this->scalarAccelMax;
        scalarVel = scalarAccel * (t - this->tInit) + this->scalarVelInit;
        scalarPos = this->a * (t - this->tInit) * (t - this->tInit) + this->scalarPosInit;
    }
    else if ( t > this->ts && t <= this->tf && this->tf - this->tInit != 0)  // Entered during the second half of the maneuver
    {
        scalarAccel = -1 * this->scalarAccelMax;
        scalarVel = scalarAccel * (t - this->tInit) + this->scalarVelInit - scalarAccel * (this->tf - this->tInit);
        scalarPos = this->b * (t - this->tf) * (t - this->tf) + this->scalarPosRef;
    }
    else  // Entered when the maneuver is complete
    {
        scalarAccel = 0.0;
        scalarVel = this->scalarVelRef;
        scalarPos = this->scalarPosRef;
        this->convergence = true;
    }

    // Convert the scalar variables to the prescribed parameters
    v3Scale(scalarPos, this->transAxis_M, this->r_FM_M);
    v3Scale(scalarVel, this->transAxis_M, this->rPrime_FM_M);
    v3Scale(scalarAccel, this->transAxis_M, this->rPrimePrime_FM_M);

    // Copy the local variables to the output message
    v3Copy(this->r_FM_M, prescribedTranslationOut.r_FM_M);
    v3Copy(this->rPrime_FM_M, prescribedTranslationOut.rPrime_FM_M);
    v3Copy(this->rPrimePrime_FM_M, prescribedTranslationOut.rPrimePrime_FM_M);

    // Write the prescribed motion output message
    this->prescribedTranslationOutMsg.write(&prescribedTranslationOut, this->moduleID, callTime);
}
