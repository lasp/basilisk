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
    Inertial 3D Spin Module

 */

/* modify the path to reflect the new module names */
#include "fswAlgorithms/attGuidance/inertial3D/inertial3D.h"




/* Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory. */
#include "architecture/utilities/linearAlgebra.h"

/*! Generate attitude reference associated with Intertial 3D Pointing.  In this case this is a fixed attitude
    with zero angular rate and acceleration vectors
 @return void
 @param attRefOut Output message
 */
static void computeInertialPointingReference(double sigma_R0N[3], AttRefMsgPayload *attRefOut)
{
    v3Copy(sigma_R0N, attRefOut->sigma_RN);
    v3SetZero(attRefOut->omega_RN_N);
    v3SetZero(attRefOut->domega_RN_N);
}

/*! This method creates a fixed attitude reference message.  The desired orientation is
    defined within the module.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Inertial3D::UpdateState(uint64_t callTime)
{
    AttRefMsgPayload attRefOut = {};         /* output message structure */

    /*! - Compute and store output message */
    computeInertialPointingReference(this->sigma_R0N, &attRefOut);

    /*! - Write output message */
    this->attRefOutMsg.write(&attRefOut, this->moduleID, callTime);

    return;
}
