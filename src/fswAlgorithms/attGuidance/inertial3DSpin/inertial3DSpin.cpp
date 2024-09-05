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

 * University of Colorado, Autonomous Vehicle Systems (AVS) Lab
 * Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved

 */

/* modify the path to reflect the new module names */
#include "fswAlgorithms/attGuidance/inertial3DSpin/inertial3DSpin.h"
#include "architecture/utilities/macroDefinitions.h"




/*
 Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory.
 */
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Inertial3DSpin::Reset(uint64_t callTime)
{

    this->priorTime = 0;              /* reset the prior time flag state.  If set
                                             to zero, the control time step is not evaluated on the
                                             first function call */
}

/*! This method performs all the main computations of the module
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Inertial3DSpin::UpdateState(uint64_t callTime)
{
    /*! - Read input message */
    AttRefMsgPayload attRefInMsgBuffer = {};

    if (this->attRefInMsg.isLinked()) {
        attRefInMsgBuffer = this->attRefInMsg();
    }

    /*! - Get input reference and compute integration time step to use downstream */
    double dt; /* integration time step [s] */
    if (this->priorTime == 0)
    {
        dt = 0.0;
        v3Copy(attRefInMsgBuffer.sigma_RN, this->sigma_RN);
    } else {
        dt = (callTime - this->priorTime) * NANO2SEC;
    }

    /*! - Generate inertial 3D Spinning Reference */
    this->attRefOutBuffer = {};
    this->computeReference_inertial3DSpin(attRefInMsgBuffer.omega_RN_N,
                                    attRefInMsgBuffer.domega_RN_N,
                                    dt);
    /*! - Write output message */
    this->attRefOutMsg.write(&this->attRefOutBuffer, this->moduleID, callTime);

    /*! Update prior time to current for next evaluation */
    this->priorTime = callTime;
}

void Inertial3DSpin::computeReference_inertial3DSpin(double omega_R0N_N[3],
                                                     double domega_R0N_N[3],
                                                     double dt)
{
    double omega_RN_N[3];
    double domega_RN_N[3];

    /*! Compute angular rate */
    double dcm_RN[3][3];   /* DCM from inertial frame N to generated ref frame R */
    double omega_RR0_N[3]; /* angular rate of the generated ref R wrt the base ref R0 in inertial N components */
    MRP2C(this->sigma_RN, dcm_RN);
    m33tMultV3(dcm_RN, this->omega_RR0_R0, omega_RR0_N);
    v3Add(omega_R0N_N, omega_RR0_N, omega_RN_N);

    /*! Compute angular acceleration */
    double v3Temp[3]; /* temporary 3x1 array */
    v3Cross(omega_R0N_N, omega_RR0_N, v3Temp);
    v3Add(v3Temp, domega_R0N_N, domega_RN_N);

    /*! Integrate Attitude */
    double B[3][3]; /* MRP rate matrix */
    double omega_RN_R[3]; /* inertial angular rate of ref R in R frame components */
    m33MultV3(dcm_RN, omega_RN_N, omega_RN_R);
    BmatMRP(this->sigma_RN, B);
    m33Scale(0.25 * dt, B, B);
    m33MultV3(B, omega_RN_R, v3Temp);
    v3Add(this->sigma_RN, v3Temp, this->sigma_RN);
    MRPswitch(this->sigma_RN, 1.0, this->sigma_RN);

    /*! Copy output in AttRefMsgPayload struct */
    v3Copy(this->sigma_RN, this->attRefOutBuffer.sigma_RN);
    v3Copy(omega_RN_N, this->attRefOutBuffer.omega_RN_N);
    v3Copy(domega_RN_N, this->attRefOutBuffer.domega_RN_N);
}
