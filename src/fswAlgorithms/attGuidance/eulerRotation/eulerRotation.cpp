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
 Euler Angle Rotation Guidance Module with Constant Euler Rates

 * University of Colorado, Autonomous Vehicle Systems (AVS) Lab
 * Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved

 */

#include "fswAlgorithms/attGuidance/eulerRotation/eulerRotation.h"
#include <math.h>
#include "architecture/utilities/macroDefinitions.h"

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"

/*! @brief This function computes the analytical derivative of the B_inv matrix for the 3-2-1 Euler Angle set.
 @return void
 @param angleSet 321 Euler angles
 @param angleRates The 321 Euler angle rates
 @param B_inv_deriv the inv(B) matrix for 321 Euler angles
 */
static void computeEuler321_Binv_derivative(double angleSet[3], double angleRates[3], double B_inv_deriv[3][3])
{
    double s2;
    double c2;
    double s3;
    double c3;

    s2 = sin(angleSet[1]);
    c2 = cos(angleSet[1]);
    s3 = sin(angleSet[2]);
    c3 = cos(angleSet[2]);


    B_inv_deriv[0][0] = -angleRates[1] * c2;
    B_inv_deriv[0][1] = 0;
    B_inv_deriv[0][2] = 0;
    B_inv_deriv[1][0] = angleRates[2] * c3 * c2 - angleRates[1] * s3 * s2;
    B_inv_deriv[1][1] = -angleRates[2] * s3;
    B_inv_deriv[1][2] = 0;
    B_inv_deriv[2][0] = -angleRates[2] * s3 * c2 - angleRates[1] * c3 * c2;
    B_inv_deriv[2][1] = -angleRates[2] * c3;
    B_inv_deriv[2][2] = 0;
}

/*! @brief This resets the module to original states.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void EulerRotation::Reset(uint64_t callTime)
{
    // check if the required input message is included
    if (!this->attRefInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: eulerRotation.attRefInMsg wasn't connected.");
    }

    this->priorTime = 0;
    v3SetZero(this->priorCmdSet);
    v3SetZero(this->priorCmdRates);
}


/*! @brief This method takes the input attitude reference frame, and and superimposes the dynamic euler angle
 scanning motion on top of this.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void EulerRotation::UpdateState(uint64_t callTime)
{
    AttRefMsgPayload inputRef;
    AttRefMsgPayload attRefOut = {};

    /* - Read input messages */
    inputRef = this->attRefInMsg();

    if (this->desiredAttInMsg.isLinked())
    {
        AttStateMsgPayload attStates;

        /* - Read Raster Manager messages */
        attStates = this->desiredAttInMsg();
        /* - Save commanded 321 Euler set and rates */
        v3Copy(attStates.state, this->cmdSet);
        v3Copy(attStates.rate, this->cmdRates);
        /* - Check the command is new */
        this->checkRasterCommands();
    }

    /* - Compute time step to use in the integration downstream */
    this->computeTimeStep(callTime);
    /* - Compute output reference frame */
    this->computeEulerRotationReference(inputRef.sigma_RN,
                                        inputRef.omega_RN_N,
                                        inputRef.domega_RN_N,
                                        &attRefOut);

    /* - Write output messages */
    this->attRefOutMsg.write(&attRefOut, this->moduleID, callTime);

    /* - Update last time the module was called to current call time */
    this->priorTime = callTime;
    return;
}



/*! @brief This function checks if there is a new commanded raster maneuver message available
 @return void
 @param this The configuration data associated with the mrpRotation module
 */
void EulerRotation::checkRasterCommands()
{
    int32_t prevCmdActive = v3IsEqual(this->cmdSet, this->priorCmdSet , 1E-12)
                            && v3IsEqual(this->cmdRates, this->priorCmdRates , 1E-12);
    if (!prevCmdActive)
    {
        v3Copy(this->cmdSet, this->angleSet);
        v3Copy(this->cmdRates, this->angleRates);

        v3Copy(this->cmdSet, this->priorCmdSet);
        v3Copy(this->cmdRates, this->priorCmdRates);
    }
}

/*! @brief This function computes control update time
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void EulerRotation::computeTimeStep(uint64_t callTime)
{
    if (this->priorTime == 0)
    {
        this->dt = 0.0;
    } else {
        this->dt = (callTime - this->priorTime)*NANO2SEC;
    }
}

/*! @brief This function computes the reference (Euler angle attitude set, angular velocity and angular acceleration)
 associated with a rotation defined in terms of an initial euler angle set and a constant euler angle rate
 @return void
 @param this The configuration data associated with the mrpRotation module
 @param sigma_R0N The input reference attitude using MRPs
 @param omega_R0N_N The input reference frame angular rate vector
 @param domega_R0N_N The input reference frame angular acceleration vector
 @param attRefOut The output message copy
 */
void EulerRotation::computeEulerRotationReference(double sigma_R0N[3],
                                                  double omega_R0N_N[3],
                                                  double domega_R0N_N[3],
                                                  AttRefMsgPayload *attRefOut)
{
    /* Compute attitude reference*/
    double attIncrement[3];         /*!< [] increment in attitude coordinates  */
    double RR0[3][3];               /*!< [] DCM rotating from R0 to R */
    double R0N[3][3];               /*!< [] DCM rotating from N to R0 */
    double RN[3][3];                /*!< [] DCM rotating from N to R */

    MRP2C(sigma_R0N, R0N);
    v3Scale(this->dt, this->angleRates, attIncrement);
    v3Add(this->angleSet, attIncrement, this->angleSet);
    Euler3212C(this->angleSet, RR0);
    m33MultM33(RR0, R0N, RN);
    C2MRP(RN, attRefOut->sigma_RN);

    /* Compute angular velocity */
    double B_inv[3][3];             /*!< [] matrix related Euler angle rates to angular velocity vector components */
    double omega_RR0_R[3];          /*!< [r/s] angular velocity vector between R and R0 frame in R frame components */
    double omega_RR0_N[3];          /*!< [r/s] angular velocity vector between R and R0 frame in N frame components */
    BinvEuler321(this->angleSet, B_inv);
    m33MultV3(B_inv, this->angleRates, omega_RR0_R);
    m33tMultV3(RN, omega_RR0_R, omega_RR0_N);
    v3Add(omega_R0N_N, omega_RR0_N, attRefOut->omega_RN_N);

    /* Compute angular acceleration */
    double B_inv_deriv[3][3];       /*!< [] time derivatie of matrix relating EA rates to omegas */
    double domega_RR0_R[3];         /*!< [r/s] inertial derivative of omega_RR0_R in R frame components */
    double domega_RR0_N[3];         /*!< [r/s] inertial derivative of omega_RR0_R in N frame components */
    double temp[3];
    computeEuler321_Binv_derivative(this->angleSet, this->angleRates, B_inv_deriv);
    m33MultV3(B_inv_deriv, this->angleRates, domega_RR0_R);
    m33tMultV3(RN, domega_RR0_R, domega_RR0_N);
    v3Cross(omega_R0N_N, omega_RR0_N, temp);
    v3Add(temp, domega_RR0_N, domega_RR0_N);
    v3Add(domega_RR0_N, domega_R0N_N, attRefOut->domega_RN_N);
}
