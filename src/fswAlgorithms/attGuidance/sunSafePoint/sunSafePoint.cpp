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

#include <math.h>
#include "fswAlgorithms/attGuidance/sunSafePoint/sunSafePoint.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SunSafePoint::Reset(uint64_t callTime)
{
    double v1[3];

    // check if the required input messages are included
    if (!this->sunDirectionInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: sunSafePoint.sunDirectionInMsg wasn't connected.");
    }
    if (!this->imuInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: sunSafePoint.imuInMsg wasn't connected.");
    }

    /* compute an Eigen axis orthogonal to sHatBdyCmd */
    if (v3Norm(this->sHatBdyCmd)  < 0.1) {
      char info[MAX_LOGGING_LENGTH];
      snprintf(info, sizeof(info), "The module vector sHatBdyCmd is not setup as a unit vector [%f, %f %f]",
                this->sHatBdyCmd[0], this->sHatBdyCmd[1], this->sHatBdyCmd[2]);
      this->bskLogger.bskLog(BSK_ERROR, info);
    } else {
        v3Set(1., 0., 0., v1);
        v3Normalize(this->sHatBdyCmd, this->sHatBdyCmd);    /* ensure that this vector is a unit vector */
        v3Cross(this->sHatBdyCmd, v1, this->eHat180_B);
        if (v3Norm(this->eHat180_B) < 0.1) {
            v3Set(0., 1., 0., v1);
            v3Cross(this->sHatBdyCmd, v1, this->eHat180_B);
        }
        v3Normalize(this->eHat180_B, this->eHat180_B);
    }

    this->attGuidanceOutBuffer = {};

    return;
}

/*! This method takes the estimated body-observed sun vector and computes the
 current attitude/attitude rate errors to pass on to control.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void SunSafePoint::UpdateState(uint64_t callTime)
{
    NavAttMsgPayload navMsg;
    double ctSNormalized;
    double sNorm;                   /*!< --- Norm of measured direction vector */
    double e_hat[3];                /*!< --- Eigen Axis */
    double omega_BN_B[3];           /*!< r/s inertial body angular velocity vector in B frame components */
    double omega_RN_B[3];           /*!< r/s local copy of the desired reference frame rate */

    NavAttMsgPayload localImuDataInBuffer;
    this->attGuidanceOutBuffer = {};

    /*! - Read the current sun body vector estimate*/
    navMsg = this->sunDirectionInMsg();
    localImuDataInBuffer = this->imuInMsg();

    v3Copy(localImuDataInBuffer.omega_BN_B, omega_BN_B);

    /*! - Compute the current error vector if it is valid*/
    sNorm = v3Norm(navMsg.vehSunPntBdy);
    if(sNorm > this->minUnitMag)
    {
        /* a good sun direction vector is available */
        ctSNormalized = v3Dot(this->sHatBdyCmd, navMsg.vehSunPntBdy)/sNorm;
        ctSNormalized = fabs(ctSNormalized) > 1.0 ?
        ctSNormalized/fabs(ctSNormalized) : ctSNormalized;
        this->sunAngleErr = safeAcos(ctSNormalized);

        /*
            Compute the heading error relative to the sun direction vector
         */
        if (this->sunAngleErr < this->smallAngle) {
            /* sun heading and desired body axis are essentially aligned.  Set attitude error to zero. */
             v3SetZero(this->attGuidanceOutBuffer.sigma_BR);
        } else {
            if (M_PI - this->sunAngleErr < this->smallAngle) {
                /* the commanded body vector nearly is opposite the sun heading */
                v3Copy(this->eHat180_B, e_hat);
            } else {
                /* normal case where sun and commanded body vectors are not aligned */
                v3Cross(navMsg.vehSunPntBdy, this->sHatBdyCmd, e_hat);
            }
            v3Normalize(e_hat, this->sunMnvrVec);
            v3Scale(tan(this->sunAngleErr*0.25), this->sunMnvrVec,
                    this->attGuidanceOutBuffer.sigma_BR);
            MRPswitch(this->attGuidanceOutBuffer.sigma_BR, 1.0, this->attGuidanceOutBuffer.sigma_BR);
        }

        /* rate tracking error are the body rates to bring spacecraft to rest */
        v3Scale(this->sunAxisSpinRate/sNorm, navMsg.vehSunPntBdy, omega_RN_B);
        v3Subtract(omega_BN_B, omega_RN_B, this->attGuidanceOutBuffer.omega_BR_B);
        v3Copy(omega_RN_B, this->attGuidanceOutBuffer.omega_RN_B);

    } else {
        /* no proper sun direction vector is available */
        v3SetZero(this->attGuidanceOutBuffer.sigma_BR);

        /* specify a body-fixed constant search rotation rate */
        v3Subtract(omega_BN_B, this->omega_RN_B, this->attGuidanceOutBuffer.omega_BR_B);
        v3Copy(this->omega_RN_B, this->attGuidanceOutBuffer.omega_RN_B);
    }

    /* write the Guidance output message */
    this->attGuidanceOutMsg.write(&this->attGuidanceOutBuffer, this->moduleID, callTime);

    return;
}
