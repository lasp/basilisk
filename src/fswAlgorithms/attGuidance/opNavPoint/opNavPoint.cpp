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

#include <string.h>
#include <math.h>
#include "fswAlgorithms/attGuidance/opNavPoint/opNavPoint.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void OpNavPoint::Reset(uint64_t callTime)
{
    double v1[3];

    // check if the required input messages are included
    if (!this->opnavDataInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: opNavPoint.opnavDataInMsg wasn't connected.");
    }
    if (!this->imuInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: opNavPoint.imuInMsg wasn't connected.");
    }
    if (!this->cameraConfigInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: opNavPoint.cameraConfigInMsg wasn't connected.");
    }

    /* compute an Eigen axis orthogonal to alignAxis_C */
    if (v3Norm(this->alignAxis_C)  < 0.1) {
        char info[MAX_LOGGING_LENGTH];
        snprintf(info, sizeof(info), "The module vector alignAxis_C is not setup as a unit vector [%f, %f %f]",
          this->alignAxis_C[0], this->alignAxis_C[1], this->alignAxis_C[2]);
        this->bskLogger.bskLog(BSK_ERROR, info);
    } else {
        v3Set(1., 0., 0., v1);
        v3Normalize(this->alignAxis_C, this->alignAxis_C);    /* ensure that this vector is a unit vector */
        v3Cross(this->alignAxis_C, v1, this->eHat180_B);
        if (v3Norm(this->eHat180_B) < 0.1) {
            v3Set(0., 1., 0., v1);
            v3Cross(this->alignAxis_C, v1, this->eHat180_B);
        }
        v3Normalize(this->eHat180_B, this->eHat180_B);
    }
    this->lastTime = 0;
    v3SetZero(this->currentHeading_N);
    this->attGuidanceOutBuffer = {};

    return;
}

/*! This method takes the estimated camera-observed target vector and computes the
 current attitude/attitude rate errors to pass on to control.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void OpNavPoint::UpdateState(uint64_t callTime)
{
    OpNavMsgPayload opNavMsg;
    double cthNormalized;
    double timeWithoutMeas;
    double currentHeading_C[3], alignAxis_B[3];
    double hNorm;                   /* Norm of measured direction vector */
    double e_hat[3];                /* Principal rotation Axis */
    double omega_BN_B[3];           /* r/s inertial body angular velocity vector in B frame components */
    double omega_RN_B[3];           /* r/s local copy of the desired reference frame rate */
    double dcm_BN[3][3], dcm_CB[3][3], dcm_CN[3][3];
    NavAttMsgPayload localImuDataInBuffer;
    CameraConfigMsgPayload cameraSpecs;

    /*! - Read the current target vector estimate*/
    opNavMsg = this->opnavDataInMsg();
    localImuDataInBuffer = this->imuInMsg();
    cameraSpecs = this->cameraConfigInMsg();

    if (this->lastTime==0){
        this->lastTime=callTime*1E-9;
        v3SetZero(this->currentHeading_N);
    }
    timeWithoutMeas = callTime*1E-9 - this->lastTime;

    v3Copy(localImuDataInBuffer.omega_BN_B, omega_BN_B);
    MRP2C(localImuDataInBuffer.sigma_BN, dcm_BN);
    MRP2C(cameraSpecs.sigma_CB, dcm_CB);
    m33MultM33(dcm_CB, dcm_BN, dcm_CN);
    /*! Compute the current error vector if it is valid. This checks for a valid, non-stale, previous message, or a new fresh measurement.*/
    if((opNavMsg.valid == 1 || v3IsZero(this->currentHeading_N, 1E-10) == 0) && (timeWithoutMeas < this->timeOut)){
        /*! - If a valid image is in save the heading direction for future use*/
        if (opNavMsg.valid == 1){
            this->lastTime = callTime*1E-9;
            v3Copy(opNavMsg.r_BN_C, currentHeading_C);
            m33tMultV3(dcm_CN, opNavMsg.r_BN_C, this->currentHeading_N);
            v3Scale(-1, currentHeading_C, currentHeading_C);
            hNorm = v3Norm(currentHeading_C);
            v3Scale(1/hNorm, currentHeading_C, currentHeading_C);
        }
        else{
            /*! - Else use the previous direction in order to continue guidance */
            m33MultV3(dcm_CN, this->currentHeading_N, currentHeading_C);
            v3Scale(-1, currentHeading_C, currentHeading_C);
            hNorm = v3Norm(currentHeading_C);
            v3Scale(1/hNorm, currentHeading_C, currentHeading_C);
        }
        cthNormalized = v3Dot(this->alignAxis_C, currentHeading_C);
        cthNormalized = fabs(cthNormalized) > 1.0 ?
        cthNormalized/fabs(cthNormalized) : cthNormalized;
        this->opNavAngleErr = safeAcos(cthNormalized);

        /*
            Compute the opNav error relative to the opNav direction vector
         */
        if (this->opNavAngleErr < this->smallAngle) {
            /* opNav heading and desired camera axis are essentially aligned.  Set attitude error to zero. */
             v3SetZero(this->attGuidanceOutBuffer.sigma_BR);
        } else {
            if (M_PI - this->opNavAngleErr < this->smallAngle) {
                /* the commanded camera vector nearly is opposite the opNav heading */
                v3Copy(this->eHat180_B, e_hat);
            } else {
                /* normal case where opNav and commanded camera vectors are not aligned */
                v3Cross(currentHeading_C, this->alignAxis_C, e_hat);
            }
            v3Normalize(e_hat, this->opNavMnvrVec);
            v3Scale(tan(this->opNavAngleErr*0.25), this->opNavMnvrVec,
                    this->attGuidanceOutBuffer.sigma_BR);
            MRPswitch(this->attGuidanceOutBuffer.sigma_BR, 1.0, this->attGuidanceOutBuffer.sigma_BR);
        }

        /* rate tracking error are the body rates to bring spacecraft to rest */
        m33tMultV3(dcm_CB, this->alignAxis_C, alignAxis_B);
        v3Scale(this->opNavAxisSpinRate, alignAxis_B, omega_RN_B);
        v3Subtract(omega_BN_B, omega_RN_B, this->attGuidanceOutBuffer.omega_BR_B);
        v3Copy(omega_RN_B, this->attGuidanceOutBuffer.omega_RN_B);

    } else {
        this->lastTime=0;
        /* no proper opNav direction vector is available */
        v3SetZero(this->attGuidanceOutBuffer.sigma_BR);

        /* specify a body-fixed constant search rotation rate */
        v3Subtract(omega_BN_B, this->omega_RN_B, this->attGuidanceOutBuffer.omega_BR_B);
        v3Copy(this->omega_RN_B, this->attGuidanceOutBuffer.omega_RN_B);
    }

    /* write the Guidance output message */
    this->attGuidanceOutMsg.write(&this->attGuidanceOutBuffer, this->moduleID, callTime);

    return;
}
