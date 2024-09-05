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


#include "solarArrayReference.h"
#include "string.h"
#include <math.h>

#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/macroDefinitions.h"

const double epsilon = 1e-12;                           // module tolerance for zero

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime [ns] time the method is called
*/
void SolarArrayReference::Reset(uint64_t callTime)
{
    if (!this->attNavInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "solarArrayReference.attNavInMsg wasn't connected.");
    }
    if (!this->attRefInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "solarArrayReference.attRefInMsg wasn't connected.");
    }
    if (!this->hingedRigidBodyInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "solarArrayReference.hingedRigidBodyInMsg wasn't connected.");
    }
    this->count = 0;
}

/*! This method computes the updated rotation angle reference based on current attitude, reference attitude, and current rotation angle
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void SolarArrayReference::UpdateState(uint64_t callTime)
{
     /*! - Create and assign buffer messages */
    NavAttMsgPayload            attNavIn = this->attNavInMsg();
    AttRefMsgPayload            attRefIn = this->attRefInMsg();
    HingedRigidBodyMsgPayload   hingedRigidBodyIn     = this->hingedRigidBodyInMsg();
    HingedRigidBodyMsgPayload   hingedRigidBodyRefOut = {};

    /*! read Sun direction in B frame from the attNav message and map it to R frame */
    double rHat_SB_B[3];    // Sun direction in body-frame coordinates
    double rHat_SB_R[3];    // Sun direction in reference-frame coordinates
    double BN[3][3];   // inertial to body frame DCM
    double RN[3][3];   // inertial to reference frame DCM
    double RB[3][3];   // body to reference DCM
    v3Normalize(attNavIn.vehSunPntBdy, rHat_SB_B);
    switch (this->attitudeFrame) {

        case referenceFrame:
            MRP2C(attNavIn.sigma_BN, BN);
            MRP2C(attRefIn.sigma_RN, RN);
            m33MultM33t(RN, BN, RB);
            m33MultV3(RB, rHat_SB_B, rHat_SB_R);
            break;

        case bodyFrame:
            v3Copy(rHat_SB_B, rHat_SB_R);
            break;

        default:
            this->bskLogger.bskLog(BSK_ERROR, "solarArrayAngle.bodyFrame input can be either 0 or 1.");
    }

    /*! compute solar array frame axes at zero rotation */
    double a1Hat_B[3];      // solar array axis drive
    double a2Hat_B[3];      // solar array axis surface normal
    double a3Hat_B[3];      // third axis according to right-hand rule
    v3Normalize(this->a1Hat_B, a1Hat_B);
    v3Cross(a1Hat_B, this->a2Hat_B, a3Hat_B);
    v3Normalize(a3Hat_B, a3Hat_B);
    v3Cross(a3Hat_B, a1Hat_B, a2Hat_B);

    /*! compute solar array reference frame axes at zero rotation */
    double a1Hat_R[3];
    double a2Hat_R[3];
    double dotP = v3Dot(a1Hat_B, rHat_SB_R);
    for (int n = 0; n < 3; n++) {
        a2Hat_R[n] = rHat_SB_R[n] - dotP * a1Hat_B[n];
    }
    v3Normalize(a2Hat_R, a2Hat_R);
    v3Cross(a2Hat_B, a2Hat_R, a1Hat_R);

    /*! compute current rotation angle thetaC from input msg */
    double sinThetaC = sin(hingedRigidBodyIn.theta);
    double cosThetaC = cos(hingedRigidBodyIn.theta);
    double thetaC = atan2(sinThetaC, cosThetaC);      // clip theta current between 0 and 2*pi

    /*! compute reference angle and store in buffer msg */
    if (v3Norm(a2Hat_R) < epsilon) {
        // if norm(a2Hat_R) = 0, reference coincides with current angle
        hingedRigidBodyRefOut.theta = hingedRigidBodyIn.theta;
    }
    else {
        double thetaR = acos( fmin(fmax(v3Dot(a2Hat_B, a2Hat_R),-1),1) );
        // if a1Hat_B and a1Hat_R are opposite, take the negative of thetaR
        if (v3Dot(a1Hat_B, a1Hat_R) < 0) {
            thetaR = -thetaR;
        }
        // always make the absolute difference |thetaR-thetaC| smaller that 2*pi
        if (thetaR - thetaC > MPI) {
            hingedRigidBodyRefOut.theta = hingedRigidBodyIn.theta + thetaR - thetaC - 2*MPI;
        }
        else if (thetaR - thetaC < - MPI) {
            hingedRigidBodyRefOut.theta = hingedRigidBodyIn.theta + thetaR - thetaC + 2*MPI;
        }
        else {
            hingedRigidBodyRefOut.theta = hingedRigidBodyIn.theta + thetaR - thetaC;
        }
    }

    /*! implement finite differences to compute thetaDotR */
    double dt;
    if (this->count == 0) {
        hingedRigidBodyRefOut.thetaDot = 0;
    }
    else {
        dt = (double) (callTime - this->priorT) * NANO2SEC;
        hingedRigidBodyRefOut.thetaDot = (hingedRigidBodyRefOut.theta - this->priorThetaR) / dt;
    }
    // update stored variables
    this->priorThetaR = hingedRigidBodyRefOut.theta;
    this->priorT = callTime;
    this->count += 1;

    /* write output message */
    this->hingedRigidBodyRefOutMsg.write(&hingedRigidBodyRefOut, this->moduleID, callTime);
}
