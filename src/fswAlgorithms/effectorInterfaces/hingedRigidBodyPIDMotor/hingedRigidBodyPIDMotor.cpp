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


/* modify the path to reflect the new module names */
#include "hingedRigidBodyPIDMotor.h"

/* Support files */
#include "architecture/utilities/macroDefinitions.h"

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime [ns] time the method is called
*/
void HingedRigidBodyPIDMotor::Reset(uint64_t callTime)
{
    if (!this->hingedRigidBodyInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: solarArrayAngle.hingedRigidBodyInMsg wasn't connected.");
    }
    if (!this->hingedRigidBodyRefInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: solarArrayAngle.hingedRigidBodyRefInMsg wasn't connected.");
    }

    /*! initialize module parameters to compute integral error via trapezoid integration */
    this->priorTime = 0;
    this->priorThetaError = 0;
    this->intError = 0;
}

/*! This method computes the control torque to the solar array drive based on a PD control law
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void HingedRigidBodyPIDMotor::UpdateState(uint64_t callTime)
{
    /*! - Create and assign buffer messages */
    ArrayMotorTorqueMsgPayload motorTorqueOut = {};
    HingedRigidBodyMsgPayload  hingedRigidBodyIn = this->hingedRigidBodyInMsg();
    HingedRigidBodyMsgPayload  hingedRigidBodyRefIn = this->hingedRigidBodyRefInMsg();

    /*! compute angle error and error rate */
    double thetaError    = hingedRigidBodyRefIn.theta - hingedRigidBodyIn.theta;
    double thetaErrorDot = hingedRigidBodyRefIn.thetaDot - hingedRigidBodyIn.thetaDot;

    /*! extract gains from input */
    double K = this->K;
    double P = this->P;
    double I = this->I;

    /*! compute integral term */
    double dt;
    if (callTime != 0) {
        dt = (callTime - this->priorTime) * NANO2SEC;
        this->intError += (thetaError + this->priorThetaError) * dt / 2;
    }

    /*! update stored quantities */
    this->priorThetaError = thetaError;
    this->priorTime = callTime;

    /*! compute torque */
    double T = K * thetaError + P * thetaErrorDot + I * this->intError;
    motorTorqueOut.motorTorque[0] = T;

    /*! write output message */
    this->motorTorqueOutMsg.write(&motorTorqueOut, this->moduleID, callTime);
}
