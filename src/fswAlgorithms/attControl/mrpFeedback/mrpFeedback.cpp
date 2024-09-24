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
    mrpFeedback Module

 */

#include "fswAlgorithms/attControl/mrpFeedback/mrpFeedback.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"

#include <string.h>
#include <math.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MrpFeedback::Reset(uint64_t callTime)
{
    /* - Read the input messages */
    int i;

    /* check that optional messages are correct connected */
    if(this->rwParamsInMsg.isLinked()) {
        if (!this->rwSpeedsInMsg.isLinked()) {
            this->bskLogger.bskLog(BSK_ERROR, "Error: the mrpFeedback.rwSpeedsInMsg wasn't connected while rwParamsInMsg was connected.");
        }

    }

    // check if the required message has not been connected
    if (!this->guidInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: mrpFeedback.guidInMsg wasn't connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: the mrpFeedback.vehConfigInMsg wasn't connected.");
    }

    /*! - zero and read in vehicle configuration message */
    VehicleConfigMsgPayload sc = this->vehConfigInMsg();
    /*! - copy over spacecraft inertia tensor */
    for (i=0; i < 9; i++){
        this->ISCPntB_B[i] = sc.ISCPntB_B[i];
    };

    /*! - zero the number of RW by default */
    this->rwConfigParams.numRW = 0;

    /*! - check if RW configuration message exists */
    if (this->rwParamsInMsg.isLinked()) {
        /*! - Read static RW config data message and store it in module variables*/
        this->rwConfigParams = this->rwParamsInMsg();
    }

    /*! - Reset the integral measure of the rate tracking error */
    v3SetZero(this->int_sigma);

    /*! - Reset the prior time flag state.
     If zero, control time step not evaluated on the first function call */
    this->priorTime = 0;
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MrpFeedback::UpdateState(uint64_t callTime)
{
    AttGuidMsgPayload      guidCmd;            /* attitude tracking error message */
    RWSpeedMsgPayload      wheelSpeeds;        /* Reaction wheel speed message */
    RWAvailabilityMsgPayload wheelsAvailability = {}; /* Reaction wheel availability message */
    CmdTorqueBodyMsgPayload controlOut = {};        /* output message */
    CmdTorqueBodyMsgPayload intFeedbackOut;    /* output int feedback msg */

    double              dt;                 /* [s] control update period */
    double              Lr[3];              /* required control torque vector [Nm] */
    double              omega_BN_B[3];      /* [r/s] body angular velocity message */
    double              v3_1[3];
    double              v3_2[3];
    double              v3_3[3];
    double              v3_4[3];
    double              v3_5[3];
    double              v3_6[3];
    double              v3_7[3];
    double              v3_8[3];
    double              v3_9[3];
    double              v3_10[3];
    double              v3_11[3];
    double              v3_12[3];
    double              intCheck;           /* Check magnitude of integrated attitude error */
    int                 i;
    double              *wheelGs;           /* Reaction wheel spin axis pointer */

    /*! - Read the attitude tracking error message */
    guidCmd = this->guidInMsg();

    /*! - read in optional RW speed and availability message */
    if(this->rwConfigParams.numRW > 0) {
        wheelSpeeds = this->rwSpeedsInMsg();
        if (this->rwAvailInMsg.isLinked()) {
            wheelsAvailability = this->rwAvailInMsg();
        }
    }

    /*! - compute control update time */
    if (this->priorTime == 0) {
        dt = 0.0;
    } else {
        dt = (callTime - this->priorTime) * NANO2SEC;
    }
    this->priorTime = callTime;

    /*! - compute body rate */
    v3Add(guidCmd.omega_BR_B, guidCmd.omega_RN_B, omega_BN_B);

    /*! - evaluate integral term */
    v3SetZero(this->z);
    if (this->Ki > 0) {   /* check if integral feedback is turned on  */
        v3Scale(this->K * dt, guidCmd.sigma_BR, v3_1);
        v3Add(v3_1, this->int_sigma, this->int_sigma);

        for (i=0;i<3;i++) {
            intCheck = fabs(this->int_sigma[i]);
            if (intCheck > this->integralLimit) {
                this->int_sigma[i] *= this->integralLimit/intCheck;
            }
        }/* keep int_sigma less than integralLimit */
        m33MultV3(RECAST3X3 this->ISCPntB_B, guidCmd.omega_BR_B, v3_2); /* -[v3Tilde(omega_r+Ki*z)]([I]omega + [Gs]h_s) */
        v3Add(this->int_sigma, v3_2, this->z);
    }

    /*! - evaluate required attitude control torque Lr */
    v3Scale(this->K, guidCmd.sigma_BR, Lr);           /* +K sigma_BR */
    v3Scale(this->P, guidCmd.omega_BR_B,
            v3_3);                                          /* +P delta_omega */
    v3Add(v3_3, Lr, Lr);
    v3Scale(this->Ki, this->z, v3_4);
    v3Scale(this->P, v3_4, v3_5);                       /* +P*Ki*z */
    v3Add(v3_5, Lr, Lr);

    /* -[v3Tilde(omega_r+Ki*z)]([I]omega + [Gs]h_s) */
    m33MultV3(RECAST3X3 this->ISCPntB_B, omega_BN_B, v3_6);
    for(i = 0; i < this->rwConfigParams.numRW; i++)
    {
        if (wheelsAvailability.wheelAvailability[i] == AVAILABLE){ /* check if wheel is available */
            wheelGs = &(this->rwConfigParams.GsMatrix_B[i*3]);
            v3Scale(this->rwConfigParams.JsList[i] * (v3Dot(omega_BN_B, wheelGs) + wheelSpeeds.wheelSpeeds[i]),
                    wheelGs, v3_7);                                 /* h_s_i */
            v3Add(v3_6, v3_7, v3_6);
        }
    }

    if (this->controlLawType == 0) {
        v3Add(guidCmd.omega_RN_B, v3_4, v3_8);      /* v3_8 = omega_RN_B + K_I * z */
    }
    else {
        v3Copy(omega_BN_B, v3_8);                        /* v3_8 = omega_BN_B */
    }
    v3Cross(v3_8, v3_6, v3_9);
    v3Subtract(Lr, v3_9, Lr);

    v3Cross(omega_BN_B, guidCmd.omega_RN_B, v3_10);
    v3Subtract(v3_10, guidCmd.domega_RN_B, v3_11);
    m33MultV3(RECAST3X3 this->ISCPntB_B, v3_11, v3_12);   /* +[I](-d(omega_r)/dt + omega x omega_r) */
    v3Add(v3_12, Lr, Lr);

    v3Add(this->knownTorquePntB_B, Lr, Lr);           /* +L */
    v3Scale(-1.0, Lr, Lr);                                  /* compute the net positive control torque onto the spacecraft */


    /*! - set the output message and write it out */
    v3Copy(Lr, controlOut.torqueRequestBody);
    this->cmdTorqueOutMsg.write(&controlOut, moduleID, callTime);

    /*! - write the output integral feedback torque */
    v3Scale(-1.0, v3_5, intFeedbackOut.torqueRequestBody);
    this->intFeedbackTorqueOutMsg.write(&intFeedbackOut, this->moduleID, callTime);

    return;
}
