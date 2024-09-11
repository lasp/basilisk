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
    mrpPD Module

 */

#include "fswAlgorithms/attControl/mrpPD/mrpPD.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include <string.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime [ns] Time the method is called
*/
void MrpPD::Reset(uint64_t callTime)
{
    // check if the required input messages are included
    if (!this->guidInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: mrpPD.guidInMsg wasn't connected.");
    }

    if (!this->vehConfigInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: mrpPD.vehConfigInMsg wasn't connected.");
    }


    /*! - read in spacecraft configuration message */
    VehicleConfigMsgPayload vcInMsg = this->vehConfigInMsg();
    mCopy(vcInMsg.ISCPntB_B, 1, 9, this->ISCPntB_B);
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MrpPD::UpdateState(uint64_t callTime)
{
    double              Lr[3];                  /*!< required control torque vector [Nm] */
    double              omega_BN_B[3];          /*!< Inertial angular body vector in body B-frame components */
    CmdTorqueBodyMsgPayload controlOutMsg = {}; /*!< Control output requests */
    AttGuidMsgPayload       guidInMsg;          /*!< Guidance Message */
    double              v3_temp1[3];
    double              v3_temp2[3];
    double              v3_temp3[3];
    double              v3_temp4[3];

    /*! - Read the guidance input message */
    guidInMsg = this->guidInMsg();

    /*! - Compute angular body rate */
    v3Add(guidInMsg.omega_BR_B, guidInMsg.omega_RN_B, omega_BN_B);

    /*! - Evaluate required attitude control torque */
    /* Lr =  K*sigma_BR + P*delta_omega  - omega_r x [I]omega - [I](d(omega_r)/dt - omega x omega_r) + L
     */
    v3Scale(this->K, guidInMsg.sigma_BR, v3_temp1); /* + K * sigma_BR */
    v3Scale(this->P, guidInMsg.omega_BR_B, v3_temp2); /* + P * delta_omega */
    v3Add(v3_temp1, v3_temp2, Lr);

    /* omega x [I]omega */
    m33MultV3(RECAST3X3 this->ISCPntB_B, omega_BN_B, v3_temp3);
    v3Cross(guidInMsg.omega_RN_B, v3_temp3, v3_temp3); /* omega_r x [I]omega */
    v3Subtract(Lr, v3_temp3, Lr);

    /* [I](d(omega_r)/dt - omega x omega_r) */
    v3Cross(omega_BN_B, guidInMsg.omega_RN_B, v3_temp4);
    v3Subtract(guidInMsg.domega_RN_B, v3_temp4, v3_temp4);
    m33MultV3(RECAST3X3 this->ISCPntB_B, v3_temp4, v3_temp4);
    v3Subtract(Lr, v3_temp4, Lr);

    v3Add(this->knownTorquePntB_B, Lr, Lr); /* + L */
    v3Scale(-1.0, Lr, Lr);

    /*! - Store and write the output message */
    v3Copy(Lr, controlOutMsg.torqueRequestBody);
    this->cmdTorqueOutMsg.write(&controlOutMsg, moduleID, callTime);

    return;
}
