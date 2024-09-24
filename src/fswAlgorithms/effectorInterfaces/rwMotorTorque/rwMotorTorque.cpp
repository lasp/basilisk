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
 Mapping required attitude control torque Lr to RW motor torques

 */

#include "fswAlgorithms/effectorInterfaces/rwMotorTorque/rwMotorTorque.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/linearAlgebra.h"

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwMotorTorque::Reset(uint64_t callTime)
{
    double *pAxis;                 /* pointer to the current control axis */
    int i;

    /*!- configure the number of axes that are controlled.
     This is determined by checking for a zero row to determinate search */
    this->numControlAxes = 0;
    for (i = 0; i < 3; i++)
    {
        pAxis = this->controlAxes_B + 3 * this->numControlAxes;
        if (v3Norm(pAxis) > 0.0) {
            this->numControlAxes += 1;
        }
    }
    if (this->numControlAxes == 0) {
        this->bskLogger.bskLog(BSK_INFORMATION,"rwMotorTorque() is not setup to control any axes!");
    }

    // check if the required input messages are included
    if (!this->rwParamsInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: rwMotorTorque.rwParamsInMsg wasn't connected.");
    }

    /*! - Read static RW config data message and store it in module variables */
    this->rwConfigParams = this->rwParamsInMsg();

    /*! - If no info is provided about RW availability we'll assume that all are available
     and create the [Gs] projection matrix once */
    if (!this->rwAvailInMsg.isLinked()) {
        this->numAvailRW = this->rwConfigParams.numRW;
        for (i = 0; i < this->rwConfigParams.numRW; i++){
            v3Copy(&this->rwConfigParams.GsMatrix_B[i * 3], &this->GsMatrix_B[i * 3]);
        }
    }
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwMotorTorque::UpdateState(uint64_t callTime)
{
    RWAvailabilityMsgPayload wheelsAvailability = {};    /*!< Msg containing RW availability */
    CmdTorqueBodyMsgPayload LrInputMsg;             /*!< Msg containing Lr control torque */
    CmdTorqueBodyMsgPayload LrInput2Msg;            /*!< Msg containing optional Lr control torque */
    double Lr_B[3];                             /*!< [Nm]    commanded ADCS control torque in body frame*/
    double Lr_C[3];                             /*!< [Nm]    commanded ADCS control torque projected onto control axes */
    double us[MAX_EFF_CNT];                     /*!< [Nm]    commanded ADCS control torque projected onto RWs g_s-Frames */
    double CGs[3][MAX_EFF_CNT];                 /*!< []      projection matrix from gs_i onto control axes */

    /*! - zero control torque and RW motor torque variables */
    v3SetZero(Lr_C);
    vSetZero(us, MAX_EFF_CNT);
    // wheelAvailability set to 0 (AVAILABLE) by default

    // check if the required input messages are included
    if (!this->vehControlInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: rwMotorTorque.vehControlInMsg wasn't connected.");
    }

    /*! - Read the input messages */
    LrInputMsg = this->vehControlInMsg();
    v3Copy(LrInputMsg.torqueRequestBody, Lr_B);

    /*! - Check if the optional second message is provided */
    if (this->vehControlIn2Msg.isLinked()) {
        LrInput2Msg = this->vehControlIn2Msg();
        v3Add(Lr_B, LrInput2Msg.torqueRequestBody, Lr_B);
    }

    /*! - Check if RW availability message is available */
    if (this->rwAvailInMsg.isLinked())
    {
        int numAvailRW = 0;

        /*! - Read in current RW availabilit Msg */
        wheelsAvailability = this->rwAvailInMsg();
        /*! - create the current [Gs] projection matrix with the available RWs */
        for (int i = 0; i < this->rwConfigParams.numRW; i++) {
            if (wheelsAvailability.wheelAvailability[i] == AVAILABLE)
            {
                v3Copy(&this->rwConfigParams.GsMatrix_B[i * 3], &this->GsMatrix_B[numAvailRW * 3]);
                numAvailRW += 1;
            }
        }
        /*! - update the number of currently available RWs */
        this->numAvailRW = numAvailRW;
    }

    /*! - Lr is assumed to be a positive torque onto the body, the [Gs]us must generate -Lr */
    v3Scale(-1.0, Lr_B, Lr_B);

    /*! - compute [Lr_C] = [C]Lr */
    mMultV(this->controlAxes_B, this->numControlAxes, 3, Lr_B, Lr_C);

    /*! - compute [CGs] */
    mSetZero(CGs, 3, MAX_EFF_CNT);
    for (uint32_t i=0; i<this->numControlAxes; i++) {
        for (int j=0; j<this->numAvailRW; j++) {
            CGs[i][j] = v3Dot(&this->GsMatrix_B[j * 3], &this->controlAxes_B[3 * i]);
        }
    }
    /*! - Compute minimum norm inverse for us = [CGs].T inv([CGs][CGs].T) [Lr_C]
     Having at least the same # of RW as # of control axes is necessary condition to guarantee inverse matrix exists. If matrix to invert it not full rank, the control torque output is zero. */
    if (this->numAvailRW >= (int) this->numControlAxes){
        double v3_temp[3]; /* inv([M]) [Lr_C] */
        double M33[3][3]; /* [M] = [CGs][CGs].T */
        double us_avail[MAX_EFF_CNT];   /* matrix of available RW motor torques */

        v3SetZero(v3_temp);
        mSetIdentity(M33, 3, 3);
        for (uint32_t i=0; i<this->numControlAxes; i++) {
            for (uint32_t j=0; j<this->numControlAxes; j++) {
                M33[i][j] = 0.0;
                for (int k=0; k < this->numAvailRW; k++) {
                    M33[i][j] += CGs[i][k]*CGs[j][k];
                }
            }
        }
        m33Inverse(M33, M33);
        m33MultV3(M33, Lr_C, v3_temp);

        /*! - compute the desired RW motor torques */
        /* us = [CGs].T v3_temp */
        vSetZero(us_avail, MAX_EFF_CNT);
        for (int i=0; i<this->numAvailRW; i++) {
            for (uint32_t j=0; j<this->numControlAxes; j++) {
                us_avail[i] += CGs[j][i] * v3_temp[j];
            }
        }

        /*! - map the desired RW motor torques to the available RWs */
        int j = 0;
        for (int i = 0; i < this->rwConfigParams.numRW; i++) {
            if (wheelsAvailability.wheelAvailability[i] == AVAILABLE)
            {
                us[i] = us_avail[j];
                j += 1;
            }
        }
    }

    /* store the output message */
    ArrayMotorTorqueMsgPayload rwMotorTorques = {};
    vCopy(us, this->rwConfigParams.numRW, rwMotorTorques.motorTorque);
    this->rwMotorTorqueOutMsg.write(&rwMotorTorques, this->moduleID, callTime);

    return;
}
