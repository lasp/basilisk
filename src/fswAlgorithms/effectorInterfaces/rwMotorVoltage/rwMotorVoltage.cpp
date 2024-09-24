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
    FSW MODULE: RW motor voltage command

 */

#include "fswAlgorithms/effectorInterfaces/rwMotorVoltage/rwMotorVoltage.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/linearAlgebra.h"
#include <string.h>

/*! This method performs a reset of the module as far as closed loop control is concerned.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime Sim time in nanos
 */
void RwMotorVoltage::Reset(uint64_t callTime)
{
    // check if the required input messages are included
    if (!this->rwParamsInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: rwMotorVoltage.rwParamsInMsg wasn't connected.");
    }

    /*! - Read static RW config data message and store it in module variables*/
    this->rwConfigParams = this->rwParamsInMsg();

    /* reset variables */
    memset(this->rwSpeedOld, 0, sizeof(double)*MAX_EFF_CNT);
    this->resetFlag = BOOL_TRUE;

    /* Reset the prior time flag state.
     If zero, control time step not evaluated on the first function call */
    this->priorTime = 0;
}

/*! Update performs the torque to voltage conversion. If a wheel speed message was provided, it also does closed loop control of the voltage sent. It then writes the voltage message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwMotorVoltage::UpdateState(uint64_t callTime)
{
    /* - Read the input messages */
//    double              torqueCmd[MAX_EFF_CNT];     /*!< [Nm]   copy of RW motor torque input vector */
    ArrayMotorTorqueMsgPayload torqueCmd;           /*!< copy of RW motor torque input message*/
    ArrayMotorVoltageMsgPayload voltageOut = {};            /*!< -- copy of the output message */

    // check if the required input messages are included
    if (!this->torqueInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: rwMotorVoltage.torqueInMsg wasn't connected.");
    }

    torqueCmd = this->torqueInMsg();

    RWSpeedMsgPayload  rwSpeed = {};                    /*!< [r/s] Reaction wheel speed estimates */
    if(this->rwSpeedInMsg.isLinked()) {
        rwSpeed = this->rwSpeedInMsg();
    }
    RWAvailabilityMsgPayload  rwAvailability = {};
    if (this->rwAvailInMsg.isLinked()){
        rwAvailability = this->rwAvailInMsg();
    }

    /* zero the output voltage vector */
    double  voltage[MAX_EFF_CNT];       /*!< [V]   RW voltage output commands */
    memset(voltage, 0, sizeof(double)*MAX_EFF_CNT);

    /* if the torque closed-loop is on, evaluate the feedback term */
    if (this->rwSpeedInMsg.isLinked()) {
        /* make sure the clock didn't just initialize, or the module was recently reset */
        if (this->priorTime != 0) {
            double dt = (callTime - this->priorTime) * NANO2SEC; /*!< [s]   control update period */
            double              OmegaDot[MAX_EFF_CNT];     /*!< [r/s^2] RW angular acceleration */
            for (int i=0; i<this->rwConfigParams.numRW; i++) {
                if (rwAvailability.wheelAvailability[i] == AVAILABLE && this->resetFlag == BOOL_FALSE) {
                    OmegaDot[i] = (rwSpeed.wheelSpeeds[i] - this->rwSpeedOld[i])/dt;
                    torqueCmd.motorTorque[i] -= this->K * (this->rwConfigParams.JsList[i] * OmegaDot[i] - torqueCmd.motorTorque[i]);
                }
                this->rwSpeedOld[i] = rwSpeed.wheelSpeeds[i];
            }
            this->resetFlag = BOOL_FALSE;
        }
        this->priorTime = callTime;
    }

    /* evaluate the feedforward mapping of torque into voltage */
    for (int i=0; i<this->rwConfigParams.numRW; i++) {
        if (rwAvailability.wheelAvailability[i] == AVAILABLE) {
            voltage[i] = (this->VMax - this->VMin)/this->rwConfigParams.uMax[i]
                        * torqueCmd.motorTorque[i];
            if (voltage[i]>0.0) voltage[i] += this->VMin;
            if (voltage[i]<0.0) voltage[i] -= this->VMin;
        }
    }

    /* check for voltage saturation */
    for (int i=0; i<this->rwConfigParams.numRW; i++) {
        if (voltage[i] > this->VMax) {
            voltage[i] = this->VMax;
        }
        if (voltage[i] < -this->VMax) {
            voltage[i] = -this->VMax;
        }
        voltageOut.voltage[i] = voltage[i];
    }

    /*
     store the output message
     */
    this->voltageOutMsg.write(&voltageOut, this->moduleID, callTime);

    return;
}
