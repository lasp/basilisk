/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "mtbMomentumManagement.h"
#include "string.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/svd.h"

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.  The local copy of the
 message output buffer should be cleared.
 @return void
 @param callTime [ns] time the method is called
*/
void MtbMomentumManagement::Reset(uint64_t callTime)
{
    /*
     * Check if the required input messages are linked.
     */
    if (!this->rwParamsInMsg.isLinked()){
        this->bskLogger.bskLog(BSK_ERROR, "Error: mtbMomentumManagement.rwParamsInMsg is not connected.");
    }
    if(!this->mtbParamsInMsg.isLinked()){
        this->bskLogger.bskLog(BSK_ERROR, "Error: mtbMomentumManagement.mtbParamsInMsg is not connected.");
    }
    if (!this->tamSensorBodyInMsg.isLinked()){
        this->bskLogger.bskLog(BSK_ERROR, "Error: mtbMomentumManagement.tamSensorBodyInMsg is not connected.");
    }
    if (!this->rwSpeedsInMsg.isLinked()){
        this->bskLogger.bskLog(BSK_ERROR, "Error: mtbMomentumManagement.rwSpeedsInMsg is not connected.");
    }
    if (!this->rwMotorTorqueInMsg.isLinked()){
        this->bskLogger.bskLog(BSK_ERROR, "Error: mtbMomentumManagement.rwMotorTorqueInMsg is not connected.");
    }

    /*! - Read the input configuration messages.*/
    this->rwConfigParams = this->rwParamsInMsg();
    this->mtbConfigParams = this->mtbParamsInMsg();

    return;
}


/*! Computes the appropriate wheel torques and magnetic torque bar dipoles to bias the wheels to their desired speeds.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MtbMomentumManagement::UpdateState(uint64_t callTime)
{
    /*
     * Declare and initialize local variables.
     */
    int numRW = this->rwConfigParams.numRW;
    int numMTB = this->mtbConfigParams.numMTB;
    int j = 0;
    double BTilde_B[3*3];
    double BGt[3*MAX_EFF_CNT];
    double BGtPsuedoInverse[MAX_EFF_CNT*3];
    double uDelta_B[3];
    double uDelta_W[MAX_EFF_CNT];
    double GsPsuedoInverse[MAX_EFF_CNT*3];
    double Gs[3 * MAX_EFF_CNT];
    mSetZero(BTilde_B, 3, 3);
    mSetZero(BGt, 3, numMTB);
    mSetZero(BGtPsuedoInverse, numMTB, 3);
    v3SetZero(uDelta_B);
    vSetZero(uDelta_W, numRW);
    mSetZero(GsPsuedoInverse, numRW, 3);
    mTranspose(this->rwConfigParams.GsMatrix_B, this->rwConfigParams.numRW, 3, Gs);
    v3SetZero(this->tauDesiredMTB_B);
    v3SetZero(this->tauDesiredRW_B);
    vSetZero(this->hDeltaWheels_W, numRW);
    v3SetZero(this->hDeltaWheels_B);
    vSetZero(this->tauDesiredRW_W, numRW);
    vSetZero(this->tauIdealRW_W, numRW);
    v3SetZero(this->tauIdealRW_B);
    vSetZero(this->wheelSpeedError_W, numRW);

    /*
     * Read input messages and initialize output messages.
     */
    TAMSensorBodyMsgPayload tamSensorBodyInMsgBuffer = this->tamSensorBodyInMsg();
    RWSpeedMsgPayload rwSpeedsInMsgBuffer = this->rwSpeedsInMsg();
    ArrayMotorTorqueMsgPayload rwMotorTorqueInMsgBuffer = this->rwMotorTorqueInMsg();
    MTBCmdMsgPayload mtbCmdOutputMsgBuffer = {};
    ArrayMotorTorqueMsgPayload rwMotorTorqueOutMsgBuffer = rwMotorTorqueInMsgBuffer;

    /*! - Compute the wheel speed feedback.*/
    vSubtract(rwSpeedsInMsgBuffer.wheelSpeeds, numRW, this->wheelSpeedBiases, this->wheelSpeedError_W);
    vElementwiseMult(this->rwConfigParams.JsList, numRW, this->wheelSpeedError_W, this->hDeltaWheels_W);
    mMultV(Gs, 3, numRW, this->hDeltaWheels_W, this->hDeltaWheels_B);
    vScale(-this->cGain, this->hDeltaWheels_W, numRW, this->tauIdealRW_W);
    mMultV(Gs, 3, numRW, this->tauIdealRW_W, this->tauIdealRW_B);

    /*! - Compute the magnetic torque bar dipole commands. */
    v3TildeM(tamSensorBodyInMsgBuffer.tam_B, BTilde_B);
    mMultM(BTilde_B, 3, 3, this->mtbConfigParams.GtMatrix_B, 3, numMTB, BGt);
    solveSVD(BGt, 3, numMTB, mtbCmdOutputMsgBuffer.mtbDipoleCmds, this->tauIdealRW_B, 0.00000000001);
    vScale(-1.0, mtbCmdOutputMsgBuffer.mtbDipoleCmds, numMTB, mtbCmdOutputMsgBuffer.mtbDipoleCmds);

    /*
     * Saturate dipoles.
     */
    for (j = 0; j < numMTB; j++)
    {
        if (mtbCmdOutputMsgBuffer.mtbDipoleCmds[j] > this->mtbConfigParams.maxMtbDipoles[j])
            mtbCmdOutputMsgBuffer.mtbDipoleCmds[j] = this->mtbConfigParams.maxMtbDipoles[j];

        if (mtbCmdOutputMsgBuffer.mtbDipoleCmds[j] < -this->mtbConfigParams.maxMtbDipoles[j])
            mtbCmdOutputMsgBuffer.mtbDipoleCmds[j] = -this->mtbConfigParams.maxMtbDipoles[j];
    }

    /*! - Compute the desired Body torque produced by the torque bars.*/
    mMultV(BGt, 3, numMTB, mtbCmdOutputMsgBuffer.mtbDipoleCmds, this->tauDesiredMTB_B);
    vScale(-1.0, this->tauDesiredMTB_B, 3, this->tauDesiredMTB_B);

    /*! - Compute the reaction wheel torque commands.*/
    v3Subtract(this->tauDesiredMTB_B, this->tauIdealRW_B, uDelta_B);
    mMinimumNormInverse(Gs, 3, numRW, GsPsuedoInverse);
    mMultV(GsPsuedoInverse, numRW, 3, uDelta_B, uDelta_W);
    vAdd(this->tauIdealRW_W, numRW, uDelta_W, this->tauDesiredRW_W);

    /*! - Compute the desired Body torque produced by the reaction wheels.*/
    mMultV(Gs, 3, numRW, this->tauDesiredRW_W, this->tauDesiredRW_B);
    vScale(-1.0, this->tauDesiredRW_B, 3, this->tauDesiredRW_B);

    /*
     * Write output messages.
     */
    this->mtbCmdOutMsg.write(&mtbCmdOutputMsgBuffer, moduleID, callTime);
    vAdd(this->tauDesiredRW_W, numRW, rwMotorTorqueOutMsgBuffer.motorTorque, rwMotorTorqueOutMsgBuffer.motorTorque);
    this->rwMotorTorqueOutMsg.write(&rwMotorTorqueOutMsgBuffer, moduleID, callTime);

    return;
}

/*
 * Returns the tilde matrix in the form of a 1-D array.
 */
void v3TildeM(double v[3], double *m_result)
{
    m_result[MXINDEX(3, 0, 0)] = 0.0;
    m_result[MXINDEX(3, 0, 1)] = -v[2];
    m_result[MXINDEX(3, 0, 2)] = v[1];
    m_result[MXINDEX(3, 1, 0)] = v[2];
    m_result[MXINDEX(3, 1, 1)] = 0.0;
    m_result[MXINDEX(3, 1, 2)] = -v[0];
    m_result[MXINDEX(3, 2, 0)] = -v[1];
    m_result[MXINDEX(3, 2, 1)] = v[0];
    m_result[MXINDEX(3, 2, 2)] = 0.0;
}
