// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef MTB_MOMENTUM_MANAGEMENT_H
#define MTB_MOMENTUM_MANAGEMENT_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/MTBArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/MTBCmdMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/TAMSensorBodyMsgPayload.h>

#include <mission/parameters.h>

/*! @brief Top level structure for the sub-module routines. */
class MtbMomentumManagement : public SysModel {
public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /*
     * Configs.
     */
    double wheelSpeedBiases[RW_EFF_CNT];  //!< [rad/s] reaction wheel speed biases
    double cGain;                         //!<[1/s]  reaction wheel momentum feedback gain

    /*
     * Inputs.
     */
    ReadFunctor<RWArrayConfigMsgPayload> rwParamsInMsg;    //!< input message for RW parameters
    ReadFunctor<MTBArrayConfigMsgPayload> mtbParamsInMsg;  //!< input message for MTB layout
    ReadFunctor<TAMSensorBodyMsgPayload>
        tamSensorBodyInMsg;                        //!< input message for magnetic field sensor data in the Body frame
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;  //!< input message for RW speeds
    ReadFunctor<RwMotorTorqueMsgPayload> rwMotorTorqueInMsg;  //!< input message for RW motor torques

    /*
     * Outputs.
     */
    Message<MTBCmdMsgPayload> mtbCmdOutMsg;                //!< output message for MTB dipole commands
    Message<RwMotorTorqueMsgPayload> rwMotorTorqueOutMsg;  //!< output message for RW motor torques

    /*
     * Other.
     */
    BSKLogger bskLogger = {};           //!< BSK Logging
    double tauDesiredMTB_B[3];          //!< [N-m] desired torque produced by the magnetic torque bars in the Body frame
    double tauDesiredRW_B[3];           //!< [N-m]  desired torque produced by the reaction wheels in the Body frame
    double hDeltaWheels_W[RW_EFF_CNT];  //!<  [N-m-s] momentum of each wheel
    double hDeltaWheels_B[3];           //!<  [N-m-s] momentum of reaction wheels in the Body frame
    double tauDesiredRW_W[RW_EFF_CNT];  //!<  [N-m] Desired individual wheel torques
    double tauIdealRW_W[RW_EFF_CNT];    //!<  [N-m-s] Ideal individual wheel torques
    double tauIdealRW_B[RW_EFF_CNT];    //!<  [N-m-s] Ideal wheel torque in the body frame
    double
        wheelSpeedError_W[RW_EFF_CNT];  //!<  [N-m-s] difference between current wheel speeds and desired wheel speeds
    RWArrayConfigMsgPayload rwConfigParams;    //!< configuration for RW's
    MTBArrayConfigMsgPayload mtbConfigParams;  //!< configuration for MTB layout
};

void v3TildeM(double v[3], double* result);

#endif
