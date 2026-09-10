// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#ifndef THRFIRINGREMAINDERALGORITHM_C_H
#define THRFIRINGREMAINDERALGORITHM_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>

#include <mission/parameters.h>
#include <stdint.h>

    typedef struct {
        double pulseRemainder[MAX_EFF_CNT];  //!< [-] Unimplemented thrust pulses (number of minimum pulses)
        double thrMinFireTime;               //!< [s] Minimum fire time
        uint32_t numThrusters;               //!< [-] The number of thrusters available on vehicle
        double maxThrust[MAX_EFF_CNT];       //!< [N] Max thrust
        int baseThrustState;                 //!< [-] Indicates on-pulsing (0) or off-pulsing (1)
        double defaultControlPeriod;         //!< [s] Default control period used for first call
        uint64_t prevCallTime;               //!< callTime from previous function call
    } ThrFiringRemainderInternalState;

    void reset(ThrFiringRemainderInternalState* moduleState, THRArrayConfigMsgPayload localThrusterData);

    THRArrayOnTimeCmdMsgPayload
    updateState(ThrFiringRemainderInternalState* moduleState, uint64_t callTime, THRArrayCmdForceMsgPayload thrForceIn);

#ifdef __cplusplus
}
#endif

#endif  // THRFIRINGREMAINDERALGORITHM_C_H
