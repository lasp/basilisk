// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _THR_FIRING_SCHMITT_ALGORITHM_H
#define _THR_FIRING_SCHMITT_ALGORITHM_H

#include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>

#include <mission/parameters.h>
#include <stdint.h>

#include <array>

enum class PulsingRegime { ONPULSING = 0, OFFPULSING = 1 };

class ThrFiringSchmittAlgorithm {
public:
    void reset(uint64_t callTime, THRArrayConfigMsgPayload const &thrusterConfigPayload);
    THRArrayOnTimeCmdMsgPayload update(uint64_t callTime, THRArrayCmdForceMsgPayload &thrForceIn);
    double getLevelOn() const;
    void setLevelOn(double level);
    double getLevelOff() const;
    void setLevelOff(double level);
    double getThrMinFireTime() const;
    void setThrMinFireTime(double time);
    PulsingRegime getPulsingRegime() const;
    void setPulsingRegime(PulsingRegime state);

private:
    double levelOn{};                                 //!< [-] ON duty cycle fraction
    double levelOff{};                                //!< [-] OFF duty cycle fraction
    double thrMinFireTime{};                          //!< [s] Minimum ON time for thrusters
    PulsingRegime baseThrustState{};                  //!< [-] Indicates on-pulsing (0) or off-pulsing (1)
    uint32_t numThrusters{};                          //!< [-] The number of thrusters available on vehicle
    std::array<double, MAX_EFF_CNT> maxThrust{};      //!< [N] Max thrust
    std::array<bool, MAX_EFF_CNT> lastThrustState{};  //!< [-] ON/OFF state of thrusters from previous call
    uint64_t prevCallTime{};                          //!< callTime from previous function call
};

#endif
