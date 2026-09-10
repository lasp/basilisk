// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef ENCODER_H
#define ENCODER_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

#include <mission/parameters.h>

/*! @brief wheel speed encoder module class */
class Encoder : public SysModel {
public:
    Encoder();
    ~Encoder();

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);
    void readInputMessages();
    void writeOutputMessages(uint64_t CurrentClock);
    void encode(uint64_t currentSimNanos);

public:
    Message<RWSpeedMsgPayload> rwSpeedOutMsg;     //!< [rad/s] reaction wheel speed output message
    ReadFunctor<RWSpeedMsgPayload> rwSpeedInMsg;  //!< [rad/s] reaction wheel speed input message
    int rwSignalState[RW_EFF_CNT];                //!< vector of reaction wheel signal states
    int clicksPerRotation;                        //!< number of clicks per full rotation
    int numRW;                                    //!< number of reaction wheels
    BSKLogger bskLogger;                          //!< -- BSK Logging

private:
    RWSpeedMsgPayload rwSpeedBuffer;     //!< reaction wheel speed buffer for internal calculations
    RWSpeedMsgPayload rwSpeedConverted;  //!< reaction wheel speed buffer for converted values
    double remainingClicks[RW_EFF_CNT];  //!< remaining clicks from the previous iteration

    uint64_t prevTime;  //!< -- Previous simulation time observed
};

#endif
