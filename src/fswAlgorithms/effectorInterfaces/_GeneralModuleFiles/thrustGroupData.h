// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _THRUST_GROUP_DATA_
#define _THRUST_GROUP_DATA_

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>

#include <mission/parameters.h>

#include <cstdint>

/*! @brief Sub structure that contains all of the configuration data and output
    information for a single thruster group.  There can be several thruster
    groups available in a single control scheme.
*/
typedef struct {
    double nomThrustOn;                                   /*!< s The nominal thruster on-time for effectors*/
    uint32_t maxNumCmds;                                  /*!< - The maximum number of commands to output*/
    uint32_t numEffectors;                                /*!< - The number of effectors we have access to*/
    double minThrustRequest;                              /*!< - The minimum allowable on-time for a thruster*/
    double thrOnMap[3 * MAX_EFF_CNT];                     /*!< - Mapping between on-times and torque requests*/
    Message<THRArrayOnTimeCmdMsgPayload> thrOnTimeOutMsg; /*!< - The name of the output message*/
    THRArrayOnTimeCmdMsgPayload cmdRequests; /*!< - The array of on-time command requests sent to thrusters*/
} ThrustGroupData;

#endif
