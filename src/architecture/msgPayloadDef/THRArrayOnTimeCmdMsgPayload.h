// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef THR_CMD_MESSAGE_H
#define THR_CMD_MESSAGE_H

#include <mission/parameters.h>

/*! @brief Structure used to define the output definition for vehicle effectors*/
typedef struct {
    double OnTimeRequest[MAX_EFF_CNT];  //!< - Control request fraction array
} THRArrayOnTimeCmdMsgPayload;

#endif
