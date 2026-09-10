// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef THR_ARRAY_MESSAGE_H
#define THR_ARRAY_MESSAGE_H

#include "THRConfigMsgPayload.h"

#include <mission/parameters.h>
#include <stdint.h>

/*! @brief FSW message definition containing the thruster cluster information */
typedef struct {
    uint32_t numThrusters;                       //!< [-] number of thrusters
    THRConfigMsgPayload thrusters[MAX_EFF_CNT];  //!< [-] array of thruster configuration information
} THRArrayConfigMsgPayload;

#endif
