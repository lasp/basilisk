// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef THR_ARRAY_CMD_FORCE_MESSAGE_H_
#define THR_ARRAY_CMD_FORCE_MESSAGE_H_

#include <mission/parameters.h>

/*! @brief Message used to define a vector of thruster force commands */
typedef struct {
    double thrForce[MAX_EFF_CNT];  //!< [N] array of thruster force values
} THRArrayCmdForceMsgPayload;

#endif
