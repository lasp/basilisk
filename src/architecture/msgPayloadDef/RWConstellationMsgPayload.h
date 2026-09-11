// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _RW_CONSTELLATION_MESSAGE_H
#define _RW_CONSTELLATION_MESSAGE_H

#include "RWConfigElementMsgPayload.h"

#include <mission/parameters.h>

/*! @brief Message used to define an array of RW FSW configurations  */
typedef struct {
    int numRW;                                             //!< [-] number of RWs
    RWConfigElementMsgPayload reactionWheels[RW_EFF_CNT];  //!< [-] array of the reaction wheels
} RWConstellationMsgPayload;

#endif
