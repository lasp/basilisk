// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _RW_AVAILABILITY_FSW_MSG_H
#define _RW_AVAILABILITY_FSW_MSG_H

#include "fswAlgorithms/fswUtilities/fswDefinitions.h"

#include <mission/parameters.h>

/*! @brief Array with availability of RW */
typedef struct {
    FSWdeviceAvailability wheelAvailability[RW_EFF_CNT];  //!< The current state of the wheel
} RWAvailabilityMsgPayload;

#endif
