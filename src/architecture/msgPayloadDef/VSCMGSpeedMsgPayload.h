// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef VSCMG_SPEED_MESSAGE_STRUCT_H
#define VSCMG_SPEED_MESSAGE_STRUCT_H

#include <mission/parameters.h>

/*! @brief Structure used to define the output definition for VSCMG speeds*/
typedef struct {
    double wheelSpeeds[RW_EFF_CNT];   //!< r/s The current angular velocities of the VSCMG wheel
    double gimbalAngles[RW_EFF_CNT];  //!< r The current angles of the VSCMG gimbal
    double gimbalRates[RW_EFF_CNT];   //!< r/s The current angular velocities of the VSCMG gimbal
} VSCMGSpeedMsgPayload;

#endif
