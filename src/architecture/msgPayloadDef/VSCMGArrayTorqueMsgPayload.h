// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FSW_VSCMG_TORQUE_H
#define FSW_VSCMG_TORQUE_H

#include <mission/parameters.h>

/*! @brief Structure used to define the output definition for vehicle effectors*/
typedef struct {
    double wheelTorque[RW_EFF_CNT];   //!< [N-m] VSCMG wheel torque array
    double gimbalTorque[RW_EFF_CNT];  //!< [N-m] VSCMG gimbal torque array
} VSCMGArrayTorqueMsgPayload;

#endif
