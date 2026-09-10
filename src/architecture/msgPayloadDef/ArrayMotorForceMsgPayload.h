// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef ARRAY_MOTOR_FORCE_H
#define ARRAY_MOTOR_FORCE_H

#include <mission/parameters.h>

/*! @brief Structure used to define the output definition for vehicle effectors*/
typedef struct {
    double motorForce[MAX_EFF_CNT];  //!< [N]  motor force array
} ArrayMotorForceMsgPayload;

#endif
