// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef ARRAY_MOTOR_TORQUE_H
#define ARRAY_MOTOR_TORQUE_H

#include <mission/parameters.h>

/*! @brief Structure used to define the output definition for vehicle effectors*/
typedef struct {
    double motorTorque[MAX_EFF_CNT];  //!< [Nm]  motor torque array
} ArrayMotorTorqueMsgPayload;

#endif
