// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef RW_MOTOR_TORQUE_MESSAGE_H
#define RW_MOTOR_TORQUE_MESSAGE_H

#include <mission/parameters.h>

/*! @brief Structure used to define the message format of the motor torque */
typedef struct {
    double motorTorque[RW_EFF_CNT];  //!< [Nm]  motor torque array
} RwMotorTorqueMsgPayload;

#endif
