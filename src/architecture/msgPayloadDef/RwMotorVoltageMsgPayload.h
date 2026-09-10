// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SIM_RW_VOLTAGE_INPUT_H
#define SIM_RW_VOLTAGE_INPUT_H

#include <mission/parameters.h>

/*! @brief Structure used to define the message format of the motor voltage input  */
typedef struct {
    double voltage[RW_EFF_CNT];  //!< [V]     Motor voltage input value
} RwMotorVoltageMsgPayload;

#endif
