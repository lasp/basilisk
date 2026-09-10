// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef ARRAY_EFFECTOR_LOCK_H
#define ARRAY_EFFECTOR_LOCK_H

#include <mission/parameters.h>

/*! @brief Structure used to define the output definition for vehicle effectors*/
typedef struct {
    int effectorLockFlag[MAX_EFF_CNT];  //!< effector lock flag; 0 : do not lock; 1 : lock
} ArrayEffectorLockMsgPayload;

#endif
