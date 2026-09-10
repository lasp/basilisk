// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef MTB_CMD_MSG_H
#define MTB_CMD_MSG_H

#include <mission/parameters.h>

/*! @brief Message for magnetic torque bar dipole commands. */
typedef struct {
    double mtbDipoleCmds[MAX_EFF_CNT];  //!< [A-m2] magnetic torque bar dipole cmds
} MTBCmdMsgPayload;

#endif
