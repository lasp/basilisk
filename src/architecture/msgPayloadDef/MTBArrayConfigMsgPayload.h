// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef MTB_ARRAY_CONFIG_MSG_H
#define MTB_ARRAY_CONFIG_MSG_H

#include <mission/parameters.h>

/*! @brief magnetic torque bar array configuration msg */
typedef struct {
    int numMTB;                          //!< [-] number of magnetic torque bars on the spacecraft
    double GtMatrix_B[3 * MAX_EFF_CNT];  //!< [-] magnetic torque bar alignment matrix in Body frame components, must be
                                         //!< provided in row-major format
    double maxMtbDipoles[MAX_EFF_CNT];   //!< [A-m2] maximum commandable dipole for each magnetic torque bar
} MTBArrayConfigMsgPayload;

#endif
