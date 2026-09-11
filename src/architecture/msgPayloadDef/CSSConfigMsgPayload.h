// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef CSS_CONFIG_MESSAGE_H
#define CSS_CONFIG_MESSAGE_H

#include "architecture/msgPayloadDef/CSSUnitConfigMsgPayload.h"

#include <mission/parameters.h>
#include <stdint.h>

/*! @brief Structure used to contain the configuration information for
 each sun sensor*/
typedef struct {
    uint32_t nCSS;                                         //!< [-] Number of coarse sun sensors in cluster
    CSSUnitConfigMsgPayload cssVals[MAX_NUM_CSS_SENSORS];  //!< [-] constellation of CSS elements
} CSSConfigMsgPayload;

#endif
