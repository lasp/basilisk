// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef CSS_ARRAY_SENSOR_MESSAGE_H
#define CSS_ARRAY_SENSOR_MESSAGE_H

#include <mission/parameters.h>

/*! @brief Output structure for CSS array or constellation interface.  Each element contains the raw measurement which
 * should be a cosine value nominally */
typedef struct {
    double timeTag;                        //!< [s]   Current vehicle time-tag associated with measurements
    double CosValue[MAX_NUM_CSS_SENSORS];  //!< Current measured CSS value (ideally a cosine value) for the
                                           //!< constellation of CSS sensors
} CSSArraySensorMsgPayload;

#endif
