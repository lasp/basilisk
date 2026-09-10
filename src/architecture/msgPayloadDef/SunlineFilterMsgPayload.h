// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SUNLINE_FILTER_MESSAGE_H
#define SUNLINE_FILTER_MESSAGE_H

#include <mission/parameters.h>

#define SKF_N_STATES 6
#define SKF_N_STATES_SWITCH 6
#define EKF_N_STATES_SWITCH 5
#define SKF_N_STATES_HALF 3

/*! @brief structure for filter-states output for the unscented kalman filter
 implementation of the sunline state estimator*/
typedef struct {
    double timeTag;                             //!< [s] Current time of validity for output
    double covar[SKF_N_STATES * SKF_N_STATES];  //!< [-] Current covariance of the filter
    double state[SKF_N_STATES];                 //!< [-] Current estimated state of the filter
    double stateError[SKF_N_STATES];            //!< [-] Current deviation of the state from the reference state
    double postFitRes[MAX_NUM_CSS_SENSORS];     //!< [-] PostFit Residuals
    int numObs;                                 //!< [-] Valid observation count for this frame
} SunlineFilterMsgPayload;

#endif
