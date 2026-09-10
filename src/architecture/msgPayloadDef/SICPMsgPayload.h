// SPDX-License-Identifier: ISC
// Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SICP_H
#define SICP_H

#include <mission/parameters.h>

#include <array>

//!@brief Message containing the output of the Scaling Iterative Closest Point algorithm
/*! This message is output by the SICP module and contains the scaling matrix, rotation matrix, and translation vector
 * output after two point clouds are registered.
 */
typedef struct
    //@cond DOXYGEN_IGNORE
    SICPMsgPayload
//@endcond
{
    bool valid;                               //!< --[-] Message was purposefully populated
    uint64_t numberOfIteration;               //!< --[-] Number of iterations used for SICP convergence
    uint64_t timeTag;                         //!< --[-] Time at which these iterations were computed
    double scaleFactor[MAX_SICP_ITERATIONS];  //!< -- [-] Array of scale factors
    double
        rotationMatrix[SICP_POINT_DIM * SICP_POINT_DIM * MAX_SICP_ITERATIONS];  //!< -- [-]  Array of rotation matrices
    double translation[SICP_POINT_DIM * MAX_SICP_ITERATIONS];                   //!< -- [-] Array of translation vectors
} SICPMsgPayload;

#endif /* SICP_H */
