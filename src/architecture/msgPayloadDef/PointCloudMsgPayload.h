// SPDX-License-Identifier: ISC
// Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef POINTCLOUDMSG_H
#define POINTCLOUDMSG_H

#include <mission/parameters.h>

#include <array>

//!@brief N-D point cloud
/*! This message contains a point cloud and corresponding time tag
 */
typedef struct
    //@cond DOXYGEN_IGNORE
    PointCloudMsgPayload
//@endcond
{
    uint64_t timeTag;                                 //!< --[ns]   Current vehicle time-tag associated with cloud
    bool valid;                                       //!< --  Quality of measurement
    int numberOfPoints;                               //!< -- [-] Number of points detected
    double points[MAX_SICP_POINTS * SICP_POINT_DIM];  //!< -- [-]  Point cloud array
} PointCloudMsgPayload;

#endif /* POINTCLOUDMSG_H */
