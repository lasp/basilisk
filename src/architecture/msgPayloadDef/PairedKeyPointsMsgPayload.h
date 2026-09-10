// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef KEYPOINTSMSG_H
#define KEYPOINTSMSG_H

#include <mission/parameters.h>

//!@brief Optical Navigation measurement message containing the matched key points between two images
/*! This message is output by the optical flow module and contains the key points shared between the two image
 * that were processed, as well as the attitudes, validity, time tags, the camera ID, and the number of points detected.
 */
typedef struct
    //@cond DOXYGEN_IGNORE
    PairedKeyPointsMsgPayload
//@endcond
{
    bool valid;                    //!< --  Quality of measurement
    int64_t cameraID;              //!< -- [-]   ID of the camera that took the images
    uint64_t timeTag_firstImage;   //!< --[ns]   Current vehicle time-tag associated with first image
    uint64_t timeTag_secondImage;  //!< --[ns]   Current vehicle time-tag associated with second image
    double keyPoints_firstImage[2 * MAX_KEY_POINTS];   //!< -- [-]   Paired features in first image
    double keyPoints_secondImage[2 * MAX_KEY_POINTS];  //!< -- [-]   Paired features in second image
    double sigma_BN_firstImage[3];   //!< -- [-]   Spacecraft body frame attitude associated with first image
    double sigma_BN_secondImage[3];  //!< -- [-]  Spacecraft body frame attitude associated with second image
    double sigma_TN_firstImage[3];   //!< -- [-]   Target frame wrt inertial at time of first image
    double sigma_TN_secondImage[3];  //!< -- [-]  Target frame wrt inertial at time of second image
    int keyPointsFound;              //!< -- [-] Number of paired features
} PairedKeyPointsMsgPayload;

#endif /* KEYPOINTSMSG_H */
