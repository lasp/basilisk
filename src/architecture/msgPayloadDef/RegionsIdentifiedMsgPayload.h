#ifndef REGIONS_IDENTIFIED_H
#define REGIONS_IDENTIFIED_H

#include <mission/parameters.h>

/*! @brief Regions of interest extracted by camera */
typedef struct {
    double timeTag[MAX_NUMBER_REGIONS];      //!< [sec] time tag of the image which provided the region(s)
    int centerX[MAX_NUMBER_REGIONS];         //!< [pix] center pixel coordinate x
    int centerY[MAX_NUMBER_REGIONS];         //!< [pix] center pixel coordinate y
    int width[MAX_NUMBER_REGIONS];           //!< [pix] region width
    int height[MAX_NUMBER_REGIONS];          //!< [pix] region height
    int numberOfPixels[MAX_NUMBER_REGIONS];  //!< [-] number of pixels in the region
} RegionsIdentifiedMsgPayload;

#endif
