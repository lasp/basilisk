// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

/**
 * This module implements visual object tracking using center of brightness detection
 * and region identification. It processes multiple detected regions and identifies
 * the most relevant region of interest based on pixel count and spatial proximity.
 */

#ifndef _IMAGE_PROC_REGIONS_ALGORITHM_H_
#define _IMAGE_PROC_REGIONS_ALGORITHM_H_

#include <mission/parameters.h>
#include <stdint.h>

#include <Eigen/Core>

/**
 * @brief Structure representing a detected region of interest in an image
 */
struct RegionOfInterest {
    double timeTag{};                                        //!< [sec] Time tag of the image which provided the region
    Eigen::Vector2i regionCenter = Eigen::Vector2i::Zero();  //!< [pix] Center pixel coordinate (x, y)
    Eigen::Vector2i regionSize = Eigen::Vector2i::Zero();    //!< [pix] Width and height of the region
    Eigen::Vector2i centerOfBrightness = Eigen::Vector2i::Zero();  //!< [pix] Center of brightness coordinate (x, y)
    int numberOfPixels{};                                          //!< [-] Number of pixels in the region
};

/**
 * @brief Visual object tracking algorithm using center of brightness detection
 *
 * This class implements a sophisticated region of interest identification algorithm
 * that can:
 * - Apply windowing to filter regions based on spatial constraints
 * - Order regions by pixel count
 * - Identify the most relevant region based on proximity and size
 * - Handle multiple regions that may represent the same object
 *
 * The algorithm uses a barycenter approach when multiple regions are close together,
 * treating them as potentially the same object that has been split into multiple
 * detections.
 */
class RegionsOfInterestAlgorithm {
public:
    RegionsOfInterestAlgorithm();
    ~RegionsOfInterestAlgorithm();

    RegionOfInterest update(std::array<RegionOfInterest, MAX_NUMBER_REGIONS> const &regions) const;
    void reset();

    void setMaxRoiSeparation(int32_t pixelSeparation);
    int32_t getMaxRoiSeparation() const;
    void setCameraId(int32_t id);
    int32_t getCameraId() const;
    void setMinimumDetectionSize(int32_t pixels);
    int32_t getMinimumDetectionSize() const;
    void setWindowCenter(Eigen::Vector2i const &center);
    Eigen::Vector2i getWindowCenter() const;
    void setWindowSize(int32_t width, int32_t height);
    Eigen::Vector2i getWindowSize() const;
    void setImageSize(int32_t width, int32_t height);
    Eigen::Vector2i getImageSize() const;

private:
    void computeWindow();
    bool regionInWindow(RegionOfInterest const &region) const;
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> applyWindow(
        std::array<RegionOfInterest, MAX_NUMBER_REGIONS> const &regions
    ) const;
    RegionOfInterest identifyRoi(std::array<RegionOfInterest, MAX_NUMBER_REGIONS> const &regions) const;
    static std::array<RegionOfInterest, MAX_NUMBER_REGIONS> orderRegions(
        std::array<RegionOfInterest, MAX_NUMBER_REGIONS> const &regions
    );

    RegionOfInterest regionOfInterest{};                         //!< [ROI] Final region of interest
    Eigen::Vector2i imageSize = Eigen::Vector2i(1'024, 1'024);   //!< [bool] Size the incoming image
    int32_t minDetectionPixel = 2;                               //!< [int] Minimum pixel detection size
    int32_t maxSeparation{500};                                  //!< [px] width of mask to be used for windowing
    int32_t cameraId{};                                          //!< [-] Id of the camera imaging
    Eigen::Vector2i windowCenter = Eigen::Vector2i::Zero();      //!< [px] center of mask to be used for windowing
    int32_t windowWidth{};                                       //!< [px] width of mask to be used for windowing
    int32_t windowHeight{};                                      //!< [px] height of mask to be used for windowing
    Eigen::Vector2i windowPointTopLeft = Eigen::Vector2i(0, 0);  //!< [px] top left point of window
    Eigen::Vector2i windowPointBottomRight = Eigen::Vector2i(1'024, 1'024);
    ;  //!< [px] bottom right point of window
};

#endif
