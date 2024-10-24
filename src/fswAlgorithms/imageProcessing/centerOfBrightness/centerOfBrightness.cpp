/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include "centerOfBrightness.h"

CenterOfBrightness::CenterOfBrightness() = default;

CenterOfBrightness::~CenterOfBrightness() = default;

/*! This method performs a complete reset of the module.  Local module variables that retain time varying states
 * between function calls are reset to their default values.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CenterOfBrightness::Reset(uint64_t CurrentSimNanos)
{
    if (!this->imageInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "CenterOfBrightness.imageInMsg wasn't connected.");
    }
}

/*! This module reads an OpNav image and extracts the weighted center of brightness. It performs a grayscale, a blur,
 * and a threshold on the image before summing the weighted pixel intensities in order to average them with the
 * total detected intensity. This provides the center of brightness measurement (as well as the total number of
 * bright pixels)
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CenterOfBrightness::UpdateState(uint64_t CurrentSimNanos)
{
    CameraImageMsgPayload imageBuffer = this->imageInMsg.zeroMsgPayload;

    OpNavCOBMsgPayload cobBuffer;
    cobBuffer = this->opnavCOBOutMsg.zeroMsgPayload;

    cv::Mat imageCV;

    /*! - Read in the image*/
    if(this->imageInMsg.isLinked())
    {
        imageBuffer = this->imageInMsg();
        this->sensorTimeTag = this->imageInMsg.timeWritten();
    }
    /* Added for debugging purposes*/
    if (!this->filename.empty()){
        imageCV = cv::imread(this->filename, cv::IMREAD_COLOR);
    }
    else if(imageBuffer.valid == 1 && imageBuffer.timeTag >= CurrentSimNanos){
        /*! - Recast image pointer to CV type*/
        std::vector<unsigned char> vectorBuffer((char*)imageBuffer.imagePointer, (char*)imageBuffer.imagePointer + imageBuffer.imageBufferLength);
        imageCV = cv::imdecode(vectorBuffer, cv::IMREAD_COLOR);
    }
    else{
        /*! - If no image is present, write zeros in message */
        this->opnavCOBOutMsg.write(&cobBuffer, this->moduleID, CurrentSimNanos);
        return;
    }

    std::string dirName;
    /*! - Save image to prescribed path if requested */
    if (this->saveImages) {
        dirName = this->saveDir + std::to_string((double) CurrentSimNanos * NANO2SEC) + ".png";
        if (!cv::imwrite(dirName, imageCV)) {
            bskLogger.bskLog(BSK_WARNING, "CenterOfBrightness: wasn't able to save images.");
        }
    }

    this->computeWindow(imageCV);
    if (this->validWindow) {
        this->applyWindow(imageCV);
    }

    std::vector<cv::Vec2i> locations = this->extractBrightPixels(imageCV);

    /*!- If no lit pixels are found do not validate the image as a measurement */
    if (!locations.empty()){
        std::pair<Eigen::Vector2d, double> cobData;
        cobData = this->computeWeightedCenterOfBrightness(locations);

        double averageBrightnessOld = 0.0;
        if (this->brightnessHistory.rows() > 0){
            averageBrightnessOld = this->brightnessHistory.mean();
        }
        this->updateBrightnessHistory(cobData.second);
        double averageBrightnessNew = this->brightnessHistory.mean();
        double brightnessIncrease = 0.0;
        if (averageBrightnessOld > 0.0){
            brightnessIncrease = (averageBrightnessNew - averageBrightnessOld)
                    / averageBrightnessOld;
        }

        /*! If brightness increase is less than brightness increase threshold, do not validate image */
        if (brightnessIncrease >= this->relativeBrightnessIncreaseThreshold){
            cobBuffer.valid = true;
            cobBuffer.timeTag = this->sensorTimeTag;
            cobBuffer.cameraID = imageBuffer.cameraID;
            cobBuffer.centerOfBrightness[0] = cobData.first[0];
            cobBuffer.centerOfBrightness[1] = cobData.first[1];
            cobBuffer.pixelsFound = static_cast<int32_t> (locations.size());
        }
        cobBuffer.rollingAverageBrightness = averageBrightnessNew;
    }

    this->opnavCOBOutMsg.write(&cobBuffer, this->moduleID, CurrentSimNanos);

}

/*! Method extracts the bright pixels (above a given threshold) by first grayscaling then bluring image.
 @return std 2 vector of integers
 @param image openCV matrix of the input image
 */
std::vector<cv::Vec2i> CenterOfBrightness::extractBrightPixels(cv::Mat image)
{
    cv::Mat blured;
    std::vector<cv::Vec2i> locations;

    /*! - Grayscale, blur, and threshold iamge*/
    cv::cvtColor(image, this->imageGray, cv::COLOR_BGR2GRAY);
    cv::blur(this->imageGray, blured, cv::Size(this->blurSize,this->blurSize) );
    cv::threshold(blured, image, this->threshold, 255, cv::THRESH_BINARY);

    /*! - Find all the non-zero pixels in the image*/
    cv::findNonZero(image, locations);

    return locations;
}

/*! Method computes the weighted center of brightness and total brightness out of the non-zero pixel coordinates.
 @return std::pair<Eigen::Vector2d, double> First: center of brightness, Second: brightness
 @param vector integer pixel coordinates of bright pixels
 */
std::pair<Eigen::Vector2d, double> CenterOfBrightness::computeWeightedCenterOfBrightness(std::vector<cv::Vec2i> nonZeroPixels)
{
    uint32_t weightSum = 0;
    Eigen::Vector2d coordinates;
    coordinates.setZero();
    for(auto & pixel : nonZeroPixels) {
        /*! Individual pixel intensity used as the weight for the contribution to the solution*/
        auto weight = this->imageGray.at<unsigned char>(pixel[1], pixel[0]);
        coordinates[0] += weight * pixel[0];
        coordinates[1] += weight * pixel[1];
        weightSum += weight; // weighted sum of all the pixels
    }
    double brightness = weightSum / 255.0;  // normalized
    coordinates /= weightSum;

    return {coordinates, brightness};
}

/*! Update brightness history by shifting back previous brightness values and updating most recent one
    @return void
    @param brightness total brightness of current time step
    */
void CenterOfBrightness::updateBrightnessHistory(double brightness)
{
    // increase vector size if it is not at its full size yet
    if (this->brightnessHistory.rows() < this->numberOfPointsBrightnessAverage) {
        this->brightnessHistory.conservativeResize(this->brightnessHistory.rows() + 1, 1);
    }
    // shift previous brightness values back (only if number of data points for rolling average is greater than 1)
    if (this->brightnessHistory.rows() > 1) {
        for (auto i = static_cast<int>(this->brightnessHistory.rows())-1; i > 0; --i) {
            this->brightnessHistory[i] = this->brightnessHistory[i-1];
        }
    }
    // update most recent brightness value
    this->brightnessHistory[0] = brightness;
}

/*! This method applies the window for windowing by setting anything outside the window to black.
 @return void
 @param image cv::Mat of the input image
 */
void CenterOfBrightness::applyWindow (cv::Mat const &image) const
{
    /*! Create a window and ignore anything outside of it (make it black).
     * Point in opencv is column, row. x goes left-to-right, y goes top-to-bottom ([0,0] is top left corner).
     * Window mask is inclusive (edge of mask should be considered in COB), so must add/subtract one pixel. */
    /*! - Left edge removal */
    if (this->windowPointTopLeft[0] > 0) {
        cv::rectangle(image,
                      cv::Point(0, 0),
                      cv::Point(this->windowPointTopLeft[0]-1, image.size().height),
                      cv::Scalar(0),
                      -1);
    }
    /*! - Right edge removal */
    if (this->windowPointBottomRight[0] < image.size().width) {
        cv::rectangle(image,
                      cv::Point(this->windowPointBottomRight[0]+1, 0),
                      cv::Point(image.size().width, image.size().height),
                      cv::Scalar(0),
                      -1);
    }
    /*! - Top edge removal */
    if (this->windowPointTopLeft[1] > 0) {
        cv::rectangle(image,
                      cv::Point(this->windowPointTopLeft[0]-1, 0),
                      cv::Point(this->windowPointBottomRight[0]+1, this->windowPointTopLeft[1]-1),
                      cv::Scalar(0),
                      -1);
    }
    /*! - Bottom edge removal */
    if (this->windowPointBottomRight[1] < image.size().height) {
        cv::rectangle(image,
                      cv::Point(this->windowPointTopLeft[0]-1, this->windowPointBottomRight[1]+1),
                      cv::Point(this->windowPointBottomRight[0]+1, image.size().height),
                      cv::Scalar(0),
                      -1);
    }
}

/*! This method computes the points of the window used for windowing
 @return void
 @param image openCV matrix of the input image
 */
void CenterOfBrightness::computeWindow(cv::Mat const &image)
{
    // if any of the window parameters is 0 (not specified), window is the same as image dimensions and won't be applied
    if (this->windowCenter.isZero() || this->windowWidth == 0 || this->windowHeight == 0) {
        this->windowPointTopLeft[0] = 0;
        this->windowPointTopLeft[1] = 0;
        this->windowPointBottomRight[0] = image.size().width;
        this->windowPointBottomRight[1] = image.size().height;
    } else {
        this->windowPointTopLeft[0] = this->windowCenter[0] - this->windowWidth/2;
        this->windowPointTopLeft[1] = this->windowCenter[1] - this->windowHeight/2;
        this->windowPointBottomRight[0] = this->windowCenter[0] + this->windowWidth/2;
        this->windowPointBottomRight[1] = this->windowCenter[1] + this->windowHeight/2;
        this->validWindow = true;
    }
    assert(windowPointTopLeft[0] >= 0);
    assert(windowPointTopLeft[1] >= 0);
    assert(windowPointBottomRight[0] <= image.size().width);
    assert(windowPointBottomRight[1] <= image.size().height);
}

/*! Set the mask center for windowing
    @param Eigen::Vector2i center [px]
    @return void
    */
void CenterOfBrightness::setWindowCenter(const Eigen::VectorXi& center)
{
    this->windowCenter = center;
}

/*! Get the mask center for windowing
    @return Eigen::Vector2i center [px]
    */
Eigen::VectorXi CenterOfBrightness::getWindowCenter() const
{
    return this->windowCenter;
}

/*! Set the mask size for windowing
    @param int32_t width [px]
    @param int32_t height [px]
    @return void
    */
void CenterOfBrightness::setWindowSize(const int32_t width, const int32_t height)
{
    this->windowWidth = width;
    this->windowHeight = height;
}

/*! Get the mask center for windowing
    @return Eigen::Vector2i size [px]
    */
Eigen::VectorXi CenterOfBrightness::getWindowSize() const
{
    Eigen::VectorXi center = {this->windowWidth, this->windowHeight};
    return center;
}

/*! Set threshold for the increase in brightness for images not to be invalidated
    @param double increaseThreshold
    @return void
    */
void CenterOfBrightness::setRelativeBrightnessIncreaseThreshold(double increaseThreshold)
{
    this->relativeBrightnessIncreaseThreshold = increaseThreshold;
}

/*! Get threshold for the increase in brightness for images not to be invalidated
    @return double increaseThreshold
    */
double CenterOfBrightness::getRelativeBrightnessIncreaseThreshold() const
{
    return this->relativeBrightnessIncreaseThreshold;
}
