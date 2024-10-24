/*
 Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "cielimInterface.h"

/*! Reset module time-tags, message freshness, and zmq connector
 @param currentSimNanos The current sim time in nanoseconds
*/
void CielimInterface::Reset(uint64_t currentSimNanos) {
    if (this->opNavMode != ClosedLoopMode::OPEN_LOOP || this->liveStream) {
        this->imagePointer = nullptr;
        if (!this->connector.isConnected()) this->connector.connect();
    }

    /*! Check spacecraft input message */
    if (this->spacecraftMessage.isLinked()) {
        this->spacecraftMessageStatus.dataFresh = false;
        this->spacecraftMessageStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    }

    /*! Check Camera input messages */
    if (this->cameraModelMessage.isLinked()) {
        this->cameraModelMessageStatus.dataFresh = false;
        this->cameraModelMessageStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    }

    /*! Check Camera rendering information messages */
    if (this->cameraRenderingMessage.isLinked()) {
        this->cameraRenderingMessageStatus.dataFresh = false;
        this->cameraRenderingMessageStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    }

    /*! Check asteroid parameter information messages */
    if (this->celestialParametersMessage.isLinked()) {
        this->celestialParametersMessageStatus.dataFresh = false;
        this->celestialParametersMessageStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    }

        /*! Check asteroid parameter information messages */
    if (this->epochMessage.isLinked()) {
        this->epochMessageStatus.dataFresh = false;
        this->epochMessageStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    }


    this->epochMessageStatus.dataFresh = false;

    /*! Check Spice input message */
    MessageStatus spiceStatus;
    spiceStatus.dataFresh = true;
    spiceStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    this->spiceBodyMessageStatus.clear();
    for (int c = 0; c<this->celestialBodiesList.size(); ++c) {
        /*! set default zero translation and rotation states */
        SpicePlanetStateMsgPayload logMsg = {};
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                logMsg.J20002Pfix[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
        strcpy(logMsg.PlanetName, this->celestialBodiesList.at(c).name.c_str());
        this->spiceBodyMessageStatus.push_back(spiceStatus);
        this->spiceBodyPayloads.push_back(logMsg);
    }

    this->frameNumber = -1;
    if (this->saveFile){
        std::string temporaryFilename = this->protoFilename;
        for (int i=1; i<1000; ++i){
            if (!std::filesystem::exists(temporaryFilename)){
                this->protoFilename = temporaryFilename;
                break;
            }
            temporaryFilename = this->protoFilename + "_" + std::to_string(i);
        }
        this->outputStream = std::ofstream(this->protoFilename, std::ios_base::app | std::ios_base::binary);
    }
}

/*! Read all the relevant Basilisk messages in order to store the data locally before packaging in a protobuffer
 @return void
 * */
void CielimInterface::readBskMessages() {
    /*! Read spacecraft state msg */
    if (this->spacecraftMessage.isLinked()) {
        SCStatesMsgPayload localSCStateArray = this->spacecraftMessage();
        if (this->spacecraftMessage.isWritten() && this->spacecraftMessage.timeWritten() != this->spacecraftMessageStatus.lastTimeTag) {
            this->spacecraftMessageStatus.lastTimeTag = this->spacecraftMessage.timeWritten();
            this->spacecraftMessageStatus.dataFresh = true;
        }
        this->spacecraftPayload = localSCStateArray;
    }

    /*! Read camera model message */
    if (this->cameraModelMessage.isLinked()) {
        CameraModelMsgPayload localCameraConfigArray = this->cameraModelMessage();
        if (this->cameraModelMessage.isWritten() &&
                this->cameraModelMessage.timeWritten() != this->cameraModelMessageStatus.lastTimeTag) {
            this->cameraModelMessageStatus.lastTimeTag = this->cameraModelMessage.timeWritten();
            this->cameraModelMessageStatus.dataFresh = true;
        }
        this->cameraModelPayload = localCameraConfigArray;
    }

    /*! Read camera rendering message */
    if (this->cameraRenderingMessage.isLinked()) {
        CameraRenderingMsgPayload cameraRenderingArray = this->cameraRenderingMessage();
        if (this->cameraRenderingMessage.isWritten() &&
                this->cameraRenderingMessage.timeWritten() != this->cameraRenderingMessageStatus.lastTimeTag) {
            this->cameraRenderingMessageStatus.lastTimeTag = this->cameraRenderingMessage.timeWritten();
            this->cameraRenderingMessageStatus.dataFresh = true;
        }
        this->cameraRenderingPayload = cameraRenderingArray;
    }

    /*! Read celestial body parameter message */
    if (this->celestialParametersMessage.isLinked()) {
        CelestialBodyParametersMsgPayload celestialParamArray = this->celestialParametersMessage();
        if (this->celestialParametersMessage.isWritten() &&
                this->celestialParametersMessage.timeWritten() != this->celestialParametersMessageStatus.lastTimeTag) {
            this->celestialParametersMessageStatus.lastTimeTag = this->celestialParametersMessage.timeWritten();
            this->celestialParametersMessageStatus.dataFresh = true;
        }
        this->celestialParametersPayload = celestialParamArray;
    }

    /*! Read sim epoch msg */
    if (this->epochMessage.isLinked()) {
        EpochMsgPayload epochMessageBuffer  = this->epochMessage();
        if (this->epochMessage.isWritten() && this->epochMessage.timeWritten() != this->epochMessageStatus.lastTimeTag) {
            this->epochMessageStatus.lastTimeTag = this->epochMessage.timeWritten();
            this->epochMessageStatus.dataFresh = true;
        }
        this->epochPayload = epochMessageBuffer;
    }

    /*! Read celestial bodies messages */
    for (size_t i = 0; i < this->celestialBodiesList.size(); ++i) {
        if (this->celestialBodiesList.at(i).spiceStateMessage.isLinked()) {
            // If the spice msg is not linked then the default zero planet ephemeris is used
            SpicePlanetStateMsgPayload localSpiceArray = this->celestialBodiesList.at(i).spiceStateMessage();
            if (this->celestialBodiesList.at(i).spiceStateMessage.isWritten() &&
            this->celestialBodiesList.at(i).spiceStateMessage.timeWritten() != this->spiceBodyMessageStatus[i].lastTimeTag) {
                this->spiceBodyMessageStatus[i].lastTimeTag = this->celestialBodiesList.at(i).spiceStateMessage.timeWritten();
                this->spiceBodyMessageStatus[i].dataFresh = true;
                this->celestialBodiesList.at(i).spiceStatePayload = localSpiceArray;
            }
        }
    }

}

/*! Write a protobuffer with the information from the simulation .
 @param currentSimNanos The current sim time in nanoseconds
 */
void CielimInterface::writeProtobuffer(uint64_t currentSimNanos) {
    auto visPayload = cielimMessage::CielimMessage();

    /*! Write timestamp output msg */
    auto* time = new cielimMessage::TimeStamp();
    time->set_framenumber(this->frameNumber);
    time->set_simtimeelapsed((double) currentSimNanos);
    visPayload.set_allocated_currenttime(time);

    /*! write epoch msg */
    if (this->epochMessageStatus.dataFresh) {
        auto * epoch =  new cielimMessage::EpochDateTime();
        epoch->set_year(this->epochPayload.year);
        epoch->set_month(this->epochPayload.month);
        epoch->set_day(this->epochPayload.day);
        epoch->set_hours(this->epochPayload.hours);
        epoch->set_minutes(this->epochPayload.minutes);
        epoch->set_seconds(this->epochPayload.seconds);
        visPayload.set_allocated_epoch(epoch);
        this->epochMessageStatus.dataFresh = false;
    }

    /*! Write spice output msgs */
    for (int k = 0; k < this->spiceBodyMessageStatus.size(); ++k) {
        if (this->spiceBodyMessageStatus[k].dataFresh) {
            cielimMessage::CelestialBody *spice = visPayload.add_celestialbodies();
            spice->set_bodyname(this->celestialBodiesList.at(k).name);
            for (int i = 0; i < 3; i++) {
                spice->add_position(this->celestialBodiesList.at(k).spiceStatePayload.PositionVector[i]);
                spice->add_velocity(this->celestialBodiesList.at(k).spiceStatePayload.VelocityVector[i]);

                spice->add_attitude(this->celestialBodiesList.at(k).spiceStatePayload.J20002Pfix[i][0]);
                spice->add_attitude(this->celestialBodiesList.at(k).spiceStatePayload.J20002Pfix[i][1]);
                spice->add_attitude(this->celestialBodiesList.at(k).spiceStatePayload.J20002Pfix[i][2]);
            }
            spice->set_centralbody(this->celestialBodiesList.at(k).isCentralBody);
            std::string parameterBodyName = this->celestialParametersPayload.bodyName;
            if (this->celestialParametersMessage.isLinked() && this->celestialParametersMessageStatus.dataFresh
            && !parameterBodyName.compare(this->celestialBodiesList.at(k).name)) {
                auto *celestialParameters = new cielimMessage::CelestialModel();
                std::string brdfModelName = this->celestialParametersPayload.brdf;
                celestialParameters->set_brdfmodel(brdfModelName);
                celestialParameters->set_meanradius(this->celestialParametersPayload.meanRadius);
                celestialParameters->set_perlinnoisestddeviation(this->celestialParametersPayload.perlinNoise);
                for (int i = 0; i < 3; ++i) {
                    celestialParameters->add_principalaxisdistortion(
                            this->celestialParametersPayload.principalAxisDistortion[i]);
                }
                celestialParameters->set_proceduralrocks(this->celestialParametersPayload.proceduralRocks);
                for (int i = 0; i < MAX_PARAMETER_LENGTH; ++i) {
                    celestialParameters->add_reflectanceparameters(this->celestialParametersPayload.reflectanceParameters[i]);
                }
                celestialParameters->set_shapemodel(this->celestialParametersPayload.shapeModel);
                spice->set_allocated_models(celestialParameters);
        }
        }
    }

    /*! Write spacecraft state output msg */
    if (this->spacecraftMessage.isLinked() && this->spacecraftMessageStatus.dataFresh) {
        auto * spacecraft =  new cielimMessage::Spacecraft();
        spacecraft->set_spacecraftname("cielim_sat");
        for (int i = 0; i < 3; i++) {
            spacecraft->add_position(this->spacecraftPayload.r_BN_N[i]);
            spacecraft->add_velocity(this->spacecraftPayload.v_BN_N[i]);
            spacecraft->add_attitude(this->spacecraftPayload.sigma_BN[i]);
        }
        visPayload.set_allocated_spacecraft(spacecraft);
    }
    /*! Write camera output msg */
    if ((this->cameraModelMessage.isLinked() && this->cameraModelMessageStatus.dataFresh)
        || this->cameraModelPayload.cameraId >= 0) {
        /*! This corrective attitude allows UE to place the camera as is expected by the python setting.
         * UE5 has a -x pointing camera, with z vertical on the sensor, and y horizontal which is not the OpNav frame:
         * z point, x horizontal, y vertical (down) */
        auto * camera = new cielimMessage::CameraModel();
        for (int j = 0; j < 2; j++) {
            camera->add_resolution(this->cameraModelPayload.resolution[j]);
            camera->add_fieldofview(this->cameraModelPayload.fieldOfView[j]);
        }
        for (int j = 0; j < 3; j++) {
            camera->add_bodyframetocameramrp(this->cameraModelPayload.bodyToCameraMrp[j]);
            camera->add_camerapositioninbody(this->cameraModelPayload.cameraBodyFramePosition[j]);
        }
        camera->set_renderrate(this->cameraModelPayload.renderRate);
        camera->set_cameraid(this->cameraModelPayload.cameraId);
        camera->set_parentname(this->cameraModelPayload.parentName);
        camera->set_focallength(this->cameraModelPayload.focalLength);
        camera->set_pointspreadfunction(this->cameraModelPayload.gaussianPointSpreadFunction);
        camera->set_exposuretime(this->cameraModelPayload.exposureTime);
        camera->set_readnoise(this->cameraModelPayload.readNoise);
        camera->set_systemgain(this->cameraModelPayload.systemGain);

        if (this->cameraRenderingMessage.isLinked() && this->cameraRenderingMessageStatus.dataFresh
            && this->cameraRenderingPayload.cameraId == this->cameraModelPayload.cameraId){
            auto * rendering = new cielimMessage::RenderingModel();
            rendering->set_cosmicraystddeviation(this->cameraRenderingPayload.cosmicRayStdDeviation);
            rendering->set_enablestraylight(this->cameraRenderingPayload.enableStrayLight);
            rendering->set_starfield(this->cameraRenderingPayload.starField);
            rendering->set_rendering(this->cameraRenderingPayload.rendering);
            rendering->set_enablesmear(this->cameraRenderingPayload.smear);
            camera->set_allocated_renderparameters(rendering);
        }
        visPayload.set_allocated_camera(camera);
    }

    /*! Enter in lock-step with the vizard to simulate a camera */
    /*!--OpNavMode set to ALL_FRAMES is to stay in lock-step with the viz at all time steps. It is a slower run,
     * but provides visual capabilities during OpNav */
    /*!--OpNavMode set to REQUESTED_FRAMES is a faster mode in which the viz only steps forward to the BSK time step
     * if an image is requested. This is a faster run but nothing can be visualized post-run */
    if (this->opNavMode == ClosedLoopMode::ALL_FRAMES ||
        (this->opNavMode == ClosedLoopMode::REQUESTED_FRAMES && this->shouldRequestACameraImage(currentSimNanos)) ||
        this->liveStream) {
        this->connector.send(visPayload);

        /*! - If the camera is requesting periodic images, request them */
        if (this->opNavMode != ClosedLoopMode::OPEN_LOOP &&
            currentSimNanos % this->cameraModelPayload.renderRate == 0 &&
            this->cameraModelPayload.isOn == 1) {
            this->requestImage(currentSimNanos);

        }
        if (this->shouldRequestACameraImage(currentSimNanos)) {
            this->connector.ping();
        }
    }

    /*!  Write protobuf to file */
    if (this->saveFile) {
        google::protobuf::util::SerializeDelimitedToOstream(visPayload, &this->outputStream);
    }
}

/*! UpdateState
 @param currentSimNanos The current sim time
 */
void CielimInterface::UpdateState(uint64_t currentSimNanos) {
    this->frameNumber += 1;
    this->readBskMessages();
    if (currentSimNanos > 0) {
        this->writeProtobuffer(currentSimNanos);
    }
}

/*! Determine if the module should request and image
 @param currentSimNanos The current sim time
 * */
bool CielimInterface::shouldRequestACameraImage(uint64_t currentSimNanos) const{
    if (currentSimNanos % this->cameraModelPayload.renderRate == 0 &&
        this->cameraModelPayload.isOn == 1 /*|| this->firstPass < 11*/) {
        return true;
    }
    return false;
}

/*! Issue an image request
 @param currentSimNanos The current sim time
 * */
void CielimInterface::requestImage(uint64_t currentSimNanos) {
    auto imageData = this->connector.requestImage(this->cameraModelPayload.cameraId);
    this->imagePointer = &imageData.imageBuffer;

    CameraImageMsgPayload imagePayload = {};
    imagePayload.timeTag = currentSimNanos;
    imagePayload.valid = 0;
    imagePayload.imagePointer = imageData.imageBuffer;
    imagePayload.imageBufferLength = imageData.imageBufferLength;
    imagePayload.cameraID = this->cameraModelPayload.cameraId;
    imagePayload.imageType = 3;
    if (imageData.imageBufferLength > 0) { imagePayload.valid = 1; }
    this->imageOutMessage.write(&imagePayload, this->moduleID, currentSimNanos);
}

/*! Get the communication mode
@return ClosedLoopMode enum containing the mode definitions

*/
ClosedLoopMode CielimInterface::getOpNavMode() const {
    return this->opNavMode;
}

/*! Set the communication mode
 * @param ClosedLoopMode enum containing the mode definitions
*/
void CielimInterface::setOpNavMode(ClosedLoopMode mode) {
    this->opNavMode = mode;
}

/*! Set the tcp port number
 * @param std::string port number
*/
void CielimInterface::setPortNumber(std::string port) {
    this->connector.setComPortNumber(port);
}

/*! Get the frame number
 * @return int64_t frame
*/
int64_t CielimInterface::getFrameNumber() const {
    return this->frameNumber;
}

/*! Set the save file path
 * @param std::string path and name to data destination
*/
void CielimInterface::setSaveFile(const std::string &saveProtobufferFile) {
    assert(saveProtobufferFile != "");
    this->protoFilename = saveProtobufferFile;
    this->saveFile = true;
}

/*! Get the save file path
 * @return std::string path and name to data destination
*/
std::string CielimInterface::getSaveFilename() const {
    return this->protoFilename;
}

/*! Manually close the save file for read/write control
*/
void CielimInterface::closeProtobufFile() {
    this->outputStream.close();
}

/*! Toggle live streaming on or off
@param bool on/off
*/
void CielimInterface::setLiveStream(bool liveStreaming){
    this->liveStream = liveStreaming;
}

/*! Add a celestial body to the interface
@param SpiceBody class containing celestial body information
*/
void CielimInterface::addCelestialBody(const SpiceBody &celestialBodyNames){
    this->celestialBodiesList.push_back(celestialBodyNames);
}

/*! Get all celestial body added to the interface
@return std::vector<SpiceBody> list of bodies
*/
std::vector<SpiceBody> CielimInterface::getCelestialBodies() const{
    return this->celestialBodiesList;
}

/*! A cleaning method to ensure the message buffers are wiped clean.
 @param data The current sim time in nanoseconds
 @param hint
 */
void message_buffer_deallocate(void *data, void *hint) {
    free(data);
}
