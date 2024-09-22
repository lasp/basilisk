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

    /* Check spacecraft input message */
    if (this->spacecraftMessage.isLinked()) {
        this->spacecraftMessageStatus.dataFresh = false;
        this->spacecraftMessageStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    }

    /* Check Camera input messages */
    if (this->cameraModelMessage.isLinked()) {
        this->cameraModelMessageStatus.dataFresh = false;
        this->cameraModelMessageStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    }

    this->epochMessageStatus.dataFresh = false;

    /* Check Spice input message */
    MessageStatus spiceStatus;
    spiceStatus.dataFresh = true;
    spiceStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    this->spiceBodyMessageStatus.clear();

    this->frameNumber = -1;
    if (this->saveFile) {
        this->outputStream = new std::ofstream(this->protoFilename, std::ios::out | std::ios::binary);
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
        CameraConfigMsgPayload localCameraConfigArray = this->cameraModelMessage();
        if (this->cameraModelMessage.isWritten() &&
                this->cameraModelMessage.timeWritten() != this->cameraModelMessageStatus.lastTimeTag) {
            this->cameraModelMessageStatus.lastTimeTag = this->cameraModelMessage.timeWritten();
            this->cameraModelMessageStatus.dataFresh = true;
        }
        this->cameraModelPayload = localCameraConfigArray;
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
    for (size_t i = 0; i < this->spiceBodyMessages.size(); i++) {
        if (this->spiceBodyMessages.at(i).isLinked()) {
            // If the spice msg is not linked then the default zero planet ephemeris is used
            SpicePlanetStateMsgPayload localSpiceArray = this->spiceBodyMessages.at(i)();
            if (this->spiceBodyMessages.at(i).isWritten() &&
                                    this->spiceBodyMessages.at(i).timeWritten() != this->spiceBodyMessageStatus[i].lastTimeTag) {
                this->spiceBodyMessageStatus[i].lastTimeTag = this->spiceBodyMessages.at(i).timeWritten();
                this->spiceBodyMessageStatus[i].dataFresh = true;
                this->spiceBodyPayloads[i] = localSpiceArray;
            }
        }
    }

}

/*! Write a protobuffer with the information from the simulation .
 @param currentSimNanos The current sim time in nanoseconds
 */
void CielimInterface::writeProtobuffer(uint64_t currentSimNanos) {
    auto *visPayload = std::unique_ptr<vizProtobufferMessage::VizMessage>;

    /*! Write timestamp output msg */
    auto *time = std::unique_ptr<vizProtobufferMessage::VizMessage::TimeStamp>;
    time->set_framenumber(this->frameNumber);
    time->set_simtimeelapsed((double) currentSimNanos);
    visPayload->set_allocated_currenttime(time);

    /*! write epoch msg */
    if (this->epochMessageStatus.dataFresh) {
        auto *epoch = std::unique_ptr<vizProtobufferMessage::VizMessage::EpochDateTime>;
        epoch->set_year(this->epochPayload.year);
        epoch->set_month(this->epochPayload.month);
        epoch->set_day(this->epochPayload.day);
        epoch->set_hours(this->epochPayload.hours);
        epoch->set_minutes(this->epochPayload.minutes);
        epoch->set_seconds(this->epochPayload.seconds);
        visPayload->set_allocated_epoch(epoch);
        this->epochMessageStatus.dataFresh = false;
        delete epoch;
    }

    /*! Write spacecraft state output msg */
    if (this->spacecraftMessage.isLinked() && this->spacecraftMessageStatus.dataFresh) {
        vizProtobufferMessage::VizMessage::Spacecraft *scp = visPayload->add_spacecraft();
        scp->set_spacecraftname(this->spacecraftPayload.spacecraftName);
        for (int i = 0; i < 3; i++) {
            scp->add_position(this->spacecraftPayload.scStateMsgBuffer.r_BN_N[i]);
            scp->add_velocity(this->spacecraftPayload.scStateMsgBuffer.v_BN_N[i]);
            scp->add_rotation(this->spacecraftPayload.scStateMsgBuffer.sigma_BN[i]);
        }
        visPayload->set_allocated_spacecraft(spacecraft);
        delete spacecraft;
    }
    /*! Write camera output msg */
    if ((this->cameraModelMessage.isLinked() && this->cameraModelMessageStatus.dataFresh)
        || this->cameraModelPayload.cameraID >= 0) {
        /*! This corrective rotation allows UE to place the camera as is expected by the python setting.
         * UE5 has a -x pointing camera, with z vertical on the sensor, and y horizontal which is not the OpNav frame:
         * z point, x horizontal, y vertical (down) */
        Eigen::Vector3d sigma_CuC << 1. / 3, 1. / 3, -1. / 3;
        Eigen::Vector3d unityCameraMrp = addMrp(Eigen::Map<Eigen::Vector3d>(this->cameraModelPayload.sigma_CB, 3, 1),
                sigma_CuC); /*! Cu is the unity Camera frame */
        vizProtobufferMessage::VizMessage::CameraConfig *camera = visPayload->add_cameras();
        for (int j = 0; j < 2; j++) {
            camera->add_resolution(this->cameraModelPayload.resolution[j]);
            camera->add_fieldofview(this->cameraModelPayload.fieldOfView * 180/M_PI); // UE expects degrees
        }
        for (int j = 0; j < 3; j++) {
            camera->add_bodyframetocameramrp(gameCameraMrp[j]);
            camera->add_camerapositioninbody(this->cameraModelPayload.cameraPos_B[j]);
        }
        camera->set_renderrate(this->cameraModelPayload.renderRate); // UE expects nanoseconds between images
        camera->set_cameraid(this->cameraModelPayload.cameraID);
        camera->set_parentname(this->cameraModelPayload.parentName);
        camera->set_focallength(this->cameraModelPayload.ppFocalLength * 1000.0); // UE expects mm
        visPayload->set_allocated_camera(camera);
        delete camera;
    }

    /*! Write spice output msgs */
    if (this->spiceBodyMessageStatus[k].dataFresh) {
        vizProtobufferMessage::VizMessage::CelestialBody *spice = visPayload->add_celestialbodies();
        spice->set_bodyname(this->gravBodyInformation.at(k).bodyName);
        for (int i = 0; i < 3; i++) {
            spice->add_position(this->spiceBodyPayloads[k].PositionVector[i]);
            spice->add_velocity(this->spiceBodyPayloads[k].VelocityVector[i]);
            for (int j = 0; j < 3; j++) {
                spice->add_rotation(this->spiceBodyPayloads[k].J20002Pfix[i][j]);
            }
        }
    }

    google::protobuf::uint8 varIntBuffer[4];
    auto byteCount = (uint32_t) visPayload->ByteSizeLong();
    google::protobuf::uint8 const *end = google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(byteCount,
                                                                                                       varIntBuffer);

    /*! Enter in lock-step with the vizard to simulate a camera */
    /*!--OpNavMode set to ALL_FRAMES is to stay in lock-step with the viz at all time steps. It is a slower run,
     * but provides visual capabilities during OpNav */
    /*!--OpNavMode set to REQUESTED_FRAMES is a faster mode in which the viz only steps forward to the BSK time step
     * if an image is requested. This is a faster run but nothing can be visualized post-run */
    if (this->opNavMode == ClosedLoopMode::ALL_FRAMES ||
        (this->opNavMode == ClosedLoopMode::REQUESTED_FRAMES && this->shouldRequestACameraImage(currentSimNanos)) ||
        this->liveStream) {
        this->connector.send(*visPayload);

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
    auto varIntBytes = (unsigned long) (end - varIntBuffer);
    if (this->saveFile) {
        this->outputStream->write(reinterpret_cast<char * > (varIntBuffer), (int) varIntBytes);
    }
    if (!this->saveFile || !visPayload->SerializeToOstream(this->outputStream)) {
        return;
    }

    delete visPayload;

    google::protobuf::ShutdownProtobufLibrary();
}

/*! Update this module at the task rate
 @param currentSimNanos The current sim time
 */
void CielimInterface::UpdateState(uint64_t currentSimNanos) {
    this->frameNumber += 1;
    this->readBskMessages();
    if (currentSimNanos > 0) {
        this->writeProtobuffer(currentSimNanos);
    }
}

bool CielimInterface::shouldRequestACameraImage(uint64_t currentSimNanos) const{
    if (currentSimNanos % this->cameraModelPayload.renderRate == 0 &&
        this->cameraModelPayload.isOn == 1 /*|| this->firstPass < 11*/) {
        return true;
    }
    return false;
}

void CielimInterface::requestImage(uint64_t currentSimNanos) {
    auto imageData = this->connector.requestImage(this->cameraModelPayload.cameraID);
    this->imagePointer = &imageData.imageBuffer;

    CameraImageMsgPayload imagePayload = {};
    imagePayload.timeTag = currentSimNanos;
    imagePayload.valid = 0;
    imagePayload.imagePointer = imageData.imageBuffer;
    imagePayload.imageBufferLength = imageData.imageBufferLength;
    imagePayload.cameraID = this->cameraModelPayload.cameraID;
    imagePayload.imageType = 3;
    if (imageData.imageBufferLength > 0) { imagePayload.valid = 1; }
    this->imageOutMessage.write(&imagePayload, this->moduleID, currentSimNanos);
}

ClosedLoopMode CielimInterface::getOpNavMode() const {
    return this->opNavMode;
}

void CielimInterface::setOpNavMode(ClosedLoopMode mode) {
    this->opNavMode = mode;
}

int64_t CielimInterface::getFrameNumber() const {
    return this->frameNumber;
}

void CielimInterface::setSaveFile(const std::string saveProtobufferFile) {
    assert(saveProtobufferFile != "");
    this->protoFilename = saveProtobufferFile;
    this->saveFile = true;
}

void CielimInterface::setLiveStream(bool liveStreaming){
    this->liveStream = liveStreaming;
}

/*! A cleaning method to ensure the message buffers are wiped clean.
 @param data The current sim time in nanoseconds
 @param hint
 */
void message_buffer_deallocate(void *data, void *hint) {
    free(data);
}

