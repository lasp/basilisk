/*
 ISC License

 Copyright (c) 2023 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "zmqConnector.h"

ZmqConnector::ZmqConnector() = default;

void ZmqConnector::connect() {
    if (!this->isConnected()) {
        this->context = std::make_shared<zmq::context_t>();
        this->requesterSocket = std::make_unique<zmq::socket_t>(*this->context, ZMQ_REQ);
        std::cout << this->comAddress << ":" << this->comPortNumber << std::endl;
        this->requesterSocket->connect(this->comProtocol + "://" + this->comAddress + ":" + this->comPortNumber);
    }
    this->ping();
    this->ping();
}

bool ZmqConnector::isConnected() const {
    if (this->requesterSocket) {
        return this->requesterSocket->connected();
    }
    return false;
}

void ZmqConnector::send(const cielimMessage::CielimMessage& message) {
    /*! - The viz needs 10 images before placing the planets, wait for 11 protobuffers to
     * have been created before attempting to go into opNavMode 2 */
    if (this->firstPass < 11){
        this->firstPass++;
    }

    /*! - send protobuffer raw over zmq_socket */
    size_t byteCount = message.ByteSizeLong();
    void* serialized_message = malloc(byteCount);
    message.SerializeToArray(serialized_message, (int)byteCount);
    auto payload = zmq::message_t(serialized_message, byteCount, ZmqConnector::message_buffer_deallocate, nullptr);

    auto emptyMsg = zmq::message_t(0);

    this->requesterSocket->send(zmq::message_t("SIM_UPDATE", 10), zmq::send_flags::sndmore);
    this->requesterSocket->send(emptyMsg, zmq::send_flags::sndmore);
    this->requesterSocket->send(emptyMsg, zmq::send_flags::sndmore);
    this->requesterSocket->send(payload, zmq::send_flags::none);

    // Receive pong
    auto pong = zmq::message_t();
    // SAFETY: it's okay to discard this [[nodiscard]] value because
    //   1) the returned optional could only be empty if ZeroMQ fails due to EAGAIN on a non-blocking socket;
    //      but our socket is not non-blocking
    //   2) the returned length in the (present) optional is recoverable from `pong.size()`.
    static_cast<void>(this->requesterSocket->recv(pong, zmq::recv_flags::none));
}

void ZmqConnector::message_buffer_deallocate(void *data, void *hint)
{
    free(data);
}

ImageData ZmqConnector::requestImage(size_t cameraId) {
    std::string cmdMsg = "REQUEST_IMAGE_";
    cmdMsg += std::to_string(cameraId);
    void* img_message = malloc(cmdMsg.length() * sizeof(char));
    memcpy(img_message, cmdMsg.c_str(), cmdMsg.length());

    auto imageRequestMessage = zmq::message_t(img_message,
                                              cmdMsg.length(),
                                              ZmqConnector::message_buffer_deallocate,
                                              nullptr);
    this->requesterSocket->send(imageRequestMessage, zmq::send_flags::none);

    // SAFETY: it's okay to discard these [[nodiscard]] values because
    //   1) the returned optional could only be empty if ZeroMQ fails due to EAGAIN on a non-blocking socket;
    //      but our socket is not non-blocking
    //   2) the returned length in the (present) optional is recoverable from the given message's `.size()` method.
    auto imageLengthMessage = zmq::message_t();
    auto imageMessage = zmq::message_t();
    static_cast<void>(this->requesterSocket->recv(imageLengthMessage, zmq::recv_flags::none));
    static_cast<void>(this->requesterSocket->recv(imageMessage, zmq::recv_flags::none));

    const int32_t *lengthPoint = imageLengthMessage.data<int32_t>();
    const void *imagePoint = imageMessage.data();
    int32_t imageBufferLength = *lengthPoint;
    void* image = malloc(imageBufferLength*sizeof(char));
    memcpy(image, imagePoint, imageBufferLength*sizeof(char));

    return  ImageData{imageBufferLength, image};
}


void ZmqConnector::ping() {
    this->requesterSocket->send(zmq::message_t("PING", 4), zmq::send_flags::none);
    auto message = zmq::message_t();
    // SAFETY: it's okay to discard this [[nodiscard]] value because
    //   1) the returned optional could only be empty if ZeroMQ fails due to EAGAIN on a non-blocking socket;
    //      but our socket is not non-blocking
    //   2) the returned length in the (present) optional is recoverable from `pong.size()`.
    static_cast<void>(this->requesterSocket->recv(message, zmq::recv_flags::none));
    std::cout << message.str() << std::endl;
}

void ZmqConnector::setComPortNumber(std::string &portNumber) {
    this->comPortNumber = portNumber;
}

