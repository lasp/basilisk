/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "KinematicsEngine.h"

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */

std::shared_ptr<Frame> KinematicsEngine::createFrame() {
    auto tempFrame  = std::make_shared<Frame>(this->rootFrame);

    tempFrame->omega_SP.writtenFrame = tempFrame;
    tempFrame->omega_SP.firstOrder.writtenFrame = tempFrame;
    tempFrame->omega_SP.firstOrder.derivFrame = tempFrame;

    tempFrame->r_SP.writtenFrame = this->rootFrame;
    tempFrame->r_SP.firstOrder.writtenFrame = this->rootFrame;
    tempFrame->r_SP.firstOrder.derivFrame = this->rootFrame;
    tempFrame->r_SP.secondOrder.writtenFrame = this->rootFrame;
    tempFrame->r_SP.secondOrder.derivFrame = this->rootFrame;

    return tempFrame;
}

std::shared_ptr<Frame> KinematicsEngine::createFrame(const std::shared_ptr<Frame>& parentFrame) {
    auto tempFrame = std::make_shared<Frame>(parentFrame);

    tempFrame->omega_SP.writtenFrame = tempFrame;
    tempFrame->omega_SP.firstOrder.writtenFrame = tempFrame;
    tempFrame->omega_SP.firstOrder.derivFrame = tempFrame;

    tempFrame->r_SP.writtenFrame = parentFrame;
    tempFrame->r_SP.firstOrder.writtenFrame = parentFrame;
    tempFrame->r_SP.firstOrder.derivFrame = parentFrame;
    tempFrame->r_SP.secondOrder.writtenFrame = parentFrame;
    tempFrame->r_SP.secondOrder.derivFrame = parentFrame;

    return tempFrame;
}

std::shared_ptr<Part> KinematicsEngine::createPart() {
    auto tempFrame = this->createFrame();
    return std::make_shared<Part>(std::move(tempFrame));
}

std::shared_ptr<Part> KinematicsEngine::createPart(const std::shared_ptr<Frame>& parentFrame) {
    auto tempFrame = KinematicsEngine::createFrame(parentFrame);
    return std::make_shared<Part>(std::move(tempFrame));
}
