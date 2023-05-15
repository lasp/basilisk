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
KinematicsEngine::KinematicsEngine() :
    rootFrame(new Frame) {
};

Frame* KinematicsEngine::createFrame() {
    return new Frame(this->rootFrame);
}

Frame* KinematicsEngine::createFrame(Frame* parentFrame) {
    return new Frame(parentFrame);
}

Frame* KinematicsEngine::createFrame(Frame *parentFrame,
                                     const MRP &sigma_SP,
                                     const Eigen::Vector3d &omega_SP_S,
                                     const Eigen::Vector3d &omegaPrime_SP_S,
                                     const Eigen::Vector3d &r_SP_P,
                                     const Eigen::Vector3d &rPrime_SP_P,
                                     const Eigen::Vector3d &rPPrime_SP_P) {
    return new Frame(parentFrame, sigma_SP, omega_SP_S, omegaPrime_SP_S, r_SP_P, rPrime_SP_P, rPPrime_SP_P);
}

Part* KinematicsEngine::createPart() {
    return new Part(this->rootFrame);
}

Part* KinematicsEngine::createPart(Frame* parentFrame) {
    return new Part(parentFrame);
}

Part* KinematicsEngine::createPart(Frame *parentFrame,
                                     const MRP &sigma_SP,
                                     const Eigen::Vector3d &omega_SP_S,
                                     const Eigen::Vector3d &omegaPrime_SP_S,
                                     const Eigen::Vector3d &r_SP_P,
                                     const Eigen::Vector3d &rPrime_SP_P,
                                     const Eigen::Vector3d &rPPrime_SP_P) {
    return new Part(parentFrame, sigma_SP, omega_SP_S, omegaPrime_SP_S, r_SP_P, rPrime_SP_P, rPPrime_SP_P);
}
