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

#ifndef FRAME_H
#define FRAME_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

#include "simulation/dynamics/_GeneralModuleFiles/Vector.h"
#include "simulation/dynamics/_GeneralModuleFiles/AttitudeParameterization.h"
#include <Eigen/Core>

/*! @brief basic Basilisk C++ module class */
class Frame {
public:
    Frame() = default;
    Frame(Frame* parentFrame);
    Frame(Frame* parentFrame,
          const MRP& sigma_CP,
          const Eigen::Vector3d& omega_CP_C,
          const Eigen::Vector3d& omegaPrime_CP_C,
          const Eigen::Vector3d& r_CP_P,
          const Eigen::Vector3d& rPrime_CP_P,
          const Eigen::Vector3d& rPPrime_CP_P);
    ~Frame() = default;

    BSKLogger bskLogger;              //!< -- BSK Logging

    Frame* parentFrame = nullptr;
    std::string tag;

    MRP sigma_CP;
    Vector omega_CP;
    Vector r_CP;

    void setParentFrame(Frame* newParentFrame) {this->parentFrame = newParentFrame;};
};


#endif
