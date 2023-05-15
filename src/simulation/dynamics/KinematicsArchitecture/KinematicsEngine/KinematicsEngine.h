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

#ifndef KINEMATICS_ENGINE_H
#define KINEMATICS_ENGINE_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "simulation/dynamics/_GeneralModuleFiles/Frame.h"
#include "simulation/dynamics/_GeneralModuleFiles/Part.h"

class KinematicsEngine: public SysModel {
public:
    KinematicsEngine();
    ~KinematicsEngine() override = default;

    BSKLogger bskLogger;              //!< -- BSK Logging

    Frame* createFrame();
    Frame* createFrame(Frame* parentFrame);
    Frame* createFrame(Frame* parentFrame,
                       const MRP& sigma_CP,
                       const Eigen::Vector3d& omega_CP_C,
                       const Eigen::Vector3d& omegaPrime_CP_C,
                       const Eigen::Vector3d& r_CP_P,
                       const Eigen::Vector3d& rPrime_CP_P,
                       const Eigen::Vector3d& rPPrime_CP_P);

    Part* createPart();
    Part* createPart(Frame* parentFrame);
    Part* createPart(Frame* parentFrame,
                       const MRP& sigma_CP,
                       const Eigen::Vector3d& omega_CP_C,
                       const Eigen::Vector3d& omegaPrime_CP_C,
                       const Eigen::Vector3d& r_CP_P,
                       const Eigen::Vector3d& rPrime_CP_P,
                       const Eigen::Vector3d& rPPrime_CP_P);

    Frame* rootFrame = nullptr;
};


#endif
