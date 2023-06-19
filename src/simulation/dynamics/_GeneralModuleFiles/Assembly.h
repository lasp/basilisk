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

#ifndef ASSEMBLY_H
#define ASSEMBLY_H

#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "simulation/dynamics/_GeneralModuleFiles/AttitudeParameterization.h"
#include "simulation/dynamics/_GeneralModuleFiles/Part.h"
#include "simulation/dynamics/_GeneralModuleFiles/Joint.h"
#include "simulation/dynamics/_GeneralModuleFiles/Frame.h"
#include "simulation/dynamics/_GeneralModuleFiles/Vector.h"
#include "simulation/dynamics/_GeneralModuleFiles/Tensor.h"
#include <Eigen/Core>
#include <utility>

/*! @brief basic Basilisk C++ module class */
class Assembly {
public:
    Assembly() = default;
    ~Assembly() = default;

    void addPart(std::shared_ptr<Part> part);

    std::vector<std::shared_ptr<Assembly>> assemblyList;
    std::vector<std::shared_ptr<Part>> partList;
    std::vector<std::shared_ptr<Joint>> jointList;

    std::string tag;
    std::shared_ptr<Frame> frame;
    double mass = 0.0;
    std::shared_ptr<InertiaTensor> IPntSc = nullptr;
    std::shared_ptr<InertiaTensor> IPntS = nullptr;
    std::shared_ptr<Point> CoMPoint = nullptr;
    std::shared_ptr<PositionVector> r_ScS = nullptr;
};


#endif
