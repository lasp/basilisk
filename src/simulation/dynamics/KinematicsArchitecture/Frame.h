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

#include "simulation/dynamics/KinematicsArchitecture/Point.h"
#include "simulation/dynamics/KinematicsArchitecture/Vector.h"

#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"

#include <Eigen/Dense>
#include <utility>
#include <memory>

/*! @brief basic Basilisk C++ module class */
class Frame {
public:
    Frame() = default;
    explicit Frame(std::shared_ptr<Frame> parentFrame, std::shared_ptr<Point> point) : parentFrame(std::move(parentFrame)), originPoint(std::move(point)) {};
    ~Frame() = default;

    std::shared_ptr<Frame> parentFrame = nullptr;
    std::shared_ptr<Point> originPoint = nullptr;
    std::string tag = "root";

    std::shared_ptr<Rotation> sigma_SP = nullptr;
    std::shared_ptr<Translation> r_SP = nullptr;

    std::shared_ptr<Point> getOriginPoint() const;

    void setParentFrame(std::shared_ptr<Frame> newParentFrame) {this->parentFrame = std::move(newParentFrame);};
    void updateFrame() {};
};


#endif
