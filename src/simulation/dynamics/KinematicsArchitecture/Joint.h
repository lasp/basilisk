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

#ifndef JOINT_H
#define JOINT_H

#include "simulation/dynamics/KinematicsArchitecture/Frame.h"
#include "simulation/dynamics/KinematicsArchitecture/Hinge.h"
#include <vector>
#include <Eigen/Core>

/*! @brief basic Basilisk C++ module class */
class Joint {
public:
    Joint() = default;
    ~Joint() = default;

    std::shared_ptr<Frame> lowerFrame;
    std::shared_ptr<Frame> upperFrame;

    std::vector<std::shared_ptr<Hinge>> hingeVector;
};


class RotaryOneDOF : public Joint {
public:
    explicit RotaryOneDOF(std::shared_ptr<Hinge> hinge);
    ~RotaryOneDOF() = default;
};


class RotaryTwoDOF : public Joint {
public:
    RotaryTwoDOF(std::shared_ptr<Hinge> firstHinge, std::shared_ptr<Hinge> secondHinge);
    ~RotaryTwoDOF() = default;
};

#endif
