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

#ifndef PART_H
#define PART_H

#include "simulation/dynamics/KinematicsArchitecture/Frame.h"
#include "simulation/dynamics/KinematicsArchitecture/Point.h"
#include "simulation/dynamics/KinematicsArchitecture/Tensor.h"
#include "simulation/dynamics/KinematicsArchitecture/Vector.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"

#include <Eigen/Core>

/*! @brief basic Basilisk C++ module class */
class Part {
public:
    Part() = default;
    explicit Part(std::shared_ptr<Frame> frame);
    ~Part() = default;

    double mass = 0.0;
    InertiaTensor IPntSc_S;
    std::shared_ptr<Translation> r_ScS;
    std::shared_ptr<Point> CoMPoint;
    std::shared_ptr<Frame> frame;

    Message<SCStatesMsgPayload> bodyStateOutMsg;
};

#endif
