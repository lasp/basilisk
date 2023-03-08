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

#ifndef ATTITUDE_PARAMETERIZATION_H
#define ATTITUDE_PARAMETERIZATION_H

#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

#include <Eigen/Core>

/*! @brief basic Basilisk C++ module class */
class MRP {
public:
    MRP() = default;
    explicit MRP(Eigen::Vector3d cmpnts) : components(std::move(cmpnts)) {};
    ~MRP() = default;

    BSKLogger bskLogger;              //!< -- BSK Logging

    Eigen::Vector3d components = Eigen::Vector3d::Zero();
};


#endif
