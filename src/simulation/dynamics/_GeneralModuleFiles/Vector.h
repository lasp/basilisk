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

#ifndef VECTOR_H
#define VECTOR_H

#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

#include <Eigen/Core>

/*! @brief basic Basilisk C++ module class */
class Frame;  // Needs a forward declaration so it compiles

class Vector {
public:
    Vector() = default;
    Vector(Eigen::Vector3d matrix, Frame* writtenFrame, Frame* derivFrame);
    ~Vector();

    Eigen::Vector3d matrix = Eigen::Vector3d::Zero();
    Frame* writtenFrame = nullptr;
    Frame* derivFrame = nullptr;

};

#endif
