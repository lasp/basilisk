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

#ifndef TENSOR_H
#define TENSOR_H

#include <Eigen/Core>

/*! @brief basic Basilisk C++ module class */
class Frame;  // Needs a forward declaration so it compiles

struct TensorProperties {
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
    std::shared_ptr<Frame> writtenFrame = nullptr;
    std::shared_ptr<Frame> derivFrame = nullptr;
};

class Tensor {
public:
    Tensor() = default;
    Tensor(Eigen::Matrix3d zerothMatrix, std::shared_ptr<Frame> zerothWrittenFrame,
           Eigen::Matrix3d firstMatrix, std::shared_ptr<Frame> firstWrittenFrame, std::shared_ptr<Frame> firstDerivFrame);
    ~Tensor() = default;

    TensorProperties zerothOrder;
    TensorProperties firstOrder;

};

#endif
