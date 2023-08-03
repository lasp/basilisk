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

#include "simulation/dynamics/KinematicsArchitecture/Frame.h"
#include "simulation/dynamics/KinematicsArchitecture/Point.h"
#include <Eigen/Core>

class Tensor {
public:
    Tensor() = default;
    ~Tensor() = default;
    Tensor(Eigen::Matrix3d newMatrix, const std::shared_ptr<Frame>& newWrittenFrame); // do we change this to const std::shared_ptr<const Frame>&?

    void set(Eigen::Matrix3d newMatrix, const std::shared_ptr<Frame>& newWrittenFrame);
    Eigen::Matrix3d getMatrix(const std::shared_ptr<Frame>& frame) const;
    std::shared_ptr<Frame> getWrittenFrame() const;

    Tensor operator+(const Tensor& tensor) const;
    void operator+=(const Tensor& tensor);
    Tensor operator-() const;
    Tensor operator-(const Tensor& tensor) const;
    void operator-=(const Tensor& tensor);
    Vector operator*(const Vector& vector) const;
    Tensor operator*(const Tensor& tensor) const;
    void operator*=(const Tensor& tensor);
    Tensor operator*(double factor) const;
    void operator*=(double factor);
    Tensor operator/(double factor) const;
    void operator/=(double factor);
    Tensor inverse() const;
    Tensor transpose() const;

private:
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
    std::shared_ptr<Frame> writtenFrame;
};

Tensor operator*(double factor, const Tensor& tensor);  // this is throwing a warning

struct TensorDerivativeProperties {
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();

    std::weak_ptr<Frame> writtenFrame;
    std::weak_ptr<Frame> derivFrame;
};

class InertiaTensor: public Tensor {
public:
    InertiaTensor() = default;
    explicit InertiaTensor(std::shared_ptr<Point> point);
    ~InertiaTensor() = default;

    void setFirstOrder(const Eigen::Matrix3d& newMatrix, const std::shared_ptr<Frame>& newWrittenFrame, const std::shared_ptr<Frame>& newDerivFrame);
    void setPoint(const std::shared_ptr<Point>& newPoint);
    std::shared_ptr<Point> getPoint() const;

    std::shared_ptr<Point> point;
    TensorDerivativeProperties firstOrder;
};

#endif
