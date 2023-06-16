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
#include <Eigen/Core>

class Frame;
class Point;

/*! @brief basic Basilisk C++ module class */
class Vector {
public:
    Vector() = default;
    Vector(Eigen::Vector3d zerothMatrix, std::weak_ptr<Frame> zerothWrittenFrame);
    ~Vector() = default;

    void setZerothOrder(Eigen::Vector3d newMatrix, std::shared_ptr<Frame> newWrittenFrame);
    Eigen::Vector3d getZerothOrder(std::shared_ptr<Frame> newWrittenFrame);

    Eigen::Vector3d matrix = Eigen::Vector3d::Zero();
    std::weak_ptr<Frame> writtenFrame;
};


/*! @brief Vector derivative properties data structure */
struct VectorDerivativeProperties {
    Eigen::Vector3d matrix = Eigen::Vector3d::Zero();
    std::weak_ptr<Frame> writtenFrame;
    std::weak_ptr<Frame> derivFrame;
};


/*! @brief basic Basilisk C++ module class */
class PositionVector : public Vector {
public:
    PositionVector() = default;
    PositionVector(std::shared_ptr<Point> headPoint, std::shared_ptr<Point> tailPoint);
    ~PositionVector() = default;

    void setFirstOrder(Eigen::Vector3d newMatrix, const std::shared_ptr<Frame>& newWrittenFrame, const std::shared_ptr<Frame>& newDerivFrame);
    void setSecondOrder(Eigen::Vector3d newMatrix, const std::shared_ptr<Frame>& newWrittenFrame, const std::shared_ptr<Frame>& newDerivFrame);

    std::shared_ptr<Point> tailPoint;
    std::shared_ptr<Point> headPoint;

    VectorDerivativeProperties firstOrder;
    VectorDerivativeProperties secondOrder;
};


/*! @brief basic Basilisk C++ module class */
class ForceVector : public Vector {
public:
    ForceVector() = default;
    ~ForceVector() = default;

    std::weak_ptr<Point> applicationPoint;
};


/*! @brief basic Basilisk C++ module class */
class AngularVelocityVector : public Vector {
public:
    AngularVelocityVector() = default;
    AngularVelocityVector(std::weak_ptr<Frame> upperFrame, std::weak_ptr<Frame> lowerFrame);
    ~AngularVelocityVector() = default;

    void setFirstOrder(Eigen::Vector3d newMatrix, const std::shared_ptr<Frame>& newWrittenFrame, const std::shared_ptr<Frame>& newDerivFrame);

    VectorDerivativeProperties firstOrder;
    std::weak_ptr<Frame> upperFrame;
    std::weak_ptr<Frame> lowerFrame;
};


/*! @brief basic Basilisk C++ module class */
class UnitVector : public Vector {
public:
    UnitVector() = default;
    UnitVector(const Eigen::Vector3d& zerothMatrix, std::weak_ptr<Frame> zerothWrittenFrame);
    ~UnitVector() = default;
};


#endif
