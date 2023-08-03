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
#include <utility>
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"

class Frame;
class Point;

class Vector {
public:
    Vector() = default;
    ~Vector() = default;
    Vector(Eigen::Vector3d  newMatrix, const std::shared_ptr<Frame>& newWrittenFrame); // do we change this to const std::shared_ptr<const Frame>&?

    void set(Eigen::Vector3d newMatrix, const std::shared_ptr<Frame>& newWrittenFrame);
    Eigen::Vector3d getMatrix(const std::shared_ptr<Frame>& frame) const;
    std::shared_ptr<Frame> getWrittenFrame() const;

    Vector operator+(const Vector& vec) const;
    void operator+=(const Vector& vec);
    Vector operator-() const;
    Vector operator-(const Vector& vec) const;
    void operator-=(const Vector& vec);
    Vector operator*(double factor) const;
    void operator*=(double factor);
    Vector operator/(double factor) const;
    void operator/=(double factor);
    double dot(const Vector& vec) const;
    Vector cross(const Vector& vec) const;

private:
    Eigen::Vector3d matrix = Eigen::Vector3d::Zero();
    std::weak_ptr<Frame> writtenFrame;
};

Vector operator*(double factor, const Vector& vector);  // this is throwing a warning


class Translation {
public:
    Translation(std::shared_ptr<Point> headPoint, std::shared_ptr<Point> tailPoint) : headPoint(std::move(headPoint)), tailPoint(std::move(tailPoint)) {};
    ~Translation() = default;
    
    void setPosition(const Eigen::Vector3d& matrix, const std::shared_ptr<Frame>& writtenFrame);  // sets position
    void setPosition(const Vector& newPosition);  // sets position
    void setVelocity(const Eigen::Vector3d& matrix, const std::shared_ptr<Frame>& newWrittenFrame, const std::shared_ptr<Frame>& newFirstDerivFrame);  // sets velocity
    void setVelocity(const Vector& newVelocity, const std::shared_ptr<Frame>& newFirstDerivFrame);  // sets velocity
    void setAcceleration(const Eigen::Vector3d& matrix, const std::shared_ptr<Frame>& newWrittenFrame, const std::shared_ptr<Frame>& newFirstDerivFrame, const std::shared_ptr<Frame>& newSecondDerivFrame);  // sets acceleration
    void setAcceleration(const Vector& newAcceleration, const std::shared_ptr<Frame>& newFirstDerivFrame, const std::shared_ptr<Frame>& newSecondDerivFrame);  // sets acceleration
    Vector getPosition() const;  // gets position
    Vector getVelocity(const std::shared_ptr<Frame>& newFirstDerivFrame) const;  // gets velocity
    Vector getAcceleration(const std::shared_ptr<Frame>& newFirstDerivFrame, const std::shared_ptr<Frame>& newSecondDerivFrame) const;  // sets acceleration, but stores it such that it is consistent with the firstDerivFrame, has to call TransportTheorem

    std::shared_ptr<Point> getHeadPoint() const;
    std::shared_ptr<Point> getTailPoint() const;

private:
    Vector position;
    Vector velocity;
    Vector acceleration;

    std::shared_ptr<Frame> firstDerivFrame = nullptr;
    std::shared_ptr<Frame> secondDerivFrame = nullptr;

    std::shared_ptr<Point> headPoint = nullptr;
    std::shared_ptr<Point> tailPoint = nullptr;
};


class Rotation {
public:
    Rotation(std::shared_ptr<Frame> upperFrame, std::shared_ptr<Frame> lowerFrame) : upperFrame(std::move(upperFrame)), lowerFrame(std::move(lowerFrame)) {};
    ~Rotation() = default;
    
    void setAttitude(const Eigen::MRPd& matrix);  // sets attitude
    void setAngularVelocity(const Eigen::Vector3d& matrix, const std::shared_ptr<Frame>& newWrittenFrame);  // sets angular velocity
    void setAngularVelocity(const Vector& newAngularVelocity);  // sets angular velocity
    void setAngularAcceleration(const Eigen::Vector3d& matrix, const std::shared_ptr<Frame>& writtenFrame, const std::shared_ptr<Frame>& derivFrame);  // sets angular acceleration
    void setAngularAcceleration(const Vector& newAngularAcceleration, const std::shared_ptr<Frame>& newDerivFrame);  // sets angular acceleration
    Eigen::MRPd getAttitude() const;  // gets attitude
    Vector getAngularVelocity() const;  // gets angular velocity
    Vector getAngularAcceleration(const std::shared_ptr<Frame>& newDerivFrame) const;  // sets angular acceleration

    std::shared_ptr<Frame> getUpperFrame() const;
    std::shared_ptr<Frame> getLowerFrame() const;

private:
    Eigen::MRPd sigma;
    Vector angularVelocity;
    Vector angularAcceleration;

    std::shared_ptr<Frame> derivFrame = nullptr;

    std::shared_ptr<Frame> upperFrame = nullptr;
    std::shared_ptr<Frame> lowerFrame = nullptr;
};


class ForceVector : public Vector {
public:
    ForceVector() = default;
    ~ForceVector() = default;

    std::shared_ptr<Point> applicationPoint;
};


class UnitVector : public Vector {
public:
    UnitVector() = default;
    UnitVector(const Eigen::Vector3d& zerothMatrix, const std::weak_ptr<Frame>& zerothWrittenFrame);
    ~UnitVector() = default;
};


#endif
