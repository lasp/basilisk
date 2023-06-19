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
#include "Vector.h"
#include "Frame.h"
#include "Point.h"
#include "KinematicsEngine.h"

void Vector::setZerothOrder(Eigen::Vector3d newMatrix, std::shared_ptr<Frame> newWrittenFrame) {
    this->matrix = newMatrix;
    this->writtenFrame = newWrittenFrame;
}

Eigen::Vector3d Vector::getZerothOrder(std::shared_ptr<Frame> newWrittenFrame) {
    Eigen::MRPd relativeAttitude;

    relativeAttitude = KinematicsEngine::findRelativeAttitude(newWrittenFrame, this->writtenFrame.lock());
    Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();

    return dcm * this->matrix;
}

double Vector::dot(std::shared_ptr<Vector> vec) {
    Eigen::Vector3d vector;
    Eigen::MRPd relativeAttitude;

    relativeAttitude = KinematicsEngine::findRelativeAttitude(this->writtenFrame.lock(), vec->writtenFrame.lock());
    Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();

    double dotProduct = this->matrix.dot(dcm * vec->matrix);

    return dotProduct;
}

std::shared_ptr<Vector> Vector::cross(std::shared_ptr<Vector> vec) {
    Eigen::MRPd relativeAttitude;

    relativeAttitude = KinematicsEngine::findRelativeAttitude(this->writtenFrame.lock(), vec->writtenFrame.lock());
    Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();

    Eigen::Vector3d returnVec = this->matrix.cross(dcm * vec->matrix);

    auto returnVector = std::make_shared<Vector>();
    returnVector->setZerothOrder(returnVec, this->writtenFrame.lock());

    return returnVector;
}

PositionVector::PositionVector(std::shared_ptr<Point> headPoint, std::shared_ptr<Point> tailPoint) {
    this->headPoint = headPoint;
    this->tailPoint = tailPoint;
}

void PositionVector::setFirstOrder(Eigen::Vector3d newMatrix, const std::shared_ptr<Frame>& newWrittenFrame,
                                   const std::shared_ptr<Frame>& newDerivFrame) {
    this->firstOrder.matrix = std::move(newMatrix);
    this->firstOrder.writtenFrame = newWrittenFrame;
    this->firstOrder.derivFrame = newDerivFrame;
}

void PositionVector::setSecondOrder(Eigen::Vector3d newMatrix, const std::shared_ptr<Frame>& newWrittenFrame,
                                   const std::shared_ptr<Frame>& newDerivFrame) {
    this->secondOrder.matrix = std::move(newMatrix);
    this->secondOrder.writtenFrame = newWrittenFrame;
    this->secondOrder.derivFrame = newDerivFrame;
}

std::shared_ptr<PositionVector> PositionVector::add(std::shared_ptr<PositionVector> vec) {
    std::shared_ptr<Point> headPoint = nullptr;
    std::shared_ptr<Point> tailPoint = nullptr;

    Eigen::MRPd relativeAttitude;
    relativeAttitude = KinematicsEngine::findRelativeAttitude(this->writtenFrame.lock(), vec->writtenFrame.lock());

    Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();
    Eigen::Vector3d relPosition = this->matrix + dcm * vec->matrix;

    if (this->tailPoint == vec->headPoint) {
        headPoint = this->headPoint;
        tailPoint = vec->tailPoint;
    }
    else if(this->headPoint == vec->tailPoint) {
        headPoint = vec->headPoint;
        tailPoint = this->tailPoint;
    }

    auto relativePosition = std::make_shared<PositionVector>(headPoint, tailPoint);
    relativePosition->setZerothOrder(relPosition, this->writtenFrame.lock());

    return relativePosition;
}

std::shared_ptr<PositionVector> PositionVector::subtract(std::shared_ptr<PositionVector> vec) {
    std::shared_ptr<Point> headPoint = nullptr;
    std::shared_ptr<Point> tailPoint = nullptr;

    Eigen::MRPd relativeAttitude;
    relativeAttitude = KinematicsEngine::findRelativeAttitude(this->writtenFrame.lock(), vec->writtenFrame.lock());

    Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();
    Eigen::Vector3d relPosition = this->matrix - dcm * vec->matrix;

    if (this->tailPoint == vec->tailPoint) {
        headPoint = this->headPoint;
        tailPoint = vec->headPoint;
    }

    auto relativePosition = std::make_shared<PositionVector>(headPoint, tailPoint);
    relativePosition->setZerothOrder(relPosition, this->writtenFrame.lock());

    return relativePosition;
}

std::shared_ptr<PositionVector> PositionVector::inverse() {
    auto inversePositionVector = std::make_shared<PositionVector>(this->tailPoint, this->headPoint);
    inversePositionVector->setZerothOrder(this->matrix, this->writtenFrame.lock());
}

AngularVelocityVector::AngularVelocityVector(std::weak_ptr<Frame> upperFrame, std::weak_ptr<Frame> lowerFrame) {
    this->upperFrame = upperFrame;
    this->lowerFrame = lowerFrame;
}

void AngularVelocityVector::setFirstOrder(Eigen::Vector3d newMatrix, const std::shared_ptr<Frame>& newWrittenFrame,
                                   const std::shared_ptr<Frame>& newDerivFrame) {
    this->firstOrder.matrix = std::move(newMatrix);
    this->firstOrder.writtenFrame = newWrittenFrame;
    this->firstOrder.derivFrame = newDerivFrame;
}

std::shared_ptr<AngularVelocityVector> AngularVelocityVector::add(std::shared_ptr<AngularVelocityVector> vec) {
    std::weak_ptr<Frame> upperFrame;
    std::weak_ptr<Frame> lowerFrame;

    Eigen::MRPd relativeAttitude;
    relativeAttitude = KinematicsEngine::findRelativeAttitude(this->writtenFrame.lock(), vec->writtenFrame.lock());

    Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();
    Eigen::Vector3d relAngularVelocity = this->matrix + dcm * vec->matrix;

    if (vec->lowerFrame.lock() == this->upperFrame.lock()) {
        upperFrame = this->lowerFrame;
        lowerFrame = vec->lowerFrame;
    }
    else if(vec->lowerFrame.lock() == this->upperFrame.lock()) {
        upperFrame = vec->upperFrame;
        lowerFrame = this->lowerFrame;
    }

    auto relativeAngularVelocity = std::make_shared<AngularVelocityVector>(upperFrame, lowerFrame);
    relativeAngularVelocity->setZerothOrder(relAngularVelocity, this->writtenFrame.lock());

    return relativeAngularVelocity;
}

std::shared_ptr<AngularVelocityVector> AngularVelocityVector::subtract(std::shared_ptr<AngularVelocityVector> vec) {
    std::weak_ptr<Frame> upperFrame;
    std::weak_ptr<Frame> lowerFrame;

    Eigen::MRPd relativeAttitude;
    relativeAttitude = KinematicsEngine::findRelativeAttitude(this->writtenFrame.lock(), vec->writtenFrame.lock());

    Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();
    Eigen::Vector3d relAngularVelocity = this->matrix - dcm * vec->matrix;

    if (this->lowerFrame.lock() == vec->lowerFrame.lock()) {
        upperFrame = this->upperFrame;
        lowerFrame = vec->upperFrame;
    }

    auto relativeAngularVelocity = std::make_shared<AngularVelocityVector>(upperFrame, lowerFrame);
    relativeAngularVelocity->setZerothOrder(relAngularVelocity, this->writtenFrame.lock());

    return relativeAngularVelocity;
}

UnitVector::UnitVector(const Eigen::Vector3d& zerothMatrix,
                       std::weak_ptr<Frame> zerothWrittenFrame) {
    this->setZerothOrder(zerothMatrix.normalized(), zerothWrittenFrame.lock());
}
