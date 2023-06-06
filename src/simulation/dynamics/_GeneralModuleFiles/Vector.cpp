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

Vector::Vector(Eigen::Vector3d zerothMatrix, std::weak_ptr<Frame> zerothWrittenFrame) {
    this->matrix = std::move(zerothMatrix);
    this->writtenFrame = std::move(zerothWrittenFrame);
}

void Vector::setZerothOrder(Eigen::Vector3d newMatrix, std::shared_ptr<Frame> newWrittenFrame) {
    this->matrix = newMatrix;
    this->writtenFrame = newWrittenFrame;
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

UnitVector::UnitVector(const Eigen::Vector3d& zerothMatrix,
                       std::weak_ptr<Frame> zerothWrittenFrame):Vector(zerothMatrix.normalized(),
                                                                       std::move(zerothWrittenFrame)) {
}
