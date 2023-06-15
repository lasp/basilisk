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


Vector::Vector(Eigen::Vector3d zerothMatrix, std::weak_ptr<Frame> zerothWrittenFrame){
    this->matrix = std::move(zerothMatrix);
    this->writtenFrame = std::move(zerothWrittenFrame);
}


void Vector::setZerothOrder(Eigen::Vector3d newMatrix, std::shared_ptr<Frame> newWrittenFrame) {
    this->matrix = newMatrix;
    this->writtenFrame = newWrittenFrame;
}


/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the module */
PositionVector::PositionVector(Eigen::Vector3d zerothMatrix,
                               std::weak_ptr<Frame> zerothWrittenFrame):Vector(std::move(zerothMatrix), std::move(zerothWrittenFrame)){
}

PositionVector::PositionVector(Eigen::Vector3d zerothMatrix, std::weak_ptr<Frame> zerothWrittenFrame,
               Eigen::Vector3d firstMatrix, std::weak_ptr<Frame> firstWrittenFrame, std::weak_ptr<Frame> firstDerivFrame,
               Eigen::Vector3d secondMatrix, std::weak_ptr<Frame> secondWrittenFrame,
               std::weak_ptr<Frame> secondDerivFrame):Vector(std::move(zerothMatrix), std::move(zerothWrittenFrame)){
this->firstOrder.matrix = std::move(firstMatrix);
this->firstOrder.writtenFrame = std::move(firstWrittenFrame);
this->firstOrder.derivFrame = std::move(firstDerivFrame);
this->secondOrder.matrix = std::move(secondMatrix);
this->secondOrder.writtenFrame = std::move(secondWrittenFrame);
this->secondOrder.derivFrame = std::move(secondDerivFrame);
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


/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the module */
AngularVelocityVector::AngularVelocityVector(Eigen::Vector3d zerothMatrix,
                               std::weak_ptr<Frame> zerothWrittenFrame):Vector(std::move(zerothMatrix), std::move(zerothWrittenFrame)){
}

AngularVelocityVector::AngularVelocityVector(Eigen::Vector3d zerothMatrix, std::weak_ptr<Frame> zerothWrittenFrame,
               Eigen::Vector3d firstMatrix, std::weak_ptr<Frame> firstWrittenFrame,
               std::weak_ptr<Frame> firstDerivFrame):Vector(std::move(zerothMatrix), std::move(zerothWrittenFrame)){
this->firstOrder.matrix = std::move(firstMatrix);
this->firstOrder.writtenFrame = std::move(firstWrittenFrame);
this->firstOrder.derivFrame = std::move(firstDerivFrame);
}

void AngularVelocityVector::setFirstOrder(Eigen::Vector3d newMatrix, const std::shared_ptr<Frame>& newWrittenFrame,
                                   const std::shared_ptr<Frame>& newDerivFrame) {
    this->firstOrder.matrix = std::move(newMatrix);
    this->firstOrder.writtenFrame = newWrittenFrame;
    this->firstOrder.derivFrame = newDerivFrame;
}



/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the module */
UnitVector::UnitVector(const Eigen::Vector3d& zerothMatrix,
                       std::weak_ptr<Frame> zerothWrittenFrame):Vector(zerothMatrix.normalized(), std::move(zerothWrittenFrame)){
}