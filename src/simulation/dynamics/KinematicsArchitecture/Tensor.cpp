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

#include "Tensor.h"
#include "simulation/dynamics/KinematicsArchitecture/KinematicsEngine.h"

#include <utility>

Tensor::Tensor(Eigen::Matrix3d newMatrix, const std::shared_ptr<Frame>& newWrittenFrame) : matrix(std::move(newMatrix)), writtenFrame(newWrittenFrame) {
}

void Tensor::set(Eigen::Matrix3d newMatrix, const std::shared_ptr<Frame>& newWrittenFrame) {
    this->matrix = std::move(newMatrix);
    this->writtenFrame = newWrittenFrame;
}

Eigen::Matrix3d Tensor::getMatrix(const std::shared_ptr<Frame>& frame) const {
    Eigen::MRPd relativeAttitude = KinematicsEngine::findRelativeAttitude(frame, this->writtenFrame);
    Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();

    return dcm * this->matrix * dcm.transpose();
}

std::shared_ptr<Frame> Tensor::getWrittenFrame() const {
    return this->writtenFrame;
}

Tensor Tensor::operator+(const Tensor& tensor) const {
    Eigen::Matrix3d addMatrix = this->matrix + tensor.getMatrix(this->writtenFrame);
    auto addTensor = Tensor(addMatrix, this->writtenFrame);

    return addTensor;
}

void Tensor::operator+=(const Tensor& tensor) {
    this->matrix += tensor.getMatrix(this->writtenFrame);
}

Tensor Tensor::operator-() const {
    Eigen::Matrix3d negativeMat = - this->matrix;
    auto negativeTensor = Tensor(negativeMat, this->writtenFrame);

    return negativeTensor;
}

Tensor Tensor::operator-(const Tensor& tensor) const {
    Eigen::Matrix3d subtractMatrix = this->matrix - tensor.getMatrix(this->writtenFrame);
    auto subtractTensor = Tensor(subtractMatrix, this->writtenFrame);

    return subtractTensor;
}

void Tensor::operator-=(const Tensor& tensor) {
    this->matrix -= tensor.getMatrix(this->writtenFrame);
}

Vector Tensor::operator*(const Vector& vector) const {
    Eigen::Vector3d multiplyMatrix = this->matrix * vector.getMatrix(this->writtenFrame);
    auto multiplyVector = Vector(multiplyMatrix, this->writtenFrame);

    return multiplyVector;
}

Tensor Tensor::operator*(const Tensor& tensor) const {
    Eigen::Matrix3d multiplyMatrix = this->matrix * tensor.getMatrix(this->writtenFrame);
    auto multiplyTensor = Tensor(multiplyMatrix, this->writtenFrame);

    return multiplyTensor;
}

void Tensor::operator*=(const Tensor& tensor) {
    this->matrix *= tensor.getMatrix(this->writtenFrame);
}

Tensor Tensor::operator*(double factor) const {
    Eigen::Matrix3d multiplyMatrix = this->matrix * factor;
    auto multiplyTensor = Tensor(multiplyMatrix, this->writtenFrame);

    return multiplyTensor;
}

Tensor operator*(double factor, const Tensor& tensor) {
    return tensor * factor;
}

void Tensor::operator*=(double factor) {
    this->matrix *= factor;
}

Tensor Tensor::operator/(double factor) const {
    Eigen::Matrix3d divideMatrix = this->matrix / factor;
    auto divideTensor = Tensor(divideMatrix, this->writtenFrame);

    return divideTensor;
}

void Tensor::operator/=(double factor) {
    this->matrix /= factor;
}

Tensor Tensor::inverse() const {
    Eigen::Matrix3d inverseMatrix = this->matrix.inverse();
    auto inverseTensor = Tensor(inverseMatrix, this->writtenFrame);

    return inverseTensor;
}

Tensor Tensor::transpose() const {
    Eigen::Matrix3d transposeMatrix = this->matrix.transpose();
    auto transposeTensor = Tensor(transposeMatrix, this->writtenFrame);

    return transposeTensor;
}

InertiaTensor::InertiaTensor(std::shared_ptr<Point> point):point(std::move(point)) {
}

void InertiaTensor::setFirstOrder(const Eigen::Matrix3d& newMatrix, const std::shared_ptr<Frame>& newWrittenFrame, const std::shared_ptr<Frame>& newDerivFrame) {
    this->firstOrder.matrix = newMatrix;
    this->firstOrder.writtenFrame = newWrittenFrame;
    this->firstOrder.derivFrame = newDerivFrame;
}

void InertiaTensor::setPoint(const std::shared_ptr<Point>& newPoint) {
    this->point = newPoint;
}

std::shared_ptr<Point> InertiaTensor::getPoint() const {
    return this->point;
}
