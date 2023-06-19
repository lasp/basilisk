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
#include "Frame.h"
#include "KinematicsEngine.h"
#include <utility>

InertiaTensor::InertiaTensor(std::shared_ptr<Point> point):point(std::move(point)) {
}

void Tensor::setZerothOrder(const Eigen::Matrix3d& newMatrix, const std::shared_ptr<Frame>& newWrittenFrame) {
    this->matrix = newMatrix;
    this->writtenFrame = newWrittenFrame;
}

Eigen::Matrix3d Tensor::getZerothOrder(std::shared_ptr<Frame> newWrittenFrame) {
    Eigen::MRPd relativeAttitude;
    relativeAttitude = KinematicsEngine::findRelativeAttitude(newWrittenFrame, this->writtenFrame);

    Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();

    return (dcm * this->matrix * dcm.transpose());
}

std::shared_ptr<Tensor> Tensor::inverse() {
    Eigen::Matrix3d inverseMat = this->matrix.inverse();

    auto returnTensor = std::make_shared<Tensor>();
    returnTensor->setZerothOrder(inverseMat, this->writtenFrame);

    return returnTensor;
}

std::shared_ptr<Tensor> Tensor::add(std::shared_ptr<Tensor> tensor) {
    Eigen::MRPd relativeAttitude;
    relativeAttitude = KinematicsEngine::findRelativeAttitude(this->writtenFrame, tensor->writtenFrame);

    Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();

    Eigen::Matrix3d returnMat = this->matrix + (dcm * tensor->matrix * dcm.transpose());

    auto returnTensor = std::make_shared<Tensor>();
    returnTensor->setZerothOrder(returnMat, this->writtenFrame);

    return returnTensor;
}

std::shared_ptr<Tensor> Tensor::subtract(std::shared_ptr<Tensor> tensor) {
    Eigen::MRPd relativeAttitude;
    relativeAttitude = KinematicsEngine::findRelativeAttitude(this->writtenFrame, tensor->writtenFrame);

    Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();

    Eigen::Matrix3d returnMat = this->matrix - (dcm * tensor->matrix * dcm.transpose());

    auto returnTensor = std::make_shared<Tensor>();
    returnTensor->setZerothOrder(returnMat, this->writtenFrame);

    return returnTensor;
}

std::shared_ptr<Vector> Tensor::times(std::shared_ptr<Vector> vector) {
    Eigen::MRPd relativeAttitude;
    relativeAttitude = KinematicsEngine::findRelativeAttitude(this->writtenFrame, vector->writtenFrame.lock());

    Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();

    Eigen::Vector3d returnVec = this->matrix * (dcm * vector->matrix);

    auto returnVector = std::make_shared<Vector>();
    returnVector->setZerothOrder(returnVec, this->writtenFrame);

    return returnVector;
}

std::shared_ptr<Tensor> Tensor::times(std::shared_ptr<Tensor> tensor) {
    Eigen::MRPd relativeAttitude;
    relativeAttitude = KinematicsEngine::findRelativeAttitude(this->writtenFrame, tensor->writtenFrame);

    Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();

    Eigen::Matrix3d returnMat = this->matrix * (dcm * tensor->matrix * dcm.transpose());

    auto returnTensor = std::make_shared<Tensor>();
    returnTensor->setZerothOrder(returnMat, this->writtenFrame);

    return returnTensor;
}

void InertiaTensor::setFirstOrder(const Eigen::Matrix3d& newMatrix, const std::shared_ptr<Frame>& newWrittenFrame, const std::shared_ptr<Frame>& newDerivFrame) {
    this->firstOrder.matrix = newMatrix;
    this->firstOrder.writtenFrame = newWrittenFrame;
    this->firstOrder.derivFrame = newDerivFrame;
}

std::shared_ptr<InertiaTensor> InertiaTensor::add(std::shared_ptr<InertiaTensor> inertiaTensor) {
    auto tensor = Tensor::add(inertiaTensor);

    auto returnInertiaTensor = std::make_shared<InertiaTensor>(inertiaTensor->point);
    returnInertiaTensor->setZerothOrder(tensor->matrix, tensor->writtenFrame);

    return returnInertiaTensor;
}

std::shared_ptr<InertiaTensor> InertiaTensor::subtract(std::shared_ptr<InertiaTensor> inertiaTensor) {
    auto tensor = Tensor::subtract(inertiaTensor);

    auto returnInertiaTensor = std::make_shared<InertiaTensor>(inertiaTensor->point);
    returnInertiaTensor->setZerothOrder(tensor->matrix, tensor->writtenFrame);

    return returnInertiaTensor;
}
