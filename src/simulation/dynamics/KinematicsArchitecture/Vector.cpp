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
#include "simulation/dynamics/KinematicsArchitecture/KinematicsEngine.h"

#include <utility>

Vector::Vector(Eigen::Vector3d  newMatrix, const std::shared_ptr<Frame>& newWrittenFrame) : matrix(std::move(newMatrix)), writtenFrame(newWrittenFrame) {
}

void Vector::set(Eigen::Vector3d newMatrix, const std::shared_ptr<Frame>& newWrittenFrame) {
    this->matrix = std::move(newMatrix);
    this->writtenFrame = newWrittenFrame;
}

Eigen::Vector3d Vector::getMatrix(const std::shared_ptr<Frame>& frame) const {
    Eigen::MRPd relativeAttitude = KinematicsEngine::findRelativeAttitude(frame, this->writtenFrame.lock());
    Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();

    return dcm * this->matrix;
}

std::shared_ptr<Frame> Vector::getWrittenFrame() const {
    return this->writtenFrame.lock();
}

Vector Vector::operator+(const Vector& vec) const{
    Eigen::Vector3d addMatrix = this->matrix + vec.getMatrix(this->writtenFrame.lock());
    auto addVector = Vector(addMatrix, this->writtenFrame.lock());

    return addVector;
}

void Vector::operator+=(const Vector& vec){
    this->matrix += vec.getMatrix(this->writtenFrame.lock());
}

Vector Vector::operator-() const {
    Eigen::Vector3d negativeMatrix = - this->matrix;
    auto negativeVector = Vector(negativeMatrix, this->writtenFrame.lock());

    return negativeVector;
}

Vector Vector::operator-(const Vector& vec) const {
    Eigen::Vector3d subtractMatrix = this->matrix - vec.getMatrix(this->writtenFrame.lock());
    auto subtractVector = Vector(subtractMatrix, this->writtenFrame.lock());

    return subtractVector;
}

void Vector::operator-=(const Vector& vec){
    this->matrix -= vec.getMatrix(this->writtenFrame.lock());
}

Vector Vector::operator*(double factor) const {
    Eigen::Vector3d multiplyMatrix = factor * this->matrix;
    auto multiplyVector = Vector(multiplyMatrix, this->writtenFrame.lock());

    return multiplyVector;
}

Vector operator*(double factor, const Vector& vector) {
    return vector * factor;
}

void Vector::operator*=(double factor){
    this->matrix *= factor;
}

Vector Vector::operator/(double factor) const {
    Eigen::Vector3d divideMatrix = this->matrix / factor;
    auto divideVector = Vector(divideMatrix, this->writtenFrame.lock());

    return divideVector;
}

void Vector::operator/=(double factor){
    this->matrix /= factor;
}

double Vector::dot(const Vector& vec) const {
    return this->matrix.dot(vec.getMatrix(this->writtenFrame.lock()));
}

Vector Vector::cross(const Vector& vec) const {
    Eigen::Vector3d crossMatrix = this->matrix.cross(vec.getMatrix(this->writtenFrame.lock()));
    auto crossVector = Vector(crossMatrix, this->writtenFrame.lock());

    return crossVector;
}

UnitVector::UnitVector(const Eigen::Vector3d& zerothMatrix,
                       const std::weak_ptr<Frame>& zerothWrittenFrame) {
    this->set(zerothMatrix.normalized(), zerothWrittenFrame.lock());
}

void Translation::setPosition(const Eigen::Vector3d& matrix, const std::shared_ptr<Frame>& writtenFrame) {
    this->position.set(matrix, writtenFrame);
}

void Translation::setPosition(const Vector& newPosition) {
    this->position = newPosition;
}

void Translation::setVelocity(const Eigen::Vector3d& matrix,
                              const std::shared_ptr<Frame>& newWrittenFrame,
                              const std::shared_ptr<Frame>& newFirstDerivFrame) {
    auto newVelocity = Vector(matrix, newWrittenFrame);
    this->setVelocity(newVelocity, newFirstDerivFrame);
}

// We're currently only changing this->firstDerivFrame if it's nullptr because it would mess with the acceleration too, but this feels funky
void Translation::setVelocity(const Vector& newVelocity, const std::shared_ptr<Frame>& newFirstDerivFrame) {
    if (!this->firstDerivFrame) {
        this->velocity = newVelocity;
        this->firstDerivFrame = newFirstDerivFrame;
    } else {
        const auto& frameA = this->firstDerivFrame;
        const auto& frameB = newFirstDerivFrame;

        const auto& r = this->position;
        const auto& rPrimeB = newVelocity;
        const auto& omega_AB = KinematicsEngine::findRelativeAngularVelocity(frameA, frameB);
        this->velocity = rPrimeB - omega_AB.cross(r);
    }
}

void Translation::setAcceleration(const Eigen::Vector3d& matrix,
                                  const std::shared_ptr<Frame>& newWrittenFrame,
                                  const std::shared_ptr<Frame>& newFirstDerivFrame,
                                  const std::shared_ptr<Frame>& newSecondDerivFrame) {
    auto newAcceleration = Vector(matrix, newWrittenFrame);
    this->setAcceleration(newAcceleration, newFirstDerivFrame, newSecondDerivFrame);
}

void Translation::setAcceleration(const Vector& newAcceleration,
                                  const std::shared_ptr<Frame>& newFirstDerivFrame,
                                  const std::shared_ptr<Frame>& newSecondDerivFrame) {
    if (!this->secondDerivFrame) {
        this->secondDerivFrame = newSecondDerivFrame;
        if (!this->firstDerivFrame) {
            this->firstDerivFrame = newFirstDerivFrame;
            this->acceleration = newAcceleration;

            return;
        }
    }
    const auto& frameA = this->firstDerivFrame;
    const auto& frameB = this->secondDerivFrame;
    const auto& frameC = newFirstDerivFrame;
    const auto& frameD = newSecondDerivFrame;

    const auto& r = this->position;
    const auto& rPrimeC = this->getVelocity(frameC);
    const auto& rPPrimeDC = newAcceleration;
    const auto& omega_BD = KinematicsEngine::findRelativeAngularVelocity(frameB, frameD);
    const auto& omegaPrimeB_AC = KinematicsEngine::findRelativeAngularAcceleration(frameA, frameC, frameB);

    this->acceleration = rPPrimeDC - omegaPrimeB_AC.cross(r) - omega_BD.cross(rPrimeC);

}

Vector Translation::getPosition() const {
    return position;
}

Vector Translation::getVelocity(const std::shared_ptr<Frame>& newFirstDerivFrame) const {
    const auto& frameA = this->firstDerivFrame;
    const auto& frameB = newFirstDerivFrame;

    const auto& r = this->position;
    const auto& rPrimeA = this->velocity;
    const auto& omega_AB = KinematicsEngine::findRelativeAngularVelocity(frameA, frameB);

    return rPrimeA + omega_AB.cross(r);
}

Vector Translation::getAcceleration(const std::shared_ptr<Frame>& newFirstDerivFrame, const std::shared_ptr<Frame>& newSecondDerivFrame) const {
    const auto& frameA = this->firstDerivFrame;
    const auto& frameB = this->secondDerivFrame;
    const auto& frameC = newFirstDerivFrame;
    const auto& frameD = newSecondDerivFrame;

    const auto& r = this->position;
    const auto& rPrimeC = this->getVelocity(frameC);
    const auto& rPPrimeBA = this->acceleration;
    const auto& omega_BD = KinematicsEngine::findRelativeAngularVelocity(frameB, frameD);
    const auto& omegaPrimeB_AC = KinematicsEngine::findRelativeAngularAcceleration(frameA, frameC, frameB);

    return rPPrimeBA + omegaPrimeB_AC.cross(r) + omega_BD.cross(rPrimeC);
}

std::shared_ptr<Point> Translation::getHeadPoint() const{
    return this->headPoint;
}

std::shared_ptr<Point> Translation::getTailPoint() const{
    return this->tailPoint;
}

void Rotation::setAttitude(const Eigen::MRPd& matrix) {
    this->sigma = matrix;
}

void Rotation::setAngularVelocity(const Eigen::Vector3d& matrix, const std::shared_ptr<Frame>& newWrittenFrame) {
    auto newAngularVelocity = Vector(matrix, newWrittenFrame);
    this->setAngularVelocity(newAngularVelocity);
}

void Rotation::setAngularVelocity(const Vector& newAngularVelocity) {
    this->angularVelocity = newAngularVelocity;
}

void Rotation::setAngularAcceleration(const Eigen::Vector3d& matrix, const std::shared_ptr<Frame>& newWrittenFrame,
                                      const std::shared_ptr<Frame>& newDerivFrame) {
    auto newAngularAcceleration = Vector(matrix, newWrittenFrame);
    this->setAngularAcceleration(newAngularAcceleration, newDerivFrame);
}

void Rotation::setAngularAcceleration(const Vector& newAngularAcceleration, const std::shared_ptr<Frame>& newDerivFrame) {
    if (!derivFrame) {
        this->angularAcceleration = newAngularAcceleration;
        this->derivFrame = newDerivFrame;
    } else {
        const auto& frameA = this->derivFrame;
        const auto& frameB = newDerivFrame;

        const auto& omega = this->angularVelocity;
        const auto& omega_AB = KinematicsEngine::findRelativeAngularVelocity(frameA, frameB);
        const auto& omegaPrimeB = newAngularAcceleration;

        this->angularAcceleration = omegaPrimeB - omega_AB.cross(omega);
    }
}

Eigen::MRPd Rotation::getAttitude() const {
    return this->sigma;
}

Vector Rotation::getAngularVelocity() const {
    return this->angularVelocity;
}

Vector Rotation::getAngularAcceleration(const std::shared_ptr<Frame>& newDerivFrame) const {
    const auto& frameA = this->derivFrame;
    const auto& frameB = newDerivFrame;

    const auto& omega = this->angularVelocity;
    const auto& omega_AB = KinematicsEngine::findRelativeAngularVelocity(frameA, frameB);
    const auto& omegaPrimeA = this->angularAcceleration;

    return omegaPrimeA + omega_AB.cross(omega);
}

std::shared_ptr<Frame> Rotation::getUpperFrame() const {
    return this->upperFrame;
}

std::shared_ptr<Frame> Rotation::getLowerFrame() const {
    return this->lowerFrame;
}
