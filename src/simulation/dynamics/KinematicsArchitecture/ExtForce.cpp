/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "simulation/dynamics/KinematicsArchitecture/ExtForce.h"

#include <utility>
#include <iostream>

ExtForce::ExtForce(const ForceVector& force, std::shared_ptr<Part> part)
        : force(force), part(std::move(part)) {
}

void ExtForce::setForce(const ForceVector& force) {
    this->force = force;
}

void ExtForce::setForce(const Vector& vector, const std::shared_ptr<Point>& applicationPoint) {
    this->force = ForceVector(vector, applicationPoint);
}

void ExtForce::setForce(Eigen::Vector3d matrix, const std::shared_ptr<Frame> &writtenFrame,
                           const std::shared_ptr<Point>& applicationPoint) {
    this->force = ForceVector(matrix, writtenFrame, applicationPoint);
}

void ExtForce::actuate() {
    this->part->addForceVector(this->force);
}
