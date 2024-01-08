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
#include "Part.h"

#include <utility>

Part::Part(std::shared_ptr<Frame> frame)
    : frame(std::move(frame)) {}

void Part::addForceVector(const ForceVector& force) {
    this->forceList.push_back(force);
}

void Part::addTorqueVector(const TorqueVector& torque) {
    this->torqueList.push_back(torque);
}

std::vector<ForceVector> Part::getForceList() const {
    return this->forceList;
}

std::vector<TorqueVector> Part::getTorqueList() const {
    return this->torqueList;
}

void Part::clearForceList() {
    this->forceList.clear();
}

void Part::clearTorqueList() {
    this->torqueList.clear();
}
