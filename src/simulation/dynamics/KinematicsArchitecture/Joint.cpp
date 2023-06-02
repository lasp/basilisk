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
#include "Joint.h"


RotaryOneDOF::RotaryOneDOF(std::shared_ptr<Hinge> hinge) {
    n = 1;

    this->hingeVector.push_back(std::move(hinge));

    this->lowerFrame = this->hingeVector[0]->equilibriumFrame;
    this->upperFrame = this->hingeVector[0]->currentFrame;
}



RotaryTwoDOF::RotaryTwoDOF(std::shared_ptr<Hinge> firstHinge, std::shared_ptr<Hinge> secondHinge) {
    n = 2;

    this->hingeVector.push_back(std::move(firstHinge));
    this->hingeVector.push_back(std::move(secondHinge));

    this->hingeVector[1]->equilibriumFrame->setParentFrame(this->hingeVector[0]->currentFrame);
    this->lowerFrame = this->hingeVector[0]->equilibriumFrame;
    this->upperFrame = this->hingeVector[1]->currentFrame;
}
