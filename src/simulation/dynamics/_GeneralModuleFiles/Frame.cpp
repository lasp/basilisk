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
#include "Frame.h"

Frame::Frame(Frame* parentFrame) :
parentFrame(parentFrame),
omega_CP(Vector(Eigen::Vector3d::Zero(), this, Eigen::Vector3d::Zero(), this, this, Eigen::Vector3d::Zero(), this, this)),
r_CP(Vector(Eigen::Vector3d::Zero(), parentFrame, Eigen::Vector3d::Zero(), parentFrame, parentFrame, Eigen::Vector3d::Zero(), parentFrame, parentFrame)) {
}

Frame::Frame(Frame* parentFrame,
             const MRP& sigma_CP,
             const Eigen::Vector3d& omega_CP_C,
             const Eigen::Vector3d& omegaPrime_CP_C,
             const Eigen::Vector3d& r_CP_P,
             const Eigen::Vector3d& rPrime_CP_P,
             const Eigen::Vector3d& rPPrime_CP_P) :
parentFrame(parentFrame),
sigma_CP(sigma_CP),
omega_CP(Vector(omega_CP_C, this, omegaPrime_CP_C, this, this, Eigen::Vector3d::Zero(), nullptr, nullptr)),
r_CP(Vector(r_CP_P, parentFrame, rPrime_CP_P, parentFrame, parentFrame, rPPrime_CP_P, parentFrame, parentFrame)) {
}
