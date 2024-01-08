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

#ifndef EXT_FORCE_H
#define EXT_FORCE_H

#include "architecture/messaging/messaging.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "simulation/dynamics/KinematicsArchitecture/Vector.h"
#include "simulation/dynamics/KinematicsArchitecture/Part.h"
#include "simulation/dynamics/KinematicsArchitecture/Actuator.h"


class ExtForce: public Actuator {
public:
    ExtForce(const ForceVector& force, std::shared_ptr<Part> part);
    ~ExtForce() = default;

    void actuate() override;

    void setForce(const ForceVector& force);
    void setForce(const Vector& vector, const std::shared_ptr<Point>& applicationPoint);
    void setForce(Eigen::Vector3d matrix, const std::shared_ptr<Frame>& writtenFrame, const std::shared_ptr<Point>& applicationPoint);

private:
    ForceVector force;

    std::shared_ptr<Part> part;
};


#endif
