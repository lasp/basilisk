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

#ifndef EXT_TORQUE_H
#define EXT_TORQUE_H

#include "architecture/messaging/messaging.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "simulation/dynamics/KinematicsArchitecture/Vector.h"
#include "simulation/dynamics/KinematicsArchitecture/Part.h"
#include "simulation/dynamics/KinematicsArchitecture/Actuator.h"


class ExtTorque: public Actuator {
public:
    ExtTorque(const TorqueVector& torque, std::shared_ptr<Part> part);
    ~ExtTorque() = default;

    void actuate() override;

    void setTorque(const TorqueVector& torque);
    void setTorque(const Vector& vector);
    void setTorque(Eigen::Vector3d matrix, const std::shared_ptr<Frame>& writtenFrame);

private:
    TorqueVector torque;

    std::shared_ptr<Part> part;
};


#endif
