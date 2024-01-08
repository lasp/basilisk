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

#ifndef DYNAMICS_ENGINE_H
#define DYNAMICS_ENGINE_H

#include <utility>

#include "simulation/dynamics/KinematicsArchitecture/KinematicsEngine.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"
#include "simulation/dynamics/KinematicsArchitecture/Actuator.h"
#include "simulation/dynamics/KinematicsArchitecture/ExtForce.h"
#include "simulation/dynamics/KinematicsArchitecture/ExtTorque.h"

class DynamicsEngine: public DynamicObject {
public:
    explicit DynamicsEngine(std::shared_ptr<KinematicsEngine> kinematicsEngine) : kinematicsEngine(std::move(kinematicsEngine)) {};
    ~DynamicsEngine() override = default;

    std::shared_ptr<KinematicsEngine> kinematicsEngine;
    std::shared_ptr<Frame> inertialFrame;

    std::vector<std::shared_ptr<Actuator>> actuatorList;

    void UpdateState(uint64_t callTime) override = 0;
    void equationsOfMotion(double t, double timeStep) override = 0;
    void preIntegration(double callTime) override = 0;
    void postIntegration(double callTime) override = 0;

    virtual void updateKinematics() = 0;

    std::shared_ptr<ExtForce> createExtForce(const ForceVector& force, const std::shared_ptr<Part>& part);
    std::shared_ptr<ExtTorque> createExtTorque(const TorqueVector& torque, const std::shared_ptr<Part>& part);
};


#endif
