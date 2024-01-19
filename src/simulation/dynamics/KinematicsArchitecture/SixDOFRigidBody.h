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

#ifndef SIX_DOF_RIGID_BODY_H
#define SIX_DOF_RIGID_BODY_H

#include <utility>

#include "DynamicsEngine.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"

class SixDOFRigidBody: public DynamicsEngine {
public:
    explicit SixDOFRigidBody(std::shared_ptr<KinematicsEngine> kinematicsEngine, std::shared_ptr<Frame> inertialFrame, std::shared_ptr<Part> body);

    // SysModel functions
    void Reset(uint64_t CurrentSimNanos) final;
    void UpdateState(uint64_t callTime) final;

    // DynamicObject functions
    void equationsOfMotion(double t, double timeStep) final;
    void preIntegration(double callTime) final;
    void postIntegration(double callTime) final;
    void initializeDynamics() final;
    void computeEnergyMomentum(double time) final;

    // DynamicEngine functions
    void updateKinematics() final;

    void writeOutputMessages(uint64_t callTime);

    std::string nameOfBodyPosition {"bodyPosition"};
    std::string nameOfBodyVelocity {"bodyVelocity"};
    std::string nameOfBodySigma {"bodySigma"};
    std::string nameOfBodyOmega {"bodyOmega"};

    double rotEnergy = 0.0;
    Eigen::Vector3d rotAngMomPntC_N = Eigen::Vector3d::Zero();
    Eigen::Vector3d transAngMomPntN_N = Eigen::Vector3d::Zero();

private:
    StateData* sigmaState = nullptr;
    StateData* omegaState_B = nullptr;
    StateData* positionState_N = nullptr;
    StateData* velocityState_N = nullptr;

    double mass = 1.0;
    InertiaTensor IScPntC;

    std::shared_ptr<Part> body;
    std::shared_ptr<Frame> baseFrame;
    Vector L;
    Vector F_C;
};


#endif
