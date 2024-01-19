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
#include "SixDOFRigidBody.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorRK4.h"
#include <iostream>
#include <utility>


SixDOFRigidBody::SixDOFRigidBody(std::shared_ptr<KinematicsEngine> kinematicsEngine, std::shared_ptr<Frame> inertialFrame, std::shared_ptr<Part> body)
: DynamicsEngine(std::move(kinematicsEngine)) {
    // - Set integrator as RK4 by default
    this->integrator = new svIntegratorRK4(this);

    this->timeBefore = 0.0;

    this->body = std::move(body);
    this->baseFrame = this->body->frame;
    this->inertialFrame = std::move(inertialFrame);
    this->mass = this->body->mass;
    this->IScPntC = this->body->IPntSc_S;

    this->L = Vector(Eigen::Vector3d::Zero(), this->baseFrame);
    this->F_C = Vector(Eigen::Vector3d::Zero(), this->inertialFrame);
}


void SixDOFRigidBody::Reset(uint64_t CurrentSimNanos) {
    this->initializeDynamics();
}

void SixDOFRigidBody::UpdateState(uint64_t callTime) {
    this->integrateState(callTime * NANO2SEC);
    this->writeOutputMessages(callTime);
}

void SixDOFRigidBody::writeOutputMessages(uint64_t callTime) {
    this->kinematicsEngine->writeOutputMessages(callTime, this->inertialFrame);
}

void SixDOFRigidBody::initializeDynamics() {
    this->positionState_N = this->dynManager.registerState(3, 1, this->nameOfBodyPosition);
    this->velocityState_N = this->dynManager.registerState(3, 1, this->nameOfBodyVelocity);
    this->sigmaState = this->dynManager.registerState(3, 1, this->nameOfBodySigma);
    this->omegaState_B = this->dynManager.registerState(3, 1, this->nameOfBodyOmega);

    // Get attitude and angular velocity
    auto sigma_BN = this->baseFrame->sigma_SP->getAttitude();
    auto omega_BN = this->baseFrame->sigma_SP->getAngularVelocity();

    // Get position and velocity from the center of mass
    // TODO: change this to use the kinematics engine to find these properties between points
    auto r_CN = this->baseFrame->r_SP->getPosition() + this->body->r_ScS->getPosition();
    auto rDot_CN = this->baseFrame->r_SP->getVelocity(this->inertialFrame)
            + this->body->r_ScS->getVelocity(this->inertialFrame);

    // Need to change this to account for center of mass offset
    this->positionState_N->setState(r_CN.getMatrix(this->inertialFrame));
    this->velocityState_N->setState(rDot_CN.getMatrix(this->inertialFrame));
    this->sigmaState->setState(eigenMRPd2Vector3d(sigma_BN));
    this->omegaState_B->setState(omega_BN.getMatrix(this->baseFrame));

    this->equationsOfMotion(0.0, 0.0);  // sets the derivatives at t0
    this->updateKinematics();
    this->computeEnergyMomentum(0.0);
}

void SixDOFRigidBody::preIntegration(double callTime) {
    this->timeStep = callTime - this->timeBefore;
}

void SixDOFRigidBody::equationsOfMotion(double t, double timeStep) {
    this->updateKinematics();

    this->body->clearForceList();
    this->body->clearTorqueList();
    for (const auto& actuator: this->actuatorList) {
        actuator->actuate();
    }

        // Grab the states
    auto rDot_CN = Vector(this->velocityState_N->getState(), this->inertialFrame);
    auto omega_BN = Vector(this->omegaState_B->getState(), this->baseFrame);
    Eigen::MRPd sigma_BN;
    sigma_BN = (Eigen::Vector3d)this->sigmaState->getState();  // this conversion is so dumb

    this->F_C.setZero();
    this->L.setZero();
    for (const auto& force: this->body->getForceList()) {
        this->F_C += force;
        auto arm = this->kinematicsEngine->findRelativePosition(this->body->CoMPoint, force.applicationPoint.lock());
        this->L += arm.cross(force);
    }
    for (const auto& torque: this->body->getTorqueList()) {
        this->L += torque;
    }

    auto omegaDot_BN = this->IScPntC.inverse() * (-omega_BN.cross(this->IScPntC * omega_BN) + this->L);
    auto rDDot_CN = this->F_C / this->mass;

    this->sigmaState->setDerivative(1.0/4.0 * sigma_BN.Bmat() * omega_BN.getMatrix(this->baseFrame));
    this->omegaState_B->setDerivative(omegaDot_BN.getMatrix(this->baseFrame));

    this->positionState_N->setDerivative(rDot_CN.getMatrix(this->inertialFrame));
    this->velocityState_N->setDerivative(rDDot_CN.getMatrix(this->inertialFrame));
}

void SixDOFRigidBody::postIntegration(double callTime) {
    this->timeBefore = callTime;

    // Switch MRPs
    Eigen::Vector3d sigma_BN;
    sigma_BN = (Eigen::Vector3d) this->sigmaState->getState();
    if (sigma_BN.norm() > 1) {
        sigma_BN = - sigma_BN / (sigma_BN.dot(sigma_BN));
        this->sigmaState->setState(sigma_BN);
    }

    this->updateKinematics();
    this->computeEnergyMomentum(callTime);
}

void SixDOFRigidBody::updateKinematics() {
    auto r_CN = Vector(this->positionState_N->getState(), this->inertialFrame);
    auto rDot_CN = Vector(this->velocityState_N->getState(), this->inertialFrame);
    auto omega_BN = Vector(this->omegaState_B->getState(), this->baseFrame);
    Eigen::MRPd sigma_BN;
    sigma_BN = (Eigen::Vector3d)this->sigmaState->getState();  // this conversion is so dumb

    auto r_BN = r_CN - this->body->r_ScS->getPosition();
    auto rDot_BN = rDot_CN - this->body->r_ScS->getVelocity(this->inertialFrame);

    this->baseFrame->r_SP->setPosition(r_BN);
    this->baseFrame->r_SP->setVelocity(rDot_BN, this->inertialFrame);
    this->baseFrame->sigma_SP->setAttitude(sigma_BN);
    this->baseFrame->sigma_SP->setAngularVelocity(omega_BN);
}

void SixDOFRigidBody::computeEnergyMomentum(double time) {
    // TODO: change this to use class info instead of grabbing from stateData since it's called after updateKinematics
    auto r_CN = Vector(this->positionState_N->getState(), this->inertialFrame);
    auto rDot_CN = Vector(this->velocityState_N->getState(), this->inertialFrame);
    auto omega_BN = Vector(this->omegaState_B->getState(), this->baseFrame);

    this->transAngMomPntN_N = this->body->mass * r_CN.cross(rDot_CN).getMatrix(this->inertialFrame);
    this->rotAngMomPntC_N = (this->IScPntC * omega_BN).getMatrix(this->inertialFrame);
    this->rotEnergy = 1.0 / 2.0 * omega_BN.dot(this->IScPntC * omega_BN);
}
