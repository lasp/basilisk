/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "stateSpaceStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <string>

StateSpaceStateEffector::StateSpaceStateEffector(int n) : sizeOfStateSpace(n)
{
    this->XInit = Eigen::VectorXd::Zero(sizeOfStateSpace);
    this->M = Eigen::MatrixXd::Ones(sizeOfStateSpace, sizeOfStateSpace);
    this->K = Eigen::MatrixXd::Zero(sizeOfStateSpace, sizeOfStateSpace);
    this->C = Eigen::MatrixXd::Zero(sizeOfStateSpace, sizeOfStateSpace);
    this->AEff = Eigen::MatrixXd::Zero(sizeOfStateSpace, 3);
    this->BEff = Eigen::MatrixXd::Zero(sizeOfStateSpace, 3);
    this->CEff = Eigen::VectorXd::Zero(sizeOfStateSpace);

    // Zero the mass props and mass prop rates contributions
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);

    this->nameOfXState = "X" + std::to_string(StateSpaceStateEffector::effectorID);
    this->nameOfXDotState = "XDot" + std::to_string(StateSpaceStateEffector::effectorID);
    StateSpaceStateEffector::effectorID++;
}

uint64_t StateSpaceStateEffector::effectorID = 1;

StateSpaceStateEffector::~StateSpaceStateEffector()
{
    StateSpaceStateEffector::effectorID --;
}

void StateSpaceStateEffector::prependSpacecraftNameToStates()
{
    this->nameOfXState = this->nameOfSpacecraftAttachedTo + this->nameOfXState;
    this->nameOfXDotState = this->nameOfSpacecraftAttachedTo + this->nameOfXDotState;
}

void StateSpaceStateEffector::linkInStates(DynParamManager& statesIn)
{
    this->m_SC = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "m_SC");
    this->c_B = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "centerOfMassSC");
    this->inertialPositionProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "r_BN_N");
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "v_BN_N");
}

void StateSpaceStateEffector::registerStates(DynParamManager& states)
{
    this->XState = states.registerState(this->sizeOfStateSpace, 1, this->nameOfXState);
    this->XDotState = states.registerState(this->sizeOfStateSpace, 1, this->nameOfXDotState);

    this->XState->setState(this->XInit);
    this->XDotState->setState(this->XDotInit);
}

void StateSpaceStateEffector::updateEffectorMassProps(double integTime)
{
    // Grab states
    this->X = this->XState->getState();
    this->XDot = this->XDotState->getState();

    // Compute intermediate variables
    Eigen::Vector3d r_SF_F = this->X.head(3);
    Eigen::Vector3d r_ScB_B = this->dcm_FB.transpose() * (this->r_ScS_F + r_SF_F) + this->r_FB_B;
    Eigen::Matrix3d rTilde_ScB_B = eigenTilde(r_ScB_B);
    Eigen::Matrix3d ISPntB_B = this->dcm_FB.transpose() * this->ISPntSc_F * this->dcm_FB
            - this->mass * rTilde_ScB_B * rTilde_ScB_B;

    // Populate mass contributions
    this->effProps.mEff = this->mass;
    this->effProps.rEff_CB_B = r_ScB_B;
    this->effProps.IEffPntB_B = ISPntB_B;
}

void StateSpaceStateEffector::updateContributions(double integTime,
                                                  BackSubMatrices & backSubContr,
                                                  Eigen::Vector3d sigma_BN,
                                                  Eigen::Vector3d omega_BN_B,
                                                  Eigen::Vector3d g_N)
{
    // Grab states
    this->X = this->XState->getState();
    this->XDot = this->XDotState->getState();
    this->sigma_BN = sigma_BN;
    this->omega_BN_B = omega_BN_B;

    // Compute intermediate variables
    Eigen::Vector3d omega_BN_F = this->dcm_FB * this->omega_BN_B;
    Eigen::Matrix3d omegaTilde_BN_F = eigenTilde(omega_BN_F);
    Eigen::Vector3d r_SF_F = this->X.head(3);
    Eigen::Vector3d r_SB_F = r_SF_F + this->dcm_FB * this->r_FB_B;
    Eigen::Vector3d r_ScB_F = this->r_ScS_F + r_SB_F;
    Eigen::Matrix3d rTilde_SB_F = eigenTilde(r_SB_F);
    Eigen::Matrix3d rTilde_ScB_F = eigenTilde(r_ScB_F);
    Eigen::Matrix3d ISPntS_F = this->ISPntSc_F - this->mass * rTilde_ScB_F * rTilde_ScB_F;

    // Populate Star matrices
    Eigen::MatrixXd AEffStar = - this->M.block(0, 0, 3, this->sizeOfStateSpace).transpose();
    Eigen::MatrixXd BEffStar = - this->M.block(3, 0, 3, this->sizeOfStateSpace).transpose();
    Eigen::VectorXd CEffStar = Eigen::VectorXd::Zero(this->sizeOfStateSpace);
    CEffStar.block<3,1>(0, 0) = - this->mass * omegaTilde_BN_F * omegaTilde_BN_F * r_ScB_F
                                - this->K.block<3, 3>(0, 0) * this->X.block<3, 1>(0, 0)
                                - this->C.block<3, 3>(0, 0) * this->XDot.block<3, 1>(0, 0);
    CEffStar.block<3,1>(3, 0) = - omegaTilde_BN_F * ISPntS_F * omega_BN_F
                                - this->K.block<3, 3>(3, 3) * this->X.block<3, 1>(3, 0)
                                - this->C.block<3, 3>(3, 3) * this->XDot.block<3, 1>(3, 0);
    CEffStar.block<1,1>(6, 0) = - this->K.block<1, 1>(6, 6) * this->X.block<1, 1>(6, 0)
                                - this->C.block<1, 1>(6, 6) * this->XDot.block<1, 1>(6, 0);

    // Find effector matrices
    this->AEff = this->M.inverse() * AEffStar;
    this->BEff = this->M.inverse() * BEffStar;
    this->CEff = this->M.inverse() * CEffStar;

    // Compute backsubstitution contributions
    Eigen::MatrixXd vTransLHS = this->M.block<3, 7>(0, 0);
    Eigen::MatrixXd vRotLHS = this->M.block<3, 7>(3, 0) + rTilde_SB_F * vTransLHS;
    backSubContr.matrixA = this->dcm_FB.transpose() * vTransLHS * this->AEff * this->dcm_FB;
    backSubContr.matrixB = this->dcm_FB.transpose() * vTransLHS * this->BEff * this->dcm_FB;
    backSubContr.matrixC = this->dcm_FB.transpose() * vRotLHS * this->AEff * this->dcm_FB;
    backSubContr.matrixD = this->dcm_FB.transpose() * vRotLHS * this->BEff * this->dcm_FB;
    backSubContr.vecTrans = - this->dcm_FB.transpose() * vTransLHS * this->CEff;
    backSubContr.vecRot = - this->dcm_FB.transpose() * vRotLHS * this->CEff;
}

void StateSpaceStateEffector::computeDerivatives(double integTime,
                                                 Eigen::Vector3d rDDot_BN_N,
                                                 Eigen::Vector3d omegaDot_BN_B,
                                                 Eigen::Vector3d sigma_BN)
{
    // Grab states
    this->XDot = this->XDotState->getState();
    this->sigma_BN = sigma_BN;

    // Compute intermediate variables
    Eigen ::Matrix3d dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();
    Eigen::Vector3d rDDot_BN_F = this->dcm_FB * dcm_BN * rDDot_BN_N;
    Eigen::Vector3d omegaDot_BN_F = this->dcm_FB * omegaDot_BN_B;

    // Compute state derivatives
    Eigen::VectorXd XDDot = this->AEff * rDDot_BN_F + this->BEff * omegaDot_BN_F + this->CEff;
    this->XState->setDerivative(this->XDot);
    this->XDotState->setDerivative(XDDot);
}

void StateSpaceStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d &rotAngMomPntCContr_B,
                                                          double &rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    // Grabe states
    this->X = this->XState->getState();
    this->XDot = this->XDotState->getState();

    // Compute state space energy
    rotEnergyContr = 0.5 * this->XDot.dot(this->M * this->XDot) + 0.5 * this->X.dot(this->K * this->X);
}
