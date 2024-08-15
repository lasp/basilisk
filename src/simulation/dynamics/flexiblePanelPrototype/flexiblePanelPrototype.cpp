/*
 ISC License

 Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "flexiblePanelPrototype.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <string>
#include <iostream>

/*! This is the constructor, setting variables to default values */
FlexiblePanelPrototype::FlexiblePanelPrototype()
{
    // Zero the mass props and mass prop rates contributions
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);
    
    this->nameOfXState = "flexiblePanelTheta" + std::to_string(FlexiblePanelPrototype::effectorID);
    this->nameOfXDotState = "flexiblePanelThetaDot" + std::to_string(FlexiblePanelPrototype::effectorID);
    FlexiblePanelPrototype::effectorID++;
}

uint64_t FlexiblePanelPrototype::effectorID = 1;

/*! This is the destructor, nothing to report here */
FlexiblePanelPrototype::~FlexiblePanelPrototype()
{
    FlexiblePanelPrototype::effectorID --;    /* reset the panel ID*/
}

/*! This method is used to reset the module. */
void FlexiblePanelPrototype::Reset(uint64_t CurrentClock)
{
    this->xHat_B = this->dcm_FB.transpose() * this->xHat_F;
    this->yHat_B = this->dcm_FB.transpose() * this->yHat_F;
    this->zHat_B = this->dcm_FB.transpose() * this->zHat_F;

    this->mass = this->mass1 + this->mass2;
    Eigen::Vector3d r_Sc2S1_F = this->r_Sc2S2_F + this->r_S2S1_F;
    this->r_ScS1_F = (this->mass1 * this->r_Sc1S1_F + this->mass2 * r_Sc2S1_F) / this->mass;

    this->rTilde_Sc1S1_F = eigenTilde(this->r_Sc1S1_F);
    this->rTilde_Sc2S2_F = eigenTilde(this->r_Sc2S2_F);
    this->rTilde_S2S1_F = eigenTilde(this->r_S2S1_F);
    Eigen::Matrix3d rTilde_Sc2S1_F = eigenTilde(r_Sc2S1_F);
    this->rTilde_ScS1_F = eigenTilde(this->r_ScS1_F);

    this->ISPntS1_F = this->IS1PntSc1_F - this->mass1 * this->rTilde_Sc1S1_F * this->rTilde_Sc1S1_F
            + this->IS2PntSc2_F - this->mass2 * rTilde_Sc2S1_F * rTilde_Sc2S1_F;
    this->IS2PntS2_F = this->IS2PntSc2_F - this->mass2 * this->rTilde_Sc2S2_F * this->rTilde_Sc2S2_F;

    this->M.block<3, 3>(0, 0) = this->mass * Eigen::Matrix3d::Identity();
    this->M.block<3, 3>(0, 3) = - this->mass * this->rTilde_ScS1_F;
    this->M.block<3, 1>(0, 6) = - this->mass2 * this->rTilde_Sc2S2_F * this->yHat_F;
    this->M.block<3, 3>(3, 0) = this->mass * this->rTilde_ScS1_F;
    this->M.block<3, 3>(3, 3) = this->ISPntS1_F;
    this->M.block<3, 1>(3, 6) = (this->IS2PntS2_F - this->mass2 * this->rTilde_S2S1_F * this->rTilde_Sc2S2_F) * this->yHat_F;
    this->M.block<1, 3>(6, 0) = this->mass2 * this->yHat_F.transpose() * this->rTilde_Sc2S2_F;
    this->M.block<1, 3>(6, 3) = this->yHat_F.transpose() * (this->IS2PntS2_F + this->mass2 * this->rTilde_Sc2S2_F * this->rTilde_S2S1_F);
    this->M.block<1, 1>(6, 6) = this->yHat_F.transpose() * this->IS2PntS2_F * this->yHat_F;

    this->K.diagonal() << this->kX, this->kY, this->kZ, this->kTheta1, this->kTheta2, this->kTheta3, this->kBeta1;
    this->C.diagonal() << this->cX, this->cY, this->cZ, this->cTheta1, this->cTheta2, this->cTheta3, this->cBeta1;
}

/*! This method prepends the name of the spacecraft for multi-spacecraft simulations.*/
void FlexiblePanelPrototype::prependSpacecraftNameToStates()
{
    this->nameOfXState = this->nameOfSpacecraftAttachedTo + this->nameOfXState;
    this->nameOfXDotState = this->nameOfSpacecraftAttachedTo + this->nameOfXDotState;
}

/*! This method allows the SB state effector to have access to the hub states and gravity*/
void FlexiblePanelPrototype::linkInStates(DynParamManager& statesIn)
{
    this->m_SC = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "m_SC");
    this->c_B = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "centerOfMassSC");
    this->cPrime_B = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "centerOfMassPrimeSC");
    this->inertialPositionProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "r_BN_N");
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "v_BN_N");
}

/*! This method allows the SB state effector to register its states: theta and thetaDot with the dynamic parameter manager */
void FlexiblePanelPrototype::registerStates(DynParamManager& states)
{
    this->XState = states.registerState(7, 1, this->nameOfXState);
    this->XDotState = states.registerState(7, 1, this->nameOfXDotState);

    Eigen::MatrixXd XInitMatrix(7,1);
    XInitMatrix << this->xInit, this->yInit, this->zInit, this->theta1Init, this->theta2Init, this->theta3Init, this->beta1Init;
    Eigen::MatrixXd XDotInitMatrix(7,1);
    XDotInitMatrix << this->xDotInit, this->yDotInit, this->zDotInit, this->theta1DotInit, this->theta2DotInit, this->theta3DotInit, this->beta1DotInit;

    this->XState->setState(XInitMatrix);
    this->XDotState->setState(XDotInitMatrix);
}

void FlexiblePanelPrototype::updateEffectorMassProps(double integTime)
{
    this->X = this->XState->getState();
    this->XDot = this->XDotState->getState();

    Eigen::Vector3d r_S1F_F = this->X.head(3);
    Eigen::Vector3d r_ScB_B = this->dcm_FB.transpose() * (this->r_ScS1_F + r_S1F_F) + this->r_FB_B;
    Eigen::Vector3d r_Sc1B_B =  this->dcm_FB.transpose() * (this->r_Sc1S1_F + r_S1F_F) + this->r_FB_B;
    Eigen::Vector3d r_Sc2B_B =  this->dcm_FB.transpose() * (this->r_Sc2S2_F + this->r_S2S1_F + r_S1F_F) + this->r_FB_B;

    Eigen::Matrix3d rTilde_Sc1B_B = eigenTilde(r_Sc1B_B);
    Eigen::Matrix3d rTilde_Sc2B_B = eigenTilde(r_Sc2B_B);
    Eigen::Matrix3d ISPntB_B = this->dcm_FB.transpose() * this->IS1PntSc1_F * this->dcm_FB
            - this->mass1 * rTilde_Sc1B_B * rTilde_Sc1B_B
            + this->dcm_FB.transpose() * this->IS2PntSc2_F * this->dcm_FB
            - this->mass2 * rTilde_Sc2B_B * rTilde_Sc2B_B;

    this->effProps.mEff = this->mass;
    this->effProps.rEff_CB_B = r_ScB_B;
//    this->effProps.rEffPrime_CB_B = this->dcm_FB.transpose() * this->XDot.head(3);
    this->effProps.IEffPntB_B = ISPntB_B;
}


/*! This method allows the SB state effector to give its contributions to the matrices needed for the back-sub 
 method */
void FlexiblePanelPrototype::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
    this->X = this->XState->getState();
    this->XDot = this->XDotState->getState();
    this->sigma_BN = sigma_BN;
    this->omega_BN_B = omega_BN_B;

    Eigen::Vector3d omega_BN_F = this->dcm_FB * this->omega_BN_B;
    Eigen::Vector3d r_S1F_F = this->X.head(3);
    Eigen::Vector3d r_S1B_F = r_S1F_F + this->dcm_FB * this->r_FB_B;
    Eigen::Vector3d r_S2B_F = r_S1B_F + this->r_S2S1_F;
    Eigen::Vector3d r_ScB_F = this->r_ScS1_F + r_S1B_F;
    Eigen::Matrix3d omegaTilde_BN_F = eigenTilde(omega_BN_F);
    Eigen::Matrix3d rTilde_S1B_F = eigenTilde(r_S1B_F);
    Eigen::Matrix3d rTilde_S2B_F = eigenTilde(r_S2B_F);
    Eigen::Matrix3d rTilde_ScB_F = eigenTilde(r_ScB_F);

    Eigen::MatrixXd AEffStar = Eigen::MatrixXd::Zero(7, 3);
    Eigen::MatrixXd BEffStar = Eigen::MatrixXd::Zero(7, 3);
    Eigen::VectorXd CEffStar = Eigen::VectorXd::Zero(7);

    AEffStar.block<3,3>(0, 0) = - this->mass * Eigen::Matrix3d::Identity();
    AEffStar.block<3, 3>(3, 0) = - this->mass * this->rTilde_ScS1_F;
    AEffStar.block<1, 3>(6, 0) = - this->mass2 * this->yHat_F.transpose() * this->rTilde_Sc2S2_F;

    BEffStar.block<3,3>(0, 0) = this->mass * rTilde_ScB_F;
    BEffStar.block<3, 3>(3, 0) = - (this->ISPntS1_F - this->mass * this->rTilde_ScS1_F * rTilde_S1B_F);
    BEffStar.block<1, 3>(6, 0) = - this->yHat_F.transpose() * (this->IS2PntS2_F
                                                               - this->mass2 * this->rTilde_Sc2S2_F * rTilde_S2B_F);

    CEffStar.block<3,1>(0, 0) = - this->mass * omegaTilde_BN_F * omegaTilde_BN_F * r_ScB_F
                                - this->K.block<3, 3>(0, 0) * this->X.block<3, 1>(0, 0)
                                - this->C.block<3, 3>(0, 0) * this->XDot.block<3, 1>(0, 0);
    CEffStar.block<3,1>(3, 0) = - omegaTilde_BN_F * this->ISPntS1_F * omega_BN_F
                                - this->mass * this->rTilde_ScS1_F * omegaTilde_BN_F * omegaTilde_BN_F * r_S1B_F
                                - this->K.block<3, 3>(3, 3) * this->X.block<3, 1>(3, 0)
                                - this->C.block<3, 3>(3, 3) * this->XDot.block<3, 1>(3, 0);
    CEffStar.block<1,1>(6, 0) = - this->yHat_F.transpose() *omegaTilde_BN_F * this->IS2PntS2_F * omega_BN_F
                                - this->yHat_F.transpose() * this->mass2 * this->rTilde_Sc2S2_F * omegaTilde_BN_F * omegaTilde_BN_F * r_S2B_F
                                - this->K.block<1, 1>(6, 6) * this->X.block<1, 1>(6, 0)
                                - this->C.block<1, 1>(6, 6) * this->XDot.block<1, 1>(6, 0);

    this->AEff = this->M.inverse() * AEffStar;
    this->BEff = this->M.inverse() * BEffStar;
    this->CEff = this->M.inverse() * CEffStar;

    Eigen::MatrixXd vTransLHS = this->M.block<3, 7>(0, 0);
    Eigen::MatrixXd vRotLHS = this->M.block<3, 7>(3, 0) + rTilde_S1B_F * vTransLHS;

    backSubContr.matrixA = this->dcm_FB.transpose() * vTransLHS * this->AEff * this->dcm_FB;
    backSubContr.matrixB = this->dcm_FB.transpose() * vTransLHS * this->BEff * this->dcm_FB;
    backSubContr.matrixC = this->dcm_FB.transpose() * vRotLHS * this->AEff * this->dcm_FB;
    backSubContr.matrixD = this->dcm_FB.transpose() * vRotLHS * this->BEff * this->dcm_FB;
    backSubContr.vecTrans = - this->dcm_FB.transpose() * vTransLHS * this->CEff;
    backSubContr.vecRot = - this->dcm_FB.transpose() * vRotLHS * this->CEff;
}

/*! This method is used to find the derivatives for the SB stateEffector: thetaDDot and the kinematic derivative */
void FlexiblePanelPrototype::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{
    this->X = this->XState->getState();
    this->XDot = this->XDotState->getState();
    this->sigma_BN = sigma_BN;
    Eigen ::Matrix3d dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();
    Eigen::Vector3d omega_BN_F = this->dcm_FB * this->omega_BN_B;
    Eigen::Vector3d rDDot_BN_F = this->dcm_FB * dcm_BN * rDDot_BN_N;
    Eigen::Vector3d omegaDot_BN_F = this->dcm_FB * omegaDot_BN_B;

    Eigen::VectorXd XDDot = this->AEff * rDDot_BN_F + this->BEff * omegaDot_BN_F + this->CEff;
    this->XState->setDerivative(this->XDot);
    this->XDotState->setDerivative(XDDot);
}

void FlexiblePanelPrototype::updateEnergyMomContributions(double integTime, Eigen::Vector3d &rotAngMomPntCContr_B,
                                                          double &rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    this->X = this->XState->getState();
    this->XDot = this->XDotState->getState();
    this->omega_BN_B = omega_BN_B;

    rotEnergyContr = 0.0;
    rotEnergyContr += computeStateSpaceEnergy();
//    rotEnergyContr += computeForcingTermEnergy();
}

double FlexiblePanelPrototype::computeStateSpaceEnergy()
{
    return 0.5 * this->XDot.dot(this->M * this->XDot) + 0.5 * this->X.dot(this->K * this->X);
}

double FlexiblePanelPrototype::computeForcingTermEnergy()
{
    Eigen::Matrix3d omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    Eigen::Vector3d rDot_FB_B = omegaTilde_BN_B * this->r_FB_B;

    Eigen::Matrix3d ISPntSc_B = this->dcm_FB.transpose() * (this->ISPntS1_F
            + this->mass * this->rTilde_ScS1_F * this->rTilde_ScS1_F) * this->dcm_FB;

    return 0.5 * omega_BN_B.dot(ISPntSc_B * omega_BN_B)
               + omega_BN_B.dot(ISPntSc_B * this->dcm_FB.transpose() * this->XDot.block<3, 1>(3, 0)
                                + this->dcm_FB.transpose() * this->IS2PntSc2_F * this->dcm_FB * this->yHat_B * this->XDot(6))
    + 0.5 * this->mass * (rDot_FB_B + omegaTilde_BN_B * this->dcm_FB.transpose() * this->X.head(3)).dot(rDot_FB_B + omegaTilde_BN_B * this->dcm_FB.transpose() * this->X.head(3))
          + this->mass * (rDot_FB_B + omegaTilde_BN_B * this->dcm_FB.transpose() * this->X.head(3)).dot(this->dcm_FB.transpose() * this->XDot.head(3));
}

void FlexiblePanelPrototype::UpdateState(uint64_t CurrentSimNanos)
{
}
