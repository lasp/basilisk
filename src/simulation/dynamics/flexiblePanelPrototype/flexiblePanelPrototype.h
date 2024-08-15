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

#ifndef FLEXIBLE_PANEL_PROTOTYPE_H
#define FLEXIBLE_PANEL_PROTOTYPE_H

#include <Eigen/Dense>
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief spinning body state effector class */
class FlexiblePanelPrototype: public StateEffector, public SysModel {
public:
    FlexiblePanelPrototype();      //!< -- Contructor
    ~FlexiblePanelPrototype();     //!< -- Destructor
    void Reset(uint64_t CurrentClock);      //!< -- Method for reset
    void UpdateState(uint64_t CurrentSimNanos);             //!< -- Method for updating information
    void registerStates(DynParamManager& statesIn);         //!< -- Method for registering the SB states
    void linkInStates(DynParamManager& states);             //!< -- Method for getting access to other states
    void updateEffectorMassProps(double integTime) override;
    void updateContributions(double integTime,
                             BackSubMatrices& backSubContr,
                             Eigen::Vector3d sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N);  //!< -- Method for back-substitution contributions
    void computeDerivatives(double integTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::Vector3d sigma_BN);                         //!< -- Method for SB to compute its derivatives
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d &rotAngMomPntCContr_B,
                                      double &rotEnergyContr, Eigen::Vector3d omega_BN_B);
    void prependSpacecraftNameToStates();                   //!< Method used for multiple spacecraft

    double xInit = 0.0;
    double xDotInit = 0.0;
    double yInit = 0.0;
    double yDotInit = 0.0;
    double zInit = 0.0;
    double zDotInit = 0.0;
    double theta1Init = 0.0;            //!< [rad] initial first axis angle
    double theta1DotInit = 0.0;         //!< [rad/s] initial first axis angle rate
    double theta2Init = 0.0;            //!< [rad] initial second axis angle
    double theta2DotInit = 0.0;         //!< [rad/s] initial second axis angle rate
    double theta3Init = 0.0;            //!< [rad] initial second axis angle
    double theta3DotInit = 0.0;         //!< [rad/s] initial second axis angle rate
    double beta1Init = 0.0;            //!< [rad] initial second axis angle
    double beta1DotInit = 0.0;         //!< [rad/s] initial second axis angle rate
    std::string nameOfXState;      //!< -- identifier for the theta1 state data container
    std::string nameOfXDotState;   //!< -- identifier for the thetaDot1 state data container

    double mass1 = 1.0;
    double mass2 = 1.0;
    double kX = 0.0;
    double kY = 0.0;
    double kZ = 0.0;
    double kTheta1 = 0.0;
    double kTheta2 = 0.0;
    double kTheta3 = 0.0;
    double kBeta1 = 0.0;
    double cX = 0.0;
    double cY = 0.0;
    double cZ = 0.0;
    double cTheta1 = 0.0;
    double cTheta2 = 0.0;
    double cTheta3 = 0.0;
    double cBeta1 = 0.0;

    Eigen::Matrix3d dcm_FB = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d IS1PntSc1_F = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d IS2PntSc2_F = Eigen::Matrix3d::Identity();
    Eigen::Vector3d r_FB_B = Eigen::Vector3d::Zero();
    Eigen::Vector3d r_Sc1S1_F = Eigen::Vector3d::Zero();
    Eigen::Vector3d r_Sc2S2_F = Eigen::Vector3d::Zero();
    Eigen::Vector3d r_S2S1_F = Eigen::Vector3d::Zero();

    Eigen::VectorXd X;
    Eigen::VectorXd XDot;

private:
    static uint64_t effectorID;     //!< [] ID number of this panel

    Eigen::MatrixXd* m_SC = nullptr;
    Eigen::MatrixXd* c_B = nullptr;
    Eigen::MatrixXd* cPrime_B = nullptr;
    Eigen::MatrixXd* inertialPositionProperty = nullptr;    //!< [m] r_N inertial position relative to system spice zeroBase/refBase
    Eigen::MatrixXd* inertialVelocityProperty = nullptr;    //!< [m] v_N inertial velocity relative to system spice zeroBase/refBase
    StateData *XState = nullptr;    //!< -- state manager of theta1 for spinning body
    StateData *XDotState = nullptr; //!< -- state manager of theta1Dot for spinning body

    Eigen::Vector3d xHat_F{1, 0, 0};
    Eigen::Vector3d yHat_F{0, 1, 0};
    Eigen::Vector3d zHat_F{0, 0, 1};
    Eigen::Vector3d xHat_B{1, 0, 0};
    Eigen::Vector3d yHat_B{0, 1, 0};
    Eigen::Vector3d zHat_B{0, 0, 1};

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(7, 7);
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(7, 7);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(7, 7);
    Eigen::VectorXd p = Eigen::VectorXd::Zero(7);

    Eigen::MatrixXd AEff = Eigen::MatrixXd::Zero(7, 3);
    Eigen::MatrixXd BEff = Eigen::MatrixXd::Zero(7, 3);
    Eigen::VectorXd CEff = Eigen::VectorXd::Zero(7);

    Eigen::Vector3d omega_BN_B = Eigen::Vector3d::Zero();
    Eigen::MRPd sigma_BN;

    double mass = 1.0;
    Eigen::Vector3d r_ScS1_F = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rTilde_Sc1S1_F = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d rTilde_Sc2S2_F = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d rTilde_S2S1_F = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d rTilde_ScS1_F = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d ISPntS1_F = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d IS2PntS2_F = Eigen::Matrix3d::Identity();



    double computeStateSpaceEnergy();
    double computeForcingTermEnergy();
};

#endif /* FLEXIBLE_PANEL_PROTOTYPE_H */
