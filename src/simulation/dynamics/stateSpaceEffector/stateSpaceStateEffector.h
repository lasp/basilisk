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

#ifndef STATE_SPACE_STATE_EFFECTOR_H
#define STATE_SPACE_STATE_EFFECTOR_H

#include <Eigen/Dense>
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"

class StateSpaceStateEffector: public StateEffector, public SysModel {
public:
    explicit StateSpaceStateEffector(int n);
    ~StateSpaceStateEffector() override;
    void registerStates(DynParamManager& statesIn) override;
    void linkInStates(DynParamManager& states) override;
    void updateEffectorMassProps(double integTime) override;
    void updateContributions(double integTime,
                             BackSubMatrices& backSubContr,
                             Eigen::Vector3d sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N) override;
    void computeDerivatives(double integTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::Vector3d sigma_BN) override;
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d &rotAngMomPntCContr_B,
                                      double &rotEnergyContr, Eigen::Vector3d omega_BN_B) override;
    void prependSpacecraftNameToStates() override;

    Eigen::VectorXd XInit;
    Eigen::VectorXd XDotInit;
    std::string nameOfXState;
    std::string nameOfXDotState;

    Eigen::MatrixXd M;
    Eigen::MatrixXd K;
    Eigen::MatrixXd C;

    double mass = 1.0;
    Eigen::Matrix3d dcm_FB = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d ISPntSc_F = Eigen::Matrix3d::Identity();
    Eigen::Vector3d r_FB_B = Eigen::Vector3d::Zero();
    Eigen::Vector3d r_ScS_F = Eigen::Vector3d::Zero();

private:
    static uint64_t effectorID;

    int sizeOfStateSpace;
    Eigen::VectorXd X;
    Eigen::VectorXd XDot;

    Eigen::MatrixXd* m_SC = nullptr;
    Eigen::MatrixXd* c_B = nullptr;
    Eigen::MatrixXd* inertialPositionProperty = nullptr;
    Eigen::MatrixXd* inertialVelocityProperty = nullptr;
    StateData *XState = nullptr;
    StateData *XDotState = nullptr;

    Eigen::VectorXd p;
    Eigen::MatrixXd AEff;
    Eigen::MatrixXd BEff;
    Eigen::VectorXd CEff;

    Eigen::Vector3d omega_BN_B = Eigen::Vector3d::Zero();
    Eigen::MRPd sigma_BN;
};

#endif /* STATE_SPACE_STATE_EFFECTOR_H */
