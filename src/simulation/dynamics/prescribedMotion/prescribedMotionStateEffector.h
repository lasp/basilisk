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

#ifndef PRESCRIBED_MOTION_STATE_EFFECTOR_H
#define PRESCRIBED_MOTION_STATE_EFFECTOR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/PrescribedRotationMsgPayload.h"
#include "architecture/msgPayloadDefC/PrescribedTranslationMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"

/*! @brief Prescribed motion state effector class */
class PrescribedMotionStateEffector: public StateEffector, public SysModel {
public:
    PrescribedMotionStateEffector();                                                //!< Constructor
    ~PrescribedMotionStateEffector();                                               //!< Destructor

    void setMass(const double mass);                                                //!< Setter method for the effector mass
    void setIPntFc_F(const Eigen::Matrix3d IPntFc_F);                               //!< Setter method for IPntFc_F
    void setR_FcF_F(const Eigen::Vector3d r_FcF_F);                                 //!< Setter method for r_FcF_F
    void setR_FM_M(const Eigen::Vector3d r_FM_M);                                   //!< Setter method for r_FM_M
    void setRPrime_FM_M(const Eigen::Vector3d rPrime_FM_M);                         //!< Setter method for rPrime_FM_M
    void setRPrimePrime_FM_M(const Eigen::Vector3d rPrimePrime_FM_M);               //!< Setter method for rPrimePrime_FM_M
    void setOmega_FM_F(const Eigen::Vector3d omega_FM_F);                           //!< Setter method for omega_FM_F
    void setOmegaPrime_FM_F(const Eigen::Vector3d omegaPrime_FM_F);                 //!< Setter method for omegaPrime_FM_F
    void setSigma_FM(const Eigen::MRPd sigma_FM);                                   //!< Setter method for sigma_FM
    void setR_MB_B(const Eigen::Vector3d r_MB_B);                                   //!< Setter method for r_MB_B
    void setOmega_MB_M(const Eigen::Vector3d omega_MB_M);                           //!< Setter method omega_MB_M
    void setOmegaPrime_MB_B(const Eigen::Vector3d omegaPrime_MB_B);                 //!< Setter method for omegaPrime_MB_B
    void setSigma_MB(const Eigen::MRPd sigma_MB);                                   //!< Setter method for sigma_MB

    double getMass() const;                                                         //!< Getter method for the effector mass
    const Eigen::Matrix3d getIPntFc_F() const;                                      //!< Getter method for IPntFc_F
    const Eigen::Vector3d getR_FcF_F() const;                                       //!< Getter method for r_FcF_F
    const Eigen::Vector3d getR_FM_M() const;                                        //!< Getter method for r_FM_M
    const Eigen::Vector3d getRPrime_FM_M() const;                                   //!< Getter method for rPrime_FM_M
    const Eigen::Vector3d getRPrimePrime_FM_M() const;                              //!< Getter method for rPrimePrime_FM_M
    const Eigen::Vector3d getOmega_FM_F() const;                                    //!< Getter method for omega_FM_F
    const Eigen::Vector3d getOmegaPrime_FM_F() const;                               //!< Getter method for omegaPrime_FM_F
    const Eigen::MRPd getSigma_FM() const;                                          //!< Getter method for sigma_FM
    const Eigen::Vector3d getR_MB_B() const;                                        //!< Getter method for r_MB_B
    const Eigen::Vector3d getOmega_MB_M() const;                                    //!< Getter method omega_MB_M
    const Eigen::Vector3d getOmegaPrime_MB_B() const;                               //!< Getter method for omegaPrime_MB_B
    const Eigen::MRPd getSigma_MB() const;                                          //!< Getter method for sigma_MB

    void Reset(uint64_t callTime) override;                                         //!< Reset method
    void UpdateState(uint64_t callTime) override;                                   //!< Method for updating the effector's states
    void computeDerivatives(double callTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::Vector3d sigma_BN) override;                     //!< Method for computing the effector's MRP attitude state derivative
    void computePrescribedMotionInertialStates();                                   //!< Method for computing the effector's inertial states
    void linkInStates(DynParamManager& states) override;                            //!< Method for giving the effector access to hub states
    void registerStates(DynParamManager& statesIn) override;                        //!< Method for registering the effector's states
    void updateContributions(double callTime,
                             BackSubMatrices & backSubContr,
                             Eigen::Vector3d sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N) override;                         //!< Method for computing the effector's backsubstitution contributions
    void updateEffectorMassProps(double callTime) override;                         //!< Method for providing the effector's contributions to the mass properties and mass property rates of the spacecraft
    void updateEnergyMomContributions(double callTime,
                                      Eigen::Vector3d & rotAngMomPntCContr_B,
                                      double & rotEnergyContr,
                                      Eigen::Vector3d omega_BN_B) override;         //!< Method for computing the effector's contributions to the energy and momentum of the spacecraft
    void writeOutputStateMessages(uint64_t callTime) override;                      //!< Method for writing the module's output messages

    ReadFunctor<PrescribedRotationMsgPayload> prescribedRotationInMsg;              //!< Input message for the effector's hub-relative rotational prescribed states
    ReadFunctor<PrescribedTranslationMsgPayload> prescribedTranslationInMsg;        //!< Input message for the effector's hub-relative translational prescribed states
    Message<PrescribedRotationMsgPayload> prescribedRotationOutMsg;                 //!< Output message for the effector's hub-relative rotational prescribed states
    Message<PrescribedTranslationMsgPayload> prescribedTranslationOutMsg;           //!< Output message for the effector's hub-relative translational prescribed states
    Message<SCStatesMsgPayload> prescribedMotionConfigLogOutMsg;                    //!< Output config log message for the effector's inertial states

private:
    // User-configurable module variables
    double mass;                                                                    //!< [kg] Effector mass
    Eigen::Matrix3d IPntFc_F;                                                       //!< [kg-m^2] Effector's inertia matrix about its center of mass point Fc expressed in F frame components
    Eigen::Vector3d r_FcF_F;                                                        //!< [m] Position vector of the effector's center of mass point Fc relative to the effector's body frame origin point F expressed in F frame components
    Eigen::Vector3d r_FM_M;                                                         //!< [m] Position vector of the effector's body frame origin point F relative to the hub-fixed mount frame origin point M expressed in M frame components
    Eigen::Vector3d rPrime_FM_M;                                                    //!< [m/s] B frame time derivative of r_FM_M expressed in M frame components
    Eigen::Vector3d rPrimePrime_FM_M;                                               //!< [m/s^2] B frame time derivative of rPrime_FM_M expressed in M frame components
    Eigen::Vector3d omega_FM_F;                                                     //!< [rad/s] Angular velocity of the effector body frame F relative to the hub-fixed mount frame M expressed in F frame components
    Eigen::Vector3d omegaPrime_FM_F;                                                //!< [rad/s^2] Angular acceleration of the effector body frame F relative to the hub-fixed mount frame M expressed in F frame components
    Eigen::MRPd sigma_FM;                                                           //!< MRP attitude of the effector's body frame F relative to the hub-fixed mount frame M
    Eigen::Vector3d r_MB_B;                                                         //!< [m] Position vector describing the hub-fixed mount frame origin point M location relative to the hub frame origin point B expressed in B frame components
    Eigen::Vector3d omega_MB_M;                                                     //!< [rad/s] Angular velocity of the hub-fixed mount frame M relative to the hub frame B expressed in M frame components
    Eigen::Vector3d omegaPrime_MB_B;                                                //!< [rad/s^2] Angular acceleration of the hub-fixed mount frame M relative to the hub frame B expressed in B frame components
    Eigen::MRPd sigma_MB;                                                           //!< MRP attitude of the hub-fixed frame M relative to the hub body frame B

    // Other module vectors
    Eigen::Vector3d r_FcF_B;                                                        //!< [m] Position vector of the effector center of mass point Fc relative to point F expressed in B frame components
    Eigen::Vector3d r_FcB_B;                                                        //!< [m] Position vector of the effector center of mass point Fc relative to point B expressed in B frame components
    Eigen::Vector3d rPrime_FcB_B;                                                   //!< [m/s] B frame time derivative of r_FcB_B expressed in B frame components
    Eigen::Vector3d rDot_FcB_B;                                                     //!< [m/s] Inertial time derivative of r_FcB_B expressed in B frame components
    Eigen::Vector3d rPrimePrime_FM_B;                                               //!< [m/s^2] B frame time derivative of rPrime_FM_B expressed in B frame components
    Eigen::Vector3d omega_FB_B;                                                     //!< [rad/s] Angular velocity of frame F relative to frame B expressed in B frame components
    Eigen::Vector3d omega_BN_B;                                                     //!< [rad/s] Angular velocity of frame B relative to the inertial frame N expressed in B frame components
    Eigen::Vector3d omega_FN_B;                                                     //!< [rad/s] Angular velocity of frame F relative to the inertial frame N expressed in B frame components
    Eigen::Vector3d omegaPrime_FB_B;                                                //!< [rad/s^2] B frame time derivative of omega_FB_B expressed in B frame components

    // Matrix quantities
    Eigen::Matrix3d dcm_BF;                                                         //!< DCM attitude of frame B relative frame F
    Eigen::Matrix3d dcm_BN;                                                         //!< DCM attitude of frame B relative frame N
    Eigen::Matrix3d IPntFc_B;                                                       //!< [kg-m^2] Effector inertia matrix about its center of mass point Fc expressed in B frame components
    Eigen::Matrix3d rTilde_FcB_B;                                                   //!< [m] Tilde cross product matrix of r_FcB_B
    Eigen::Matrix3d omegaTilde_BN_B;                                                //!< [rad/s] Tilde cross product matrix of omega_BN_B
    Eigen::Matrix3d omegaTilde_FB_B;                                                //!< [rad/s] Tilde cross product matrix of omega_FB_B

    // Effector inertial states
    Eigen::Vector3d r_FcN_N;                                                        //!< [m] Position vector of the effector center of mass point Fc relative to the inertial frame origin point N expressed in inertial frame N components
    Eigen::Vector3d v_FcN_N;                                                        //!< [m/s] Velocity vector of the effector center of mass point Fc relative to the inertial frame origin point N expressed in inertial frame N components
    Eigen::Vector3d sigma_FN;                                                       //!< MRP attitude of the effector body frame F relative to the inertial frame N
    Eigen::Vector3d omega_FN_F;                                                     //!< [rad/s] Angular velocity of frame F relative to the inertial frame N expressed in F frame components

    // Hub inertial states
    Eigen::MRPd sigma_BN;                                                           //!< MRP attitude of the hub body frame B relative to the inertial frame N
    StateData *hubSigma;                                                            //!< sigma_BN state data
    StateData *hubOmega;                                                            //!< [rad/s] omega_BN_B state data
    Eigen::MatrixXd* inertialPositionProperty;                                      //!< [m] r_N Hub inertial position vector relative to system spice zeroBase/refBase
    Eigen::MatrixXd* inertialVelocityProperty;                                      //!< [m] v_N Hub inertial velocity vector relative to system spice zeroBase/refBase

    // Prescribed states at epoch (Dynamics time step)
    Eigen::Vector3d rEpoch_FM_M;                                                    //!< [m] Current r_FM_M at the dynamics time step
    Eigen::Vector3d rPrimeEpoch_FM_M;                                               //!< [m/s] Current rPrime_FM_M at the dynamics time step
    Eigen::Vector3d omegaEpoch_FM_F;                                                //!< [rad/s] Current omega_FM_F at the dynamics time step
    StateData *sigma_FMState;                                                       //!< sigma_FM state data

    double currentSimTimeSec;                                                       //!< [s] Current simulation time, updated at the dynamics time step
    std::string nameOfsigma_FMState;                                                //!< Identifier for the effector MRP attitude sigma_FM state data container
    static uint64_t effectorID;                                                     //!< Effector ID
};

#endif /* PRESCRIBED_MOTION_STATE_EFFECTOR_H */
