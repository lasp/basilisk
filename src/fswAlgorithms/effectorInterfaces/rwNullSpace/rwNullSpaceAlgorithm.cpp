// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "rwNullSpaceAlgorithm.h"

#include <architecture/utilities/eigenSupport.h>

#include <stdexcept>

/*! @brief This resets the module to original states by reading in the RW configuration messages and recreating any
   module specific variables.  The output message is reset to zero.
    @return void
    @param rwConfigInMsg Reaction Wheel constellation input message
 */
void RwNullSpaceAlgorithm::reset(RWConstellationMsgPayload &rwConfigInMsg) {
    this->numWheels = (uint32_t) rwConfigInMsg.numRW;

    Eigen::Matrix<double, 3, RW_EFF_CNT> G_s_B{};
    G_s_B.setZero();
    for (uint32_t i = 0; i < this->numWheels; i = i + 1) {
        G_s_B.col(i) = cArrayToEigenVector(rwConfigInMsg.reactionWheels[i].gsHat_B);
    }

    /* find the [tau] null space projection matrix [tau] = ([I] - [Gs]^T.([Gs].[Gs]^T)^-1.[Gs]) */
    this->tau = Eigen::Matrix<double, RW_EFF_CNT, RW_EFF_CNT>::Identity()
              - G_s_B.transpose() * (G_s_B * G_s_B.transpose()).inverse() * G_s_B;
}

/*! This method takes the input reaction wheel commands as well as the observed
    reaction wheel speeds and balances the commands so that the overall vehicle
    momentum is minimized.
 @return RwMotorTorqueMsgPayload
 @param controlRequest array of RW torques requested by control law
 @param rwSpeeds array of wheel speeds
 @param rwDesiredSpeeds array of desired wheel speeds
 */
RwMotorTorqueMsgPayload RwNullSpaceAlgorithm::update(
    RwMotorTorqueMsgPayload &controlRequest,
    RWSpeedMsgPayload &rwSpeeds,
    RWSpeedMsgPayload &rwDesiredSpeeds
) {
    RwMotorTorqueMsgPayload finalControl{}; /* [Nm]  array of final RW motor torques containing both
                                            the control and null motion torques */

    /* compute the wheel speed control vector d = -K.DeltaOmega */
    Eigen::Vector<double, MAX_EFF_CNT> d =
        -this->omegaGain
        * (cArrayToEigenVector(rwSpeeds.wheelSpeeds) - cArrayToEigenVector(rwDesiredSpeeds.wheelSpeeds));

    /* compute the RW null space motor torque solution to reduce the wheel speeds */
    Eigen::Vector<double, MAX_EFF_CNT> motorTorqueNullSpace = this->tau * d;

    /* add the null motion RW torque solution to the RW feedback control torque solution */
    Eigen::Vector<double, MAX_EFF_CNT> motorTorque =
        motorTorqueNullSpace + cArrayToEigenVector(controlRequest.motorTorque);

    eigenVectorToCArray(motorTorque, finalControl.motorTorque);

    return finalControl;
}

/**
 * @brief Set the gain used for the wheel speed difference.
 * @param gain The gain used for the wheel speed difference.
 */
void RwNullSpaceAlgorithm::setOmegaGain(double const gain) {
    if (gain < 0.0) { throw std::invalid_argument("Feedback gain must not be negative"); }
    this->omegaGain = gain;
}

/**
 * @brief Get the gain used for the wheel speed difference.
 * @return double The gain used for the wheel speed difference.
 */
double RwNullSpaceAlgorithm::getOmegaGain() const {
    return this->omegaGain;
}
