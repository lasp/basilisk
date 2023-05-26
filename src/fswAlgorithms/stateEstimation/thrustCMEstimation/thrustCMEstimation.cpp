/*
 ISC License
 
 Copyright (c) 2023, Laboratory  for Atmospheric and Space Physics, University of Colorado at Boulder
 
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

#include "thrustCMEstimation.h"
#include <iostream>

ThrustCMEstimation::ThrustCMEstimation() = default;

ThrustCMEstimation::~ThrustCMEstimation() = default;

/*! Reset the flyby OD filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void ThrustCMEstimation::Reset(uint64_t CurrentSimNanos)
{
    /*! - Check if the required message has not been connected */
    if (!this->thrusterConfigInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,  " thrusterConfigInMsg wasn't connected.");
    }
    if (!this->cmdTorqueInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,  " cmdTorqueInMsg wasn't connected.");
    }
    if (!this->attGuidInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,  " attGuidInMsg wasn't connected.");
    }

}

/*! Take the relative position measurements and outputs an estimate of the
 spacecraft states in the inertial frame.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void ThrustCMEstimation::UpdateState(uint64_t CurrentSimNanos)
{
    /*! read and allocate the thrustConfigMsg */
    THRConfigMsgPayload thrConfigBuffer = this->thrusterConfigInMsg();

    /*! compute thruster information in B-frame coordinates */
    Eigen::Vector3d r_TB_B = cArray2EigenVector3d(thrConfigBuffer.rThrust_B);
    Eigen::Vector3d T_B = cArray2EigenVector3d(thrConfigBuffer.tHatThrust_B);
    T_B = T_B * thrConfigBuffer.maxThrust;

    /*! compute error w.r.t. target attitude */
    AttGuidMsgPayload attGuidBuffer = this->attGuidInMsg();
    Eigen::Vector3d sigma_BR   = cArray2EigenVector3d(attGuidBuffer.sigma_BR);
    Eigen::Vector3d omega_BR_B = cArray2EigenVector3d(attGuidBuffer.omega_BR_B);
    this->attError = pow(sigma_BR.squaredNorm() + omega_BR_B.squaredNorm(), 0.5);

    CmdTorqueBodyMsgPayload cmdTorqueBuffer = this->cmdTorqueInMsg();
    Eigen::Vector3d u = cArray2EigenVector3d(cmdTorqueBuffer.torqueRequestBody);


    std::cout << this->attError << "\n";
}


