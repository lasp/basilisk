/*
 ISC License

 Copyright (c) 2024, University of Colorado at Boulder

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

#include "fswAlgorithms/attGuidance/flybyPoint/flybyPoint.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"

/*! This method is used to reset the module.
 @return void
 */
void FlybyPoint::Reset(uint64_t currentSimNanos)
{
    this->lastFilterReadTime = 0;
    this->firstRead = true;
}


/*! This method is the main carrier for the boresight calculation routine.  If it detects
 that it needs to re-init (direction change maybe) it will re-init itself.
 Then it will compute the angles away that the boresight is from the celestial target.
 @return void
 @param currentSimNanos The current simulation time for system
 */
void FlybyPoint::UpdateState(uint64_t currentSimNanos)
{
    /*! compute dt from current time and last filter read time and get new states*/
    this->dt = (currentSimNanos - this->lastFilterReadTime)*NANO2SEC;
    if ((this->dt >= this->timeBetweenFilterData) || this->firstRead) {
        /*! If this is the first read, seed the algorithm with the solution  */
        auto [r_BN_N, v_BN_N] = this->readRelativeState();
        if (this->firstRead){
            this->computeFlybyParameters(r_BN_N, v_BN_N);
            this->computeRN(r_BN_N, v_BN_N);
            this->firstRead = false;
        }
        /*! Protect against bad new solutions by checking validity */
        if (this->checkValidity(r_BN_N, v_BN_N)) {
            /*! update flyby parameters and guidance frame */
            this->computeFlybyParameters(r_BN_N, v_BN_N);
            this->computeRN(r_BN_N, v_BN_N);

            /*! update lastFilterReadTime to current time and dt to zero */
            this->lastFilterReadTime = currentSimNanos;
            this->dt = 0;
        }
    }

    auto [sigma_RN, omega_RN_N, omegaDot_RN_N] = this->computeGuidanceSolution();
    this->writeMessages(currentSimNanos, sigma_RN, omega_RN_N, omegaDot_RN_N);
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> FlybyPoint::readRelativeState() {
    NavTransMsgPayload relativeState = this->filterInMsg();

    Eigen::Vector3d r_BN_N = Eigen::Map<Eigen::Vector3d>(relativeState.r_BN_N);
    Eigen::Vector3d v_BN_N = Eigen::Map<Eigen::Vector3d>(relativeState.v_BN_N);

    return {r_BN_N, v_BN_N};
}

void FlybyPoint::computeFlybyParameters(Eigen::Vector3d &r_BN_N, Eigen::Vector3d &v_BN_N){
    this->f0 = v_BN_N.norm() / r_BN_N.norm();

    /*! compute radial (ur_N), velocity (uv_N), along-track (ut_N), and out-of-plane (uh_N) unit direction vectors */
    Eigen::Vector3d ur_N = r_BN_N.normalized();
    Eigen::Vector3d uv_N = v_BN_N.normalized();

    Eigen::Vector3d uh_N = ur_N.cross(uv_N).normalized();
    Eigen::Vector3d ut_N = uh_N.cross(ur_N).normalized();

    // compute flight path angle at the time of read
    this->gamma0 = std::atan(v_BN_N.dot(ur_N) / v_BN_N.dot(ut_N));
}

bool FlybyPoint::checkValidity(Eigen::Vector3d &r_BN_N, Eigen::Vector3d &v_BN_N) const{
    bool valid = true;
    Eigen::Vector3d ur_N = r_BN_N.normalized();
    Eigen::Vector3d uv_N = v_BN_N.normalized();
    /*! assert r and v are not collinear (collision trajectory) */
    if (std::abs(1 - ur_N.dot(uv_N)) < this->toleranceForCollinearity) {
         valid = false;
    }
    Eigen::Vector3d uh_N = ur_N.cross(uv_N).normalized();
    Eigen::Vector3d ut_N = uh_N.cross(ur_N).normalized();
    double gamma0 = std::atan(v_BN_N.dot(ur_N) / v_BN_N.dot(ut_N));
    double distanceClosestApproach = -r_BN_N.norm()*std::sin(gamma0);

    double maxPredictedRate = v_BN_N.norm()/distanceClosestApproach*180/M_PI;
    if (maxPredictedRate > this->maxRate && this->maxRate > 0) {
        valid = false;
    }
    double maxPredictedAcceleration = 3*std::sqrt(3)/8*pow(v_BN_N.norm()/distanceClosestApproach, 2)*180/M_PI;
    if (maxPredictedAcceleration > this->maxAcceleration && this->maxAcceleration > 0) {
        valid = false;
    }

    return valid;
}

void FlybyPoint::computeRN(Eigen::Vector3d &r_BN_N, Eigen::Vector3d &v_BN_N){
    /*! compute radial (ur_N), velocity (uv_N), along-track (ut_N), and out-of-plane (uh_N) unit direction vectors */
    Eigen::Vector3d ur_N = r_BN_N.normalized();
    Eigen::Vector3d uv_N = v_BN_N.normalized();

    Eigen::Vector3d uh_N = ur_N.cross(uv_N).normalized();
    Eigen::Vector3d ut_N = uh_N.cross(ur_N).normalized();

    /*! compute inertial-to-reference DCM at time of read */
    this->R0N.row(0) = ur_N;
    this->R0N.row(1) = ut_N;
    this->R0N.row(2) = uh_N;
}
std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> FlybyPoint::computeGuidanceSolution() const{
    /*! compute DCM (RtR0) of reference frame from last read time */
    double theta = std::atan(std::tan(this->gamma0) + this->f0 / std::cos(this->gamma0) * this->dt) - this->gamma0;
    Eigen::Vector3d PRV_theta;
    PRV_theta << 0, 0, theta;
    Eigen::Matrix3d RtR0 = prvToDcm(PRV_theta);

    /*! compute DCM of reference frame at time t_0 + dt with respect to inertial frame */
    Eigen::Matrix3d RtN = RtR0*this->R0N;

    /*! compute scalar angular rate and acceleration of the reference frame in R-frame coordinates */
    double den = (this->f0*this->f0*this->dt*this->dt + 2*this->f0*sin(this->gamma0)*this->dt + 1);
    double thetaDot = this->f0 * cos(this->gamma0) / den;
    double thetaDDot = -2*this->f0*this->f0*cos(this->gamma0) * (this->f0*this->dt + sin(this->gamma0)) / (den*den);
    Eigen::Vector3d omega_RN_R;
    omega_RN_R << 0, 0, thetaDot;
    Eigen::Vector3d omegaDot_RN_R;
    omegaDot_RN_R << 0, 0, thetaDDot;

    /*! populate attRefOut with reference frame information */
    Eigen::Vector3d sigma_RN = dcmToMrp(RtN);

    if (this->signOfOrbitNormalFrameVector == -1) {
        Eigen::Vector3d halfRotationX;
        halfRotationX << 1, 0, 0;
        sigma_RN = addMrp(sigma_RN, halfRotationX);
    }
    Eigen::Vector3d omega_RN_N = RtN.transpose()*omega_RN_R;
    Eigen::Vector3d omegaDot_RN_N = RtN.transpose()*omegaDot_RN_R;

    return {sigma_RN, omega_RN_N, omegaDot_RN_N};

}
void FlybyPoint::writeMessages(uint64_t currentSimNanos,
                   Eigen::Vector3d &sigma_RN,
                   Eigen::Vector3d &omega_RN_N,
                   Eigen::Vector3d &omegaDot_RN_N){

    AttRefMsgPayload attMsgBuffer = this->attRefOutMsg.zeroMsgPayload;

    eigenVector3d2CArray(sigma_RN, attMsgBuffer.sigma_RN);
    eigenVector3d2CArray(omega_RN_N, attMsgBuffer.omega_RN_N);
    eigenVector3d2CArray(omegaDot_RN_N, attMsgBuffer.domega_RN_N);

    /*! Write the output messages */
    this->attRefOutMsg.write(&attMsgBuffer, this->moduleID, currentSimNanos);
}

double FlybyPoint::getTimeBetweenFilterData() const {
    return this->timeBetweenFilterData;
}

void FlybyPoint::setTimeBetweenFilterData(double time) {
    this->timeBetweenFilterData = time;
}

double FlybyPoint::getToleranceForCollinearity() const {
    return this->toleranceForCollinearity;
}

void FlybyPoint::setToleranceForCollinearity(double tolerance) {
    this->toleranceForCollinearity = tolerance;
}

/*! Get the sign (+1 or -1) of the axis of rotation of the Z axis during the flyby
 @param int sign (+1 or -1)
 */
int FlybyPoint::getSignOfOrbitNormalFrameVector() const {
    return this->signOfOrbitNormalFrameVector;
}

/*! Set the sign (+1 or -1) of the axis of rotation of the Z axis during the flyby
 @param int sign (+1 or -1)
 */
void FlybyPoint::setSignOfOrbitNormalFrameVector(int sign) {
    this->signOfOrbitNormalFrameVector = sign;
}

/*! Get the maximum acceleration threshold to consider a solution invalid
 @return double maximum accceleration
 */
double FlybyPoint::getMaximumAccelerationThreshold() const {
    return this->maxAcceleration;
}

/*! Set the maximum acceleration threshold to consider a solution invalid
 @param double maximum accceleration
 */
void FlybyPoint::setMaximumAccelerationThreshold(double maxAccelerationThreshold) {
    this->maxAcceleration = maxAccelerationThreshold;
}

/*! Get the maximum rate threshold to consider a solution invalid
 @return maximum rate
 */
double FlybyPoint::getMaximumRateThreshold() const {
    return this->maxRate;
}

/*! Set the maximum rate threshold to consider a solution invalid
 @param maximum rate
 */
void FlybyPoint::setMaximumRateThreshold(double maxRateThreshold) {
    this->maxRate = maxRateThreshold;
}
