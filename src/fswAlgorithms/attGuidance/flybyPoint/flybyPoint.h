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

#ifndef FLYBY_POINT_H
#define FLYBY_POINT_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"
#include <Eigen/Dense>
#include <math.h>

/*! @brief A class to perform flyby pointing */
class FlybyPoint: public SysModel {
public:
    void Reset(uint64_t CurrentSimNanos) override;
    void UpdateState(uint64_t CurrentSimNanos) override;

    std::tuple<Eigen::Vector3d, Eigen::Vector3d> readRelativeState();
    bool checkValidity(Eigen::Vector3d &r_BN_N, Eigen::Vector3d &v_BN_N) const;
    void computeFlybyParameters(Eigen::Vector3d &r_BN_N, Eigen::Vector3d &v_BN_N);
    void computeRN(Eigen::Vector3d &r_BN_N, Eigen::Vector3d &v_BN_N);
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> computeGuidanceSolution() const;
    void writeMessages(uint64_t currentSimNanos,
                       Eigen::Vector3d &sigma_RN,
                       Eigen::Vector3d &omega_RN_N,
                       Eigen::Vector3d &omegaDot_RN_N);

    double getTimeBetweenFilterData() const;
    void setTimeBetweenFilterData(double timeBetweenFilterData);
    double getToleranceForCollinearity() const;
    void setToleranceForCollinearity(double toleranceForCollinearity);
    int getSignOfOrbitNormalFrameVector() const;
    void setSignOfOrbitNormalFrameVector(int signOfOrbitNormalFrameVector);
    double getMaximumAccelerationThreshold() const;
    void setMaximumAccelerationThreshold(double maxAccelerationThreshold);
    double getMaximumRateThreshold() const;
    void setMaximumRateThreshold(double maxRateThreshold);

    ReadFunctor<NavTransMsgPayload>  filterInMsg;               //!< input msg relative position w.r.t. asteroid
    ReadFunctor<EphemerisMsgPayload> asteroidEphemerisInMsg;    //!< input asteroid ephemeris msg
    Message<AttRefMsgPayload> attRefOutMsg;                     //!< Attitude reference output message

private:
    double dt = 0; //!< current time step between last two updates
    double timeBetweenFilterData = 0;       //!< time between two subsequent reads of the filter information
    double toleranceForCollinearity = 0;            //!< tolerance for singular conditions when position and velocity are collinear
    int signOfOrbitNormalFrameVector = 1;  //!< Sign of orbit normal vector to complete reference frame

    double maxRate = 0;  //!< maximum rate spacecraft can control to, used for validity of solution
    double maxAcceleration = 0;  //!< maximum acceleration spacecraft can control to, used for validity of solution

    bool firstRead = true;           //!< variable to attest if this is the first read after a Reset
    double f0 = 0;                  //!< ratio between relative velocity and position norms at time of read [Hz]
    double gamma0 = 0;              //!< flight path angle of the spacecraft at time of read [rad]
    uint64_t lastFilterReadTime = 0;  //!< time of last filter read
    Eigen::Matrix3d R0N;           //!< inertial-to-reference DCM at time of read

};


#endif
