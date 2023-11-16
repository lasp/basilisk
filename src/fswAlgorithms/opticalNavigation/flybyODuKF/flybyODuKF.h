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


/*! @brief Top level structure for the flyby OD unscented kalman filter.
 Used to estimate the spacecraft's inertial position relative to a body.
 */

#ifndef FLYBYODUKF_H
#define FLYBYODUKF_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/orbitalMotion.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefCpp/OpNavSUKFMsgPayload.h"
#include "architecture/msgPayloadDefCpp/OpNavUnitVecMsgPayload.h"

#include <string.h>
#include <array>
#include <math.h>

class FlybyODuKF: public SysModel {
public:
    FlybyODuKF();
    ~FlybyODuKF() override;
    void Reset(uint64_t CurrentSimNanos) override;
    void UpdateState(uint64_t CurrentSimNanos) override;

private:
    void timeUpdate(const float updateTime);
    void measurementUpdate();
    void measurementModel();
    void readFilterMeasurements();
    void writeOutputMessages(uint64_t CurrentSimNanos);
    void computePostFitResiudals();
    Eigen::MatrixXf qrDecompositionJustR(const Eigen::MatrixXf input) const;
    Eigen::MatrixXf choleskyUpDownDate(const Eigen::MatrixXf input,
                                       const Eigen::VectorXf inputVector,
                                       const float coefficient) const;
    Eigen::MatrixXf choleskyDecomposition(const Eigen::MatrixXf input) const;
    Eigen::MatrixXf backSubstitution(const Eigen::MatrixXf U, const Eigen::MatrixXf b) const;
    Eigen::MatrixXf forwardSubstitution(const Eigen::MatrixXf L, const Eigen::MatrixXf b) const;

    Eigen::VectorXf rk4(const std::function<Eigen::VectorXf(float, Eigen::VectorXf)>& ODEfunction,
                        const Eigen::VectorXf& X0,
                        float t0,
                        float dt) const;
    Eigen::VectorXf propagate(std::array<float, 2> interval, const Eigen::VectorXf& X0, float dt) const;

public:
    ReadFunctor<OpNavUnitVecMsgPayload> opNavHeadingMsg;
    OpNavUnitVecMsgPayload opNavHeadingBuffer;
    Message<NavTransMsgPayload> navTransOutMsg;
    Message<OpNavSUKFMsgPayload> opNavFilterMsg;

    //!< Variables are named closely to the reference document :
    //!< "The Square-root unscented Kalman Filter for state and parameter-estimation" by van der Merwe and Wan
    float beta;
    float alpha;
    float kappa;
    float lambda;
    float eta;
    float muCentral;

    Eigen::MatrixXf processNoise; //!< [-] process noise matrix
    Eigen::MatrixXf measurementNoise; //!< [-] Measurement Noise
    Eigen::VectorXf stateInitial; //!< [-] State estimate for time TimeTag at previous time
    Eigen::MatrixXf sBarInitial; //!< [-] Time updated covariance at previous time
    Eigen::MatrixXf covarInitial; //!< [-] covariance at previous time

    float measNoiseScaling = 1; //!< [s] Scale factor that can be applied on the measurement noise to over/under weight

private:
    NavTransMsgPayload navTransOutMsgBuffer; //!< Message buffer for input translational nav message
    OpNavSUKFMsgPayload opNavFilterMsgBuffer;

    float dt; //!< [s] seconds since last data epoch
    float previousFilterTimeTag; //!< [s]  Time tag for statecovar/etc
    bool computePostFits; //!< [bool]  Presence of a valid measurement to process
    size_t numberSigmaPoints; //!< [s]  2n+1 sigma points for convenience
    Eigen::VectorXf wM;
    Eigen::VectorXf wC;

    Eigen::VectorXf state; //!< [-] State estimate for time TimeTag
    Eigen::MatrixXf sBar; //!< [-] Time updated covariance
    Eigen::MatrixXf covar; //!< [-] covariance
    Eigen::VectorXf xBar; //!< [-] Current mean state estimate
    Eigen::MatrixXf sigmaPoints; //!< [-]    sigma point matrix

    Eigen::VectorXf obs; //!< [-] Observation vector for frame
    Eigen::MatrixXf yMeas; //!< [-] Measurement model data
    Eigen::VectorXf postFits; //!< [-] PostFit residuals
    Eigen::MatrixXf cholProcessNoise; //!< [-] cholesky of Qnoise
    Eigen::MatrixXf cholMeasurementNoise; //!< [-] cholesky of Qnoise

    BSKLogger bskLogger; //!< -- BSK Logging
};

#endif
