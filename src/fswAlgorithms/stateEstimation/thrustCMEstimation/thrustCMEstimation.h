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


/*! @brief Top level structure for the thrust CM estimation kalman filter.
 Used to estimate the spacecraft's center of mass position with respect to the B frame.
 */

#ifndef THRUSTCMESTIMATION_H
#define THRUSTCMESTIMATION_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/msgPayloadDefC/THRConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"

#include <string.h>
#include <array>
#include <math.h>

class ThrustCMEstimation: public SysModel {
public:
    ThrustCMEstimation();
    ~ThrustCMEstimation() override;
    void Reset(uint64_t CurrentSimNanos) override;
    void UpdateState(uint64_t CurrentSimNanos) override;

private:
    void timeUpdate(const double updateTime);
    void measurementUpdate();
    void measurementModel();
    void readFilterMeasurements();
    void writeOutputMessages(uint64_t CurrentSimNanos);
    void computePostFitResiudals();
    Eigen::MatrixXd qrDecompositionJustR(const Eigen::MatrixXd input) const;
    Eigen::MatrixXd choleskyUpDownDate(const Eigen::MatrixXd input,
                                       const Eigen::VectorXd inputVector,
                                       const double coefficient) const;
    Eigen::MatrixXd choleskyDecomposition(const Eigen::MatrixXd input) const;
    Eigen::MatrixXd backSubstitution(const Eigen::MatrixXd U, const Eigen::MatrixXd b) const;
    Eigen::MatrixXd forwardSubstitution(const Eigen::MatrixXd L, const Eigen::MatrixXd b) const;

    Eigen::VectorXd rk4(const std::function<Eigen::VectorXd(double, Eigen::VectorXd)>& ODEfunction,
                        const Eigen::VectorXd& X0,
                        double t0,
                        double dt) const;
    Eigen::VectorXd propagate(std::array<double, 2> interval, const Eigen::VectorXd& X0, double dt) const;

public:
    ReadFunctor<THRConfigMsgPayload>      thrusterConfigInMsg;
    ReadFunctor<CmdTorqueBodyMsgPayload>  cmdTorqueInMsg;
    ReadFunctor<AttGuidMsgPayload>        attGuidInMsg;
    Message<VehicleConfigMsgPayload>      vehConfigOutMsg;

    //!< Variables are named closely to the reference document :
    //!< "The Square-root unscented Kalman Filter for state and parameter-estimation" by van der Merwe and Wan
    double beta;
    double alpha;
    double kappa;
    double lambda;
    double eta;
    double muCentral;

    Eigen::MatrixXd processNoise; //!< [-] process noise matrix
    Eigen::MatrixXd measurementNoise; //!< [-] Measurement Noise
    Eigen::VectorXd stateInitial; //!< [-] State estimate for time TimeTag at previous time
    Eigen::MatrixXd sBarInitial; //!< [-] Time updated covariance at previous time
    Eigen::MatrixXd covarInitial; //!< [-] covariance at previous time

    double measNoiseScaling = 1; //!< [s] Scale factor that can be applied on the measurement noise to over/under weight

private:
    THRConfigMsgPayload      thrConfigMsgBuffer;  //!< Message buffer
    CmdTorqueBodyMsgPayload  cmdTorqueMsgBuffer;  //!< Message buffer
    AttGuidMsgPayload        attGuidMsgBuffer;    //!< Message buffer
    VehicleConfigMsgPayload  vehConfigMsgBuffer;  //!< Message buffer

    double dt; //!< [s] seconds since last data epoch
    double previousFilterTimeTag; //!< [s]  Time tag for statecovar/etc
    bool computePostFits; //!< [bool]  Presence of a valid measurement to process
    size_t numberSigmaPoints; //!< [s]  2n+1 sigma points for convenience
    Eigen::VectorXd wM;
    Eigen::VectorXd wC;

    Eigen::VectorXd state; //!< [-] State estimate for time TimeTag
    Eigen::MatrixXd sBar; //!< [-] Time updated covariance
    Eigen::MatrixXd covar; //!< [-] covariance
    Eigen::VectorXd xBar; //!< [-] Current mean state estimate
    Eigen::MatrixXd sigmaPoints; //!< [-]    sigma point matrix

    Eigen::VectorXd obs; //!< [-] Observation vector for frame
    Eigen::MatrixXd yMeas; //!< [-] Measurement model data
    Eigen::VectorXd postFits; //!< [-] PostFit residuals
    Eigen::MatrixXd cholProcessNoise; //!< [-] cholesky of Qnoise
    Eigen::MatrixXd cholMeasurementNoise; //!< [-] cholesky of Qnoise

    BSKLogger bskLogger; //!< -- BSK Logging
};

#endif
