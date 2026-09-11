// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _SUNLINE_EKF_H_
#define _SUNLINE_EKF_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
#include <architecture/msgPayloadDef/CSSConfigMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/SunlineFilterMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

#include <stdint.h>
#include <string.h>

/*!@brief Data structure for CSS Extended kalman filter estimator without gyros measurements.
 */

class OkeefeEKF : public SysModel {
public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void sunlineTimeUpdate(double updateTime);
    void sunlineMeasUpdate(double updateTime);

    Message<NavAttMsgPayload> navStateOutMsg;           /*!< The name of the output message*/
    Message<SunlineFilterMsgPayload> filtDataOutMsg;    /*!< The name of the output filter data message*/
    ReadFunctor<CSSArraySensorMsgPayload> cssDataInMsg; /*!< The name of the Input message*/
    ReadFunctor<CSSConfigMsgPayload> cssConfigInMsg;    /*!< [-] The name of the CSS configuration message*/

    double qObsVal;  /*!< [-] CSS instrument noise parameter*/
    double qProcVal; /*!< [-] Process noise parameter*/

    double dt;      /*!< [s] seconds since last data epoch */
    double timeTag; /*!< [s]  Time tag for statecovar/etc */

    double state[SKF_N_STATES_HALF];                               /*!< [-] State estimate for time TimeTag*/
    double prev_states[SKF_N_STATES_HALF];                         /*!< [-] State estimate for previous time TimeTag*/
    double omega[SKF_N_STATES_HALF];                               /*!< [-] Rotation rate vector*/
    double x[SKF_N_STATES_HALF];                                   /*!< [-] State errors */
    double xBar[SKF_N_STATES_HALF];                                /*!< [-] Current time updated mean state estimate*/
    double covarBar[SKF_N_STATES_HALF * SKF_N_STATES_HALF];        /*!< [-] Time updated covariance */
    double covar[SKF_N_STATES_HALF * SKF_N_STATES_HALF];           /*!< [-] covariance */
    double stateTransition[SKF_N_STATES_HALF * SKF_N_STATES_HALF]; /*!< [-] covariance */
    double kalmanGain[SKF_N_STATES_HALF * MAX_NUM_CSS_SENSORS];    /*!< Kalman Gain */

    double dynMat[SKF_N_STATES_HALF * SKF_N_STATES_HALF];    /*!< [-] Dynamics Matrix, A */
    double measMat[MAX_NUM_CSS_SENSORS * SKF_N_STATES_HALF]; /*!< [-] Measurement Matrix, H*/

    double obs[MAX_NUM_CSS_SENSORS];   /*!< [-] Observation vector for frame*/
    double yMeas[MAX_NUM_CSS_SENSORS]; /*!< [-] Linearized measurement model data */

    double procNoise[SKF_N_STATES_HALF * SKF_N_STATES_HALF];     /*!< [-] process noise matrix */
    double measNoise[MAX_NUM_CSS_SENSORS * MAX_NUM_CSS_SENSORS]; /*!< [-] Maximally sized obs noise matrix*/
    double postFits[MAX_NUM_CSS_SENSORS];                        /*!< [-] PostFit residuals */

    double cssNHat_B[MAX_NUM_CSS_SENSORS * 3]; /*!< [-] CSS normal vectors converted over to body*/
    double CBias[MAX_NUM_CSS_SENSORS];         /*!< [-] CSS individual calibration coefficients */

    uint32_t numStates;             /*!< [-] Number of states for this filter*/
    size_t numObs;                  /*!< [-] Number of measurements this cycle */
    uint32_t numActiveCss;          /*!< -- Number of currently active CSS sensors*/
    uint32_t numCSSTotal;           /*!< [-] Count on the number of CSS we have on the spacecraft*/
    double sensorUseThresh;         /*!< -- Threshold below which we discount sensors*/
    double eKFSwitch;               /*!< -- Max covariance element after which the filter switches to an EKF*/
    NavAttMsgPayload outputSunline; /*!< -- Output sunline estimate data */
    CSSArraySensorMsgPayload cssSensorInBuffer; /*!< [-] CSS sensor data read in from message bus*/

    BSKLogger bskLogger = {};  //!< BSK Logging
};

void sunlineStateSTMProp(
    double dynMat[SKF_N_STATES_HALF * SKF_N_STATES_HALF],
    double dt,
    double omega[SKF_N_STATES_HALF],
    double* stateInOut,
    double* prevstates,
    double* stateTransition
);

void sunlineHMatrixYMeas(
    double states[SKF_N_STATES_HALF],
    size_t numCSS,
    double cssSensorCos[MAX_NUM_CSS_SENSORS],
    double sensorUseThresh,
    double cssNHat_B[MAX_NUM_CSS_SENSORS * 3],
    double CBias[MAX_NUM_CSS_SENSORS],
    double* obs,
    double* yMeas,
    int* numObs,
    double* measMat
);

void sunlineKalmanGainOkeefe(
    double covarBar[SKF_N_STATES_HALF * SKF_N_STATES_HALF],
    double hObs[MAX_NUM_CSS_SENSORS * SKF_N_STATES_HALF],
    double qObsVal,
    int numObsInt,
    double* kalmanGain
);

void sunlineRateCompute(
    double states[SKF_N_STATES_HALF],
    double dt,
    double prev_states[SKF_N_STATES_HALF],
    double* omega
);

void sunlineDynMatrixOkeefe(double omega[SKF_N_STATES_HALF], double dt, double* dynMat);

void sunlineCKFUpdateOkeefe(
    double xBar[SKF_N_STATES_HALF],
    double kalmanGain[SKF_N_STATES_HALF * MAX_NUM_CSS_SENSORS],
    double covarBar[SKF_N_STATES_HALF * SKF_N_STATES_HALF],
    double qObsVal,
    int numObsInt,
    double yObs[MAX_NUM_CSS_SENSORS],
    double hObs[MAX_NUM_CSS_SENSORS * SKF_N_STATES_HALF],
    double* x,
    double* covar
);

void okeefeEKFUpdate(
    double kalmanGain[SKF_N_STATES_HALF * MAX_NUM_CSS_SENSORS],
    double covarBar[SKF_N_STATES_HALF * SKF_N_STATES_HALF],
    double qObsVal,
    int numObsInt,
    double yObs[MAX_NUM_CSS_SENSORS],
    double hObs[MAX_NUM_CSS_SENSORS * SKF_N_STATES_HALF],
    double* states,
    double* x,
    double* covar
);

#endif
