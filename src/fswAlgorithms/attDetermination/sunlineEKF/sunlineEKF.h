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

/*! @brief Top level structure for the CSS-based Extended Kalman Filter.
 Used to estimate the sun state in the vehicle body frame. */
class SunlineEKF : public SysModel {
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
    double timeTag; /*!< [s]  Time tag for state/covar */

    double state[SKF_N_STATES];                            /*!< [-] State estimate for time TimeTag*/
    double x[SKF_N_STATES];                                /*!< State errors */
    double xBar[SKF_N_STATES];                             /*!< [-] Current mean time updated state estimate*/
    double covarBar[SKF_N_STATES * SKF_N_STATES];          /*!< [-] Time updated covariance */
    double covar[SKF_N_STATES * SKF_N_STATES];             /*!< [-] covariance */
    double stateTransition[SKF_N_STATES * SKF_N_STATES];   /*!< [-] State Transtion Matrix */
    double kalmanGain[SKF_N_STATES * MAX_NUM_CSS_SENSORS]; /*!< Kalman Gain */

    double dynMat[SKF_N_STATES * SKF_N_STATES];         /*!< [-] Dynamics Matrix, A */
    double measMat[MAX_NUM_CSS_SENSORS * SKF_N_STATES]; /*!< [-] Measurement Matrix H*/

    double obs[MAX_NUM_CSS_SENSORS];   /*!< [-] Observation vector for frame*/
    double yMeas[MAX_NUM_CSS_SENSORS]; /*!< [-] Linearized measurement model data */

    double procNoise[SKF_N_STATES / 2 * SKF_N_STATES / 2];       /*!< [-] process noise matrix */
    double measNoise[MAX_NUM_CSS_SENSORS * MAX_NUM_CSS_SENSORS]; /*!< [-] Maximally sized obs noise matrix*/
    double postFits[MAX_NUM_CSS_SENSORS];                        /*!< [-] PostFit residuals */

    double cssNHat_B[MAX_NUM_CSS_SENSORS * 3]; /*!< [-] CSS normal vectors converted over to body*/
    double CBias[MAX_NUM_CSS_SENSORS];         /*!< [-] CSS individual calibration coefficients */

    size_t numStates;               /*!< [-] Number of states for this filter*/
    int numObs;                     /*!< [-] Number of measurements this cycle */
    size_t numActiveCss;            /*!< -- Number of currently active CSS sensors*/
    size_t numCSSTotal;             /*!< [-] Count on the number of CSS we have on the spacecraft*/
    double sensorUseThresh;         /*!< -- Threshold below which we discount sensors*/
    double eKFSwitch;               /*!< -- Max covariance element after which the filter switches to an EKF update*/
    NavAttMsgPayload outputSunline; /*!< -- Output sunline estimate data */
    CSSArraySensorMsgPayload cssSensorInBuffer; /*!< [-] CSS sensor data read in from message bus*/

    BSKLogger bskLogger = {};  //!< BSK Logging
};

void sunlineStateSTMProp(
    double dynMat[SKF_N_STATES * SKF_N_STATES],
    double dt,
    double* stateInOut,
    double* stateTransition
);
void sunlineHMatrixYMeas(
    double states[SKF_N_STATES],
    int numCSS,
    double cssSensorCos[MAX_NUM_CSS_SENSORS],
    double sensorUseThresh,
    double cssNHat_B[MAX_NUM_CSS_SENSORS * 3],
    double CBias[MAX_NUM_CSS_SENSORS],
    double* obs,
    double* yMeas,
    int* numObs,
    double* measMat
);
void sunlineKalmanGain(
    double covarBar[SKF_N_STATES * SKF_N_STATES],
    double hObs[MAX_NUM_CSS_SENSORS * SKF_N_STATES],
    double qObsVal,
    int numObs,
    double* kalmanGain
);
void sunlineDynMatrix(double stateInOut[SKF_N_STATES], double dt, double* dynMat);
void sunlineCKFUpdate(
    double xBar[SKF_N_STATES],
    double kalmanGain[SKF_N_STATES * MAX_NUM_CSS_SENSORS],
    double covarBar[SKF_N_STATES * SKF_N_STATES],
    double qObsVal,
    int numObs,
    double yObs[MAX_NUM_CSS_SENSORS],
    double hObs[MAX_NUM_CSS_SENSORS * SKF_N_STATES],
    double* x,
    double* covar
);
void sunlineEKFUpdate(
    double kalmanGain[SKF_N_STATES * MAX_NUM_CSS_SENSORS],
    double covarBar[SKF_N_STATES * SKF_N_STATES],
    double qObsVal,
    int numObs,
    double yObs[MAX_NUM_CSS_SENSORS],
    double hObs[MAX_NUM_CSS_SENSORS * SKF_N_STATES],
    double* states,
    double* x,
    double* covar
);
#endif
