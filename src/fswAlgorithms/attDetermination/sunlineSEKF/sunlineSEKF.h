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

/*! @brief Top level structure for the CSS-based Switch Extended Kalman Filter.
 Used to estimate the sun state in the vehicle body frame. */
class SunlineSEKF : public SysModel {
public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    Message<NavAttMsgPayload> navStateOutMsg;           /*!< The name of the output message*/
    Message<SunlineFilterMsgPayload> filtDataOutMsg;    /*!< The name of the output filter data message*/
    ReadFunctor<CSSArraySensorMsgPayload> cssDataInMsg; /*!< The name of the Input message*/
    ReadFunctor<CSSConfigMsgPayload> cssConfigInMsg;    /*!< [-] The name of the CSS configuration message*/

    double qObsVal;  /*!< [-] CSS instrument noise parameter*/
    double qProcVal; /*!< [-] Process noise parameter*/

    double dt;      /*!< [s] seconds since last data epoch */
    double timeTag; /*!< [s]  Time tag for statecovar/etc */

    double bVec_B[SKF_N_STATES_HALF]; /*!< [-] current vector of the b frame used to make Switch frame */
    double switchTresh; /*!< [-]  Cosine of angle between singularity and S-frame. If close to 1, the threshold for
                           switching frames is lower. If closer to 0.5 singularity is more largely avoided but switching
                           is more frequent  */

    double state[EKF_N_STATES_SWITCH];                                 /*!< [-] State estimate for time TimeTag*/
    double x[EKF_N_STATES_SWITCH];                                     /*!< State errors */
    double xBar[EKF_N_STATES_SWITCH];                                  /*!< [-] Current mean state estimate*/
    double covarBar[EKF_N_STATES_SWITCH * EKF_N_STATES_SWITCH];        /*!< [-] Time updated covariance */
    double covar[EKF_N_STATES_SWITCH * EKF_N_STATES_SWITCH];           /*!< [-] covariance */
    double stateTransition[EKF_N_STATES_SWITCH * EKF_N_STATES_SWITCH]; /*!< [-] State transition Matrix */
    double kalmanGain[EKF_N_STATES_SWITCH * MAX_NUM_CSS_SENSORS];      /*!< Kalman Gain */

    double dynMat[EKF_N_STATES_SWITCH * EKF_N_STATES_SWITCH];  /*!< [-] Dynamics Matrix, A */
    double measMat[MAX_NUM_CSS_SENSORS * EKF_N_STATES_SWITCH]; /*!< [-] Measurement Matrix, H*/
    double W_BS[EKF_N_STATES_SWITCH * EKF_N_STATES_SWITCH]; /*!< [-] Switch Matrix to bring states and covariance to new
                                                               S-frame when switch occurs*/

    double obs[MAX_NUM_CSS_SENSORS];      /*!< [-] Observation vector for frame*/
    double yMeas[MAX_NUM_CSS_SENSORS];    /*!< [-] Linearized measurement model data */
    double postFits[MAX_NUM_CSS_SENSORS]; /*!< [-] PostFit residuals */

    double procNoise[(EKF_N_STATES_SWITCH - 3) * (EKF_N_STATES_SWITCH - 3)]; /*!< [-] process noise matrix */
    double measNoise[MAX_NUM_CSS_SENSORS * MAX_NUM_CSS_SENSORS];             /*!< [-] Maximally sized obs noise matrix*/

    double cssNHat_B[MAX_NUM_CSS_SENSORS * 3]; /*!< [-] CSS normal vectors converted over to body*/
    uint32_t numStates;                        /*!< [-] Number of states for this filter*/
    size_t numObs;                             /*!< [-] Number of measurements this cycle */
    uint32_t numActiveCss;                     /*!< -- Number of currently active CSS sensors*/
    uint32_t numCSSTotal;                      /*!< [-] Count on the number of CSS we have on the spacecraft*/
    double sensorUseThresh;                    /*!< -- Threshold below which we discount sensors*/
    double eKFSwitch;               /*!< -- Max covariance element after which the filter switches to an EKF*/
    NavAttMsgPayload outputSunline; /*!< -- Output sunline estimate data */
    CSSArraySensorMsgPayload cssSensorInBuffer; /*!< [-] CSS sensor data read in from message bus*/

    BSKLogger bskLogger = {};  //!< BSK Logging
};

void sunlineTimeUpdate(SunlineSEKF* data, double updateTime);
void sunlineMeasUpdate(SunlineSEKF* data, double updateTime);
void sunlineStateSTMProp(
    double dynMat[EKF_N_STATES_SWITCH * EKF_N_STATES_SWITCH],
    double bVec[SKF_N_STATES],
    double dt,
    double* stateInOut,
    double* stateTransition
);
void sunlineHMatrixYMeas(
    double states[EKF_N_STATES_SWITCH],
    size_t numCSS,
    double cssSensorCos[MAX_NUM_CSS_SENSORS],
    double sensorUseThresh,
    double cssNHat_B[MAX_NUM_CSS_SENSORS * 3],
    double* obs,
    double* yMeas,
    int* numObs,
    double* measMat
);
void sunlineKalmanGain(
    double covarBar[EKF_N_STATES_SWITCH * EKF_N_STATES_SWITCH],
    double hObs[MAX_NUM_CSS_SENSORS * EKF_N_STATES_SWITCH],
    double qObsVal,
    size_t numObs,
    double* kalmanGain
);
void sunlineDynMatrix(double states[EKF_N_STATES_SWITCH], double bVec[SKF_N_STATES], double dt, double* dynMat);
void sunlineCKFUpdate(
    double xBar[EKF_N_STATES_SWITCH],
    double kalmanGain[EKF_N_STATES_SWITCH * MAX_NUM_CSS_SENSORS],
    double covarBar[EKF_N_STATES_SWITCH * EKF_N_STATES_SWITCH],
    double qObsVal,
    size_t numObs,
    double yObs[MAX_NUM_CSS_SENSORS],
    double hObs[MAX_NUM_CSS_SENSORS * EKF_N_STATES_SWITCH],
    double* x,
    double* covar
);
void sunlineSEKFUpdate(
    double kalmanGain[EKF_N_STATES_SWITCH * MAX_NUM_CSS_SENSORS],
    double covarBar[EKF_N_STATES_SWITCH * EKF_N_STATES_SWITCH],
    double qObsVal,
    size_t numObs,
    double yObs[MAX_NUM_CSS_SENSORS],
    double hObs[MAX_NUM_CSS_SENSORS * EKF_N_STATES_SWITCH],
    double* states,
    double* x,
    double* covar
);
void sunlineSEKFSwitch(double* bVec_B, double* states, double* covar);
void sunlineSEKFComputeDCM_BS(double sunheading[SKF_N_STATES_HALF], double bVec[SKF_N_STATES_HALF], double* dcm);

#endif
