// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _SUNLINE_UKF_H_
#define _SUNLINE_UKF_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
#include <architecture/msgPayloadDef/CSSConfigMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/SunlineFilterMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

#include <stdint.h>

/*! @brief Top level structure for the CSS-based unscented Kalman Filter.
 Used to estimate the sun state in the vehicle body frame. */
class SunlineUKF : public SysModel {
public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    Message<NavAttMsgPayload> navStateOutMsg;           /*!< The name of the output message*/
    Message<SunlineFilterMsgPayload> filtDataOutMsg;    /*!< The name of the output filter data message*/
    ReadFunctor<CSSArraySensorMsgPayload> cssDataInMsg; /*!< The name of the Input message*/
    ReadFunctor<CSSConfigMsgPayload> cssConfigInMsg;    /*!< [-] The name of the CSS configuration message*/

    int numStates;    /*!< [-] Number of states for this filter*/
    int countHalfSPs; /*!< [-] Number of sigma points over 2 */
    int numObs;       /*!< [-] Number of measurements this cycle */
    double beta;      /*!< [-] Beta parameter for filter */
    double alpha;     /*!< [-] Alpha parameter for filter*/
    double kappa;     /*!< [-] Kappa parameter for filter*/
    double lambdaVal; /*!< [-] Lambda parameter for filter*/
    double gamma;     /*!< [-] Gamma parameter for filter*/
    double qObsVal;   /*!< [-] CSS instrument noise parameter*/

    double dt;      /*!< [s] seconds since last data epoch */
    double timeTag; /*!< [s]  Time tag for statecovar/etc */

    double wM[2 * SKF_N_STATES + 1]; /*!< [-] Weighting vector for sigma points*/
    double wC[2 * SKF_N_STATES + 1]; /*!< [-] Weighting vector for sigma points*/

    double state[SKF_N_STATES];                /*!< [-] State estimate for time TimeTag*/
    double sBar[SKF_N_STATES * SKF_N_STATES];  /*!< [-] Time updated covariance */
    double covar[SKF_N_STATES * SKF_N_STATES]; /*!< [-] covariance */
    double xBar[SKF_N_STATES];                 /*!< [-] Current mean state estimate*/

    double obs[MAX_NUM_CSS_SENSORS];                            /*!< [-] Observation vector for frame*/
    double yMeas[MAX_NUM_CSS_SENSORS * (2 * SKF_N_STATES + 1)]; /*!< [-] Measurement model data */
    double postFits[MAX_NUM_CSS_SENSORS];                       /*!< [-] PostFit residuals */

    double SP[(2 * SKF_N_STATES + 1) * SKF_N_STATES]; /*!< [-]    sigma point matrix */

    double qNoise[SKF_N_STATES * SKF_N_STATES];  /*!< [-] process noise matrix */
    double sQnoise[SKF_N_STATES * SKF_N_STATES]; /*!< [-] cholesky of Qnoise */

    double qObs[MAX_NUM_CSS_SENSORS * MAX_NUM_CSS_SENSORS]; /*!< [-] Maximally sized obs noise matrix*/

    double cssNHat_B[MAX_NUM_CSS_SENSORS * 3]; /*!< [-] CSS normal vectors converted over to body*/
    double CBias[MAX_NUM_CSS_SENSORS];         /*!< [-] CSS individual calibration coefficients */

    uint32_t numActiveCss;                      /*!< -- Number of currently active CSS sensors*/
    uint32_t numCSSTotal;                       /*!< [-] Count on the number of CSS we have on the spacecraft*/
    double sensorUseThresh;                     /*!< -- Threshold below which we discount sensors*/
    NavAttMsgPayload outputSunline;             /*!< -- Output sunline estimate data */
    CSSArraySensorMsgPayload cssSensorInBuffer; /*!< [-] CSS sensor data read in from message bus*/

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
