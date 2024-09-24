/*
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "fswAlgorithms/attDetermination/sunlineSuKF/sunlineSuKF.h"
#include "architecture/utilities/ukfUtilities.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"
#include <string.h>
#include <math.h>

/*! This method resets the sunline attitude filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SunlineSuKF::Reset(uint64_t callTime)
{

    CSSConfigMsgPayload cssConfigInBuffer;
    int32_t badUpdate;
    double tempMatrix[SKF_N_STATES_SWITCH*SKF_N_STATES_SWITCH];
    badUpdate = 0;

    /*! - Zero the local configuration data structures and outputs */
    mSetZero(this->cssNHat_B, MAX_NUM_CSS_SENSORS, 3);
    this->outputSunline = {};

    // check if the required input messages are included
    if (!this->cssConfigInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: sunlineSuKF.cssConfigInMsg wasn't connected.");
    }
    if (!this->cssDataInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: sunlineSuKF.cssDataInMsg wasn't connected.");
    }

    /*! - Read in mass properties and coarse sun sensor configuration information.*/
    cssConfigInBuffer = this->cssConfigInMsg();
    if (cssConfigInBuffer.nCSS > MAX_N_CSS_MEAS) {
        this->bskLogger.bskLog(BSK_ERROR, "sunlineSuKF.cssConfigInMsg.nCSS must not be greater than "
                                                  "MAX_N_CSS_MEAS value.");
    }

    /*! - For each coarse sun sensor, convert the configuration data over from structure to body*/
    for(uint32_t i=0; i<cssConfigInBuffer.nCSS; i++)
    {
        v3Copy(cssConfigInBuffer.cssVals[i].nHat_B, &(this->cssNHat_B[i*3]));
        this->CBias[i] = cssConfigInBuffer.cssVals[i].CBias;
    }
    /*! - Save the count of sun sensors for later use */
    this->numCSSTotal = cssConfigInBuffer.nCSS;

    /*! - Initialize filter parameters to max values */
    this->dt = 0.0;
    this->timeTag = callTime*NANO2SEC;
    this->numStates = SKF_N_STATES_SWITCH;
    this->countHalfSPs = SKF_N_STATES_SWITCH;
    this->numObs = MAX_N_CSS_MEAS;

    /*! Initalize the filter to use b_1 of the body frame to make frame*/
    v3Set(1, 0, 0, this->bVec_B);
    this->switchTresh = 0.866;

    /*! - Ensure that all internal filter matrices are zeroed*/
    vSetZero(this->obs, this->numObs);
    vSetZero(this->wM, this->countHalfSPs * 2 + 1);
    vSetZero(this->wC, this->countHalfSPs * 2 + 1);
    mSetZero(this->sBar, this->numStates, this->numStates);
    mSetZero(this->SP, this->countHalfSPs * 2 + 1,
             this->numStates);
    mSetZero(this->sQnoise, this->numStates, this->numStates);

    /*! - Set lambda/gamma to standard value for unscented kalman filters */
    this->lambdaVal = this->alpha*this->alpha*
    (this->numStates + this->kappa) - this->numStates;
    this->gamma = sqrt(this->numStates + this->lambdaVal);


    /*! - Set the wM/wC vectors to standard values for unscented kalman filters*/
    this->wM[0] = this->lambdaVal / (this->numStates +
                                                 this->lambdaVal);
    this->wC[0] = this->lambdaVal / (this->numStates +
                                                 this->lambdaVal) + (1 - this->alpha*this->alpha + this->beta);
    for (uint32_t i = 1; i<this->countHalfSPs * 2 + 1; i++)
    {
        this->wM[i] = 1.0 / 2.0*1.0 / (this->numStates +
                                             this->lambdaVal);
        this->wC[i] = this->wM[i];
    }

    /*! - User a cholesky decomposition to obtain the sBar and sQnoise matrices for use in
          filter at runtime*/
    mCopy(this->covarInit, this->numStates, this->numStates,
          this->covar);
    mSetZero(this->covarPrev, this->numStates, this->numStates);
    mSetZero(this->sBarPrev, this->numStates, this->numStates);
    vSetZero(this->statePrev, this->numStates);
    mCopy(this->covar, this->numStates, this->numStates,
          this->sBar);
    badUpdate += ukfCholDecomp(this->sBar, (int32_t) this->numStates,
                               (int32_t) this->numStates, tempMatrix);
    mCopy(tempMatrix, this->numStates, this->numStates,
          this->sBar);
    badUpdate += ukfCholDecomp(this->qNoise, (int32_t)this->numStates,
                               (int32_t) this->numStates, this->sQnoise);
    mTranspose(this->sQnoise, this->numStates,
               this->numStates, this->sQnoise);

    if (this->cssDataInMsg.isWritten()){
        this->cssSensorInBuffer = this->cssDataInMsg();
    } else {
        this->cssSensorInBuffer = {};
    }

    if (badUpdate <0){
        this->bskLogger.bskLog(BSK_WARNING, "Reset method contained bad update");
    }
}

/*! This method takes the parsed CSS sensor data and outputs an estimate of the
 sun vector in the ADCS body frame
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SunlineSuKF::UpdateState(uint64_t callTime)
{
    double newTimeTag;
    double yBar[MAX_N_CSS_MEAS];
    double tempYVec[MAX_N_CSS_MEAS];
    double sunheading_hat[3];
    double states_BN[SKF_N_STATES_SWITCH];
    uint64_t i;
    uint64_t timeOfMsgWritten;
    int isWritten;
    SunlineFilterMsgPayload sunlineDataOutBuffer;
    double maxSens;

    /*! - Read the input parsed CSS sensor data message*/
    this->cssSensorInBuffer = this->cssDataInMsg();
    timeOfMsgWritten = this->cssDataInMsg.timeWritten();
    isWritten = this->cssDataInMsg.isWritten();

    /* zero the output messages */
    this->outputSunline = {};
    sunlineDataOutBuffer = {};

    /*! If the filter is not initialized manually, give it an initial guess using the CSS with the strongest signal.*/
    if(this->filterInitialized==0)
    {
        vSetZero(this->stateInit, SKF_N_STATES_SWITCH);
        this->stateInit[5] = 1;
        this->stateInit[0] = 1;
        maxSens = 0.0;
        /*! Loop through sensors to find max*/
        for(i=0; i<this->numCSSTotal; i++)
        {
            if(this->cssSensorInBuffer.CosValue[i] > maxSens)
            {
                v3Copy(&(this->cssNHat_B[i*3]), this->stateInit);
                maxSens = this->cssSensorInBuffer.CosValue[i];
                /*! Max sensor reading is initial guess for the kelly factor*/
                this->stateInit[5] = maxSens;
            }
        }
        if(maxSens < this->sensorUseThresh)
        {
            return;
        }
        /*! The normal of the max activated sensor is the initial state*/
        vCopy(this->stateInit, this->numStates, this->state);
        this->filterInitialized = 1;
    }

    v3Normalize(&this->state[0], sunheading_hat);

    /*! - Check for switching frames */
    if (fabs(v3Dot(this->bVec_B, sunheading_hat)) > this->switchTresh)
    {
        sunlineSuKFSwitch(this->bVec_B, this->state, this->covar);
    }

    /*! - If the time tag from the measured data is new compared to previous step,
          propagate and update the filter*/
    newTimeTag = timeOfMsgWritten * NANO2SEC;
    if(newTimeTag >= this->timeTag && isWritten)
    {
        sunlineSuKFTimeUpdate(this, newTimeTag);
        sunlineSuKFMeasUpdate(this, newTimeTag);
    }
    v3Normalize(this->state, this->state);
    /*! - If current clock time is further ahead than the measured time, then
          propagate to this current time-step*/
    newTimeTag = callTime*NANO2SEC;
    if(newTimeTag > this->timeTag)
    {
        sunlineSuKFTimeUpdate(this, newTimeTag);
    }

    /*! - Compute Post Fit Residuals, first get Y (eq 22) using the states post fit*/
    sunlineSuKFMeasModel(this);

    /*! - Compute the value for the yBar parameter (equation 23)*/
    vSetZero(yBar, this->numObs);
    for(i=0; i<this->countHalfSPs*2+1; i++)
    {
        vCopy(&(this->yMeas[i*this->numObs]), this->numObs,
              tempYVec);
        vScale(this->wM[i], tempYVec, this->numObs, tempYVec);
        vAdd(yBar, this->numObs, tempYVec, yBar);
    }

    /*! - The post fits are y- ybar*/
    mSubtract(this->obs, this->numObs, 1, yBar, this->postFits);

    /*! - Write the sunline estimate into the copy of the navigation message structure*/
	v3Copy(this->state, this->outputSunline.vehSunPntBdy);
    v3Normalize(this->outputSunline.vehSunPntBdy,
        this->outputSunline.vehSunPntBdy);
    this->outputSunline.timeTag = this->timeTag;
    this->navStateOutMsg.write(&this->outputSunline, this->moduleID, callTime);

    /*! - Switch the rates back to omega_BN instead of omega_SB */
    vCopy(this->state, SKF_N_STATES_SWITCH, states_BN);
    vScale(-1, &(states_BN[3]), 2, &(states_BN[3])); /*! The Filter currently outputs omega_SB = -omega_BN (check doc for details)*/

    /*! - Populate the filter states output buffer and write the output message*/
    sunlineDataOutBuffer.timeTag = this->timeTag;
    sunlineDataOutBuffer.numObs = (int) this->numObs;
    memmove(sunlineDataOutBuffer.covar, this->covar,
            SKF_N_STATES_SWITCH*SKF_N_STATES_SWITCH*sizeof(double));
    memmove(sunlineDataOutBuffer.state, states_BN, SKF_N_STATES_SWITCH*sizeof(double));
    memmove(sunlineDataOutBuffer.postFitRes, this->postFits, MAX_N_CSS_MEAS*sizeof(double));
    this->filtDataOutMsg.write(&sunlineDataOutBuffer, this->moduleID, callTime);

    return;
}

/*! This method propagates a sunline state vector forward in time.  Note
    that the calling parameter is updated in place to save on data copies.
	@return void
    @param stateInOut The state that is propagated
    @param b_Vec b vector
    @param dt time step (s)
*/
void sunlineStateProp(double *stateInOut, double *b_Vec, double dt)
{

    double propagatedVel[SKF_N_STATES_HALF];
    double omegaCrossd[SKF_N_STATES_HALF];
    double omega_BN_S[SKF_N_STATES_HALF] = {0, -stateInOut[3], -stateInOut[4]};
    double omega_BN_B[SKF_N_STATES_HALF];
    double dcm_BS[SKF_N_STATES_HALF][SKF_N_STATES_HALF];

    mSetZero(dcm_BS, SKF_N_STATES_HALF, SKF_N_STATES_HALF);

    sunlineSuKFComputeDCM_BS(stateInOut, b_Vec, &dcm_BS[0][0]);
    mMultV(dcm_BS, SKF_N_STATES_HALF, SKF_N_STATES_HALF, omega_BN_S, omega_BN_B);
    /* Set local variables to zero*/
    vSetZero(propagatedVel, SKF_N_STATES_HALF);

    /*! Take omega cross d*/
    v3Cross(omega_BN_B, stateInOut, omegaCrossd);

    /*! - Multiply omega cross d by dt and add to state to propagate */
    v3Scale(-dt, omegaCrossd, propagatedVel);
    v3Add(stateInOut, propagatedVel, stateInOut);
    v3Normalize(stateInOut, stateInOut);

	return;
}

/*! This method performs the time update for the sunline kalman filter.
     It propagates the sigma points forward in time and then gets the current
	 covariance and state estimates.
	 @return void
     @param configData The configuration data associated with the CSS estimator
     @param updateTime The time that we need to fix the filter to (seconds)
*/
int sunlineSuKFTimeUpdate(SunlineSuKF *data, double updateTime)
{
    int Index, badUpdate;
	double sBarT[SKF_N_STATES_SWITCH*SKF_N_STATES_SWITCH];
	double xComp[SKF_N_STATES_SWITCH], AT[(2 * SKF_N_STATES_SWITCH + SKF_N_STATES_SWITCH)*SKF_N_STATES_SWITCH];
	double aRow[SKF_N_STATES_SWITCH], rAT[SKF_N_STATES_SWITCH*SKF_N_STATES_SWITCH], xErr[SKF_N_STATES_SWITCH];
	double sBarUp[SKF_N_STATES_SWITCH*SKF_N_STATES_SWITCH];
	double *spPtr;
    double procNoise[SKF_N_STATES_SWITCH*SKF_N_STATES_SWITCH];
    badUpdate = 0;

    vCopy(data->state, data->numStates, data->statePrev);
    mCopy(data->sBar, data->numStates, data->numStates, data->sBarPrev);
    mCopy(data->covar, data->numStates, data->numStates, data->covarPrev);
    data->dt = updateTime - data->timeTag;
    mCopy(data->sQnoise, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, procNoise);
    /*! - Copy over the current state estimate into the 0th Sigma point and propagate by dt*/
	vCopy(data->state, data->numStates,
		&(data->SP[0 * data->numStates + 0]));
	mSetZero(rAT, data->countHalfSPs, data->countHalfSPs);
	sunlineStateProp(&(data->SP[0]), data->bVec_B, data->dt);
    /*! - Scale that Sigma point by the appopriate scaling factor (Wm[0])*/
	vScale(data->wM[0], &(data->SP[0]),
        data->numStates, data->xBar);
    /*! - Get the transpose of the sBar matrix because it is easier to extract Rows vs columns*/
    mTranspose(data->sBar, data->numStates, data->numStates,
               sBarT);
    /*! - For each Sigma point, apply sBar-based error, propagate forward, and scale by Wm just like 0th.
          Note that we perform +/- sigma points simultaneously in loop to save loop values.*/
	for (uint64_t i = 0; i<data->countHalfSPs; i++)
	{
		Index = (int) i + 1;
		spPtr = &(data->SP[Index*(int)data->numStates]);
		vCopy(&sBarT[i*(int)data->numStates], data->numStates, spPtr);
		vScale(data->gamma, spPtr, data->numStates, spPtr);
		vAdd(spPtr, data->numStates, data->state, spPtr);
		sunlineStateProp(spPtr, data->bVec_B, data->dt);
		vScale(data->wM[Index], spPtr, data->numStates, xComp);
		vAdd(xComp, data->numStates, data->xBar, data->xBar);

		Index = (int) i + 1 + (int) data->countHalfSPs;
        spPtr = &(data->SP[Index*(int)data->numStates]);
        vCopy(&sBarT[i*(int) data->numStates], data->numStates, spPtr);
        vScale(-data->gamma, spPtr, data->numStates, spPtr);
        vAdd(spPtr, data->numStates, data->state, spPtr);
        sunlineStateProp(spPtr, data->bVec_B, data->dt);
        vScale(data->wM[Index], spPtr, data->numStates, xComp);
        vAdd(xComp, data->numStates, data->xBar, data->xBar);
	}
    /*! - Zero the AT matrix prior to assembly*/
    mSetZero(AT, (2 * data->countHalfSPs + data->numStates),
        data->countHalfSPs);
	/*! - Assemble the AT matrix.  Note that data matrix is the internals of
          the qr decomposition call in the source design documentation.  It is
          the inside of equation 20 in that document*/
	for (uint64_t i = 0; i<2 * data->countHalfSPs; i++)
	{

        vScale(-1.0, data->xBar, data->numStates, aRow);
        vAdd(aRow, data->numStates,
             &(data->SP[(i+1)*(int) data->numStates]), aRow);
        if (data->wC[i+1] <0){return -1;}
        vScale(sqrt(data->wC[i+1]), aRow, data->numStates, aRow);
		memcpy((void *)&AT[i*(int) data->numStates], (void *)aRow,
			data->numStates*sizeof(double));
	}

    /*! - Pop the sQNoise matrix on to the end of AT prior to getting QR decomposition*/
	memcpy(&AT[2 * data->countHalfSPs*data->numStates],
		procNoise, data->numStates*data->numStates
        *sizeof(double));
    /*! - QR decomposition (only R computed!) of the AT matrix provides the new sBar matrix*/
    ukfQRDJustR(AT, (int32_t) (2 * data->countHalfSPs + data->numStates),
                (int32_t) data->countHalfSPs, rAT);
    mCopy(rAT, data->numStates, data->numStates, sBarT);
    mTranspose(sBarT, data->numStates, data->numStates,
        data->sBar);

    /*! - Shift the sBar matrix over by the xBar vector using the appropriate weight
          like in equation 21 in design document.*/
    vScale(-1.0, data->xBar, data->numStates, xErr);
    vAdd(xErr, data->numStates, &data->SP[0], xErr);
    badUpdate += ukfCholDownDate(data->sBar, xErr, data->wC[0],
                                 (int32_t) data->numStates, sBarUp);

    /*! - Save current sBar matrix, covariance, and state estimate off for further use*/
    mCopy(sBarUp, data->numStates, data->numStates, data->sBar);
    mTranspose(data->sBar, data->numStates, data->numStates,
        data->covar);
	mMultM(data->sBar, data->numStates, data->numStates,
        data->covar, data->numStates, data->numStates,
           data->covar);
    vCopy(&(data->SP[0]), data->numStates, data->state );

	data->timeTag = updateTime;

    if (badUpdate<0){
        sunlineSuKFCleanUpdate(data);
        return -1;
    }
    return 0;
}

/*! This method computes what the expected measurement vector is for each CSS
    that is present on the spacecraft.  All data is transacted from the main
    data structure for the model because there are many variables that would
    have to be updated otherwise.
 @return void
 @param configData The configuration data associated with the CSS estimator

 */
void sunlineSuKFMeasModel(SunlineSuKF *data)
{
    uint32_t i, j, obsCounter;
    double sensorNormal[3];
    double normalizedState[3];
    double expectedMeas;
    double kellDelta;

    obsCounter = 0;
    /*! - Loop over all available coarse sun sensors and only use ones that meet validity threshold*/
    for(i=0; i<data->numCSSTotal; i++)
    {
        v3Scale(data->CBias[i], &(data->cssNHat_B[i*3]), sensorNormal);
        v3Normalize(data->state, normalizedState);
        expectedMeas = v3Dot(normalizedState, sensorNormal);
        expectedMeas = expectedMeas > 0.0 ? expectedMeas : 0.0;
        kellDelta = 1.0;
        /*! - Scale the measurement by the kelly factor.*/
        if(data->kellFits[i].cssKellFact > 0.0)
        {
            kellDelta -= exp(-pow(expectedMeas,data->kellFits[i].cssKellPow) /
                             data->kellFits[i].cssKellFact);
            expectedMeas *= kellDelta;
            expectedMeas *= data->kellFits[i].cssRelScale;
        }
        expectedMeas *= data->state[5];
        expectedMeas = expectedMeas > 0.0 ? expectedMeas : 0.0;
        if(data->cssSensorInBuffer.CosValue[i] > data->sensorUseThresh ||
           expectedMeas > data->sensorUseThresh)
        {
            /*! - For each valid measurement, copy observation value and compute expected obs value
                  on a per sigma-point basis.*/
            data->obs[obsCounter] = data->cssSensorInBuffer.CosValue[i];
            for(j=0; j<data->countHalfSPs*2+1; j++)
            {
                v3Normalize(&(data->SP[j*SKF_N_STATES_SWITCH]), normalizedState);
                expectedMeas = v3Dot(normalizedState, sensorNormal);
                expectedMeas = expectedMeas > 0.0 ? expectedMeas : 0.0;
                kellDelta = 1.0;
                /*! - Scale the measurement by the kelly factor.*/
                if(data->kellFits[i].cssKellFact > 0.0)
                {
                    kellDelta -= exp(-pow(expectedMeas,data->kellFits[i].cssKellPow) /
                                     data->kellFits[i].cssKellFact);
                    expectedMeas *= kellDelta;
                    expectedMeas *= data->kellFits[i].cssRelScale;
                }
                expectedMeas *= data->SP[j*SKF_N_STATES_SWITCH+5];
                expectedMeas = expectedMeas > 0.0 ? expectedMeas : 0.0;
                data->yMeas[obsCounter*(data->countHalfSPs*2+1) + j] =
                    expectedMeas;
            }
            obsCounter++;
        }
    }
    /*! - yMeas matrix was set backwards deliberately so we need to transpose it through*/
    mTranspose(data->yMeas, obsCounter, data->countHalfSPs*2+1, data->yMeas);
    data->numObs = obsCounter;

}

/*! This method performs the measurement update for the sunline kalman filter.
 It applies the observations in the obs vectors to the current state estimate and
 updates the state/covariance with that information.
 @return void
 @param configData The configuration data associated with the CSS estimator
 @param updateTime The time that we need to fix the filter to (seconds)
 */
int sunlineSuKFMeasUpdate(SunlineSuKF *data, double updateTime)
{
    uint32_t i;
    int32_t badUpdate;
    double yBar[MAX_N_CSS_MEAS], syInv[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS];
    double kMat[SKF_N_STATES_SWITCH*MAX_N_CSS_MEAS];
    double xHat[SKF_N_STATES_SWITCH], sBarT[SKF_N_STATES_SWITCH*SKF_N_STATES_SWITCH], tempYVec[MAX_N_CSS_MEAS];
    double AT[(2 * SKF_N_STATES_SWITCH + MAX_N_CSS_MEAS)*MAX_N_CSS_MEAS], qChol[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS];
    double rAT[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS], syT[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS];
    double sy[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS], Ucol[SKF_N_STATES_SWITCH];
    double updMat[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS], pXY[SKF_N_STATES_SWITCH*MAX_N_CSS_MEAS], Umat[SKF_N_STATES_SWITCH*MAX_N_CSS_MEAS];
    badUpdate = 0;

    vCopy(data->state, data->numStates, data->statePrev);
    mCopy(data->sBar, data->numStates, data->numStates, data->sBarPrev);
    mCopy(data->covar, data->numStates, data->numStates, data->covarPrev);

    /*! - Compute the valid observations and the measurement model for all observations*/
    sunlineSuKFMeasModel(data);

    /*! - Compute the value for the yBar parameter (note that data is equation 23 in the
          time update section of the reference document*/
    vSetZero(yBar, data->numObs);
    for(i=0; i<data->countHalfSPs*2+1; i++)
    {
        vCopy(&(data->yMeas[i*data->numObs]), data->numObs,
              tempYVec);
        vScale(data->wM[i], tempYVec, data->numObs, tempYVec);
        vAdd(yBar, data->numObs, tempYVec, yBar);
    }

    /*! - Populate the matrix that we perform the QR decomposition on in the measurement
          update section of the code.  This is based on the differenence between the yBar
          parameter and the calculated measurement models.  Equation 24 in driving doc. */
    mSetZero(AT, data->countHalfSPs*2+data->numObs,
        data->numObs);
    for(i=0; i<data->countHalfSPs*2; i++)
    {
        vScale(-1.0, yBar, data->numObs, tempYVec);
        vAdd(tempYVec, data->numObs,
             &(data->yMeas[(i+1)*data->numObs]), tempYVec);
        if (data->wC[i+1] <0){return -1;}
        vScale(sqrt(data->wC[i+1]), tempYVec, data->numObs, tempYVec);
        memcpy(&(AT[i*data->numObs]), tempYVec,
               data->numObs*sizeof(double));
    }

    /*! - This is the square-root of the Rk matrix which we treat as the Cholesky
        decomposition of the observation variance matrix constructed for our number
        of observations*/
    mSetZero(data->qObs, data->numCSSTotal, data->numCSSTotal);
    mSetIdentity(data->qObs, data->numObs, data->numObs);
    mScale(data->qObsVal, data->qObs, data->numObs,
           data->numObs, data->qObs);
    ukfCholDecomp(data->qObs, (int32_t) data->numObs, (int32_t) data->numObs, qChol);
    memcpy(&(AT[2*data->countHalfSPs*data->numObs]),
           qChol, data->numObs*data->numObs*sizeof(double));
    /*! - Perform QR decomposition (only R again) of the above matrix to obtain the
          current Sy matrix*/
    ukfQRDJustR(AT, (int32_t) (2*data->countHalfSPs+data->numObs),
                (int32_t) data->numObs, rAT);
    mCopy(rAT, data->numObs, data->numObs, syT);
    mTranspose(syT, data->numObs, data->numObs, sy);
    /*! - Shift the matrix over by the difference between the 0th SP-based measurement
          model and the yBar matrix (cholesky down-date again)*/
    vScale(-1.0, yBar, data->numObs, tempYVec);
    vAdd(tempYVec, data->numObs, &(data->yMeas[0]), tempYVec);
    badUpdate += ukfCholDownDate(sy, tempYVec, data->wC[0],
                                 (int32_t) data->numObs, updMat);
    /*! - Shifted matrix represents the Sy matrix */
    mCopy(updMat, data->numObs, data->numObs, sy);
    mTranspose(sy, data->numObs, data->numObs, syT);

    /*! - Construct the Pxy matrix (equation 26) which multiplies the Sigma-point cloud
          by the measurement model cloud (weighted) to get the total Pxy matrix*/
    mSetZero(pXY, data->numStates, data->numObs);
    for(i=0; i<2*data->countHalfSPs+1; i++)
    {
        vScale(-1.0, yBar, data->numObs, tempYVec);
        vAdd(tempYVec, data->numObs,
             &(data->yMeas[i*data->numObs]), tempYVec);
        vSubtract(&(data->SP[i*data->numStates]), data->numStates,
                  data->xBar, xHat);
        vScale(data->wC[i], xHat, data->numStates, xHat);
        mMultM(xHat, data->numStates, 1, tempYVec, 1, data->numObs,
            kMat);
        mAdd(pXY, data->numStates, data->numObs, kMat, pXY);
    }

    /*! - Then we need to invert the SyT*Sy matrix to get the Kalman gain factor.  Since
          The Sy matrix is lower triangular, we can do a back-sub inversion instead of
          a full matrix inversion.  That is the ukfUInv and ukfLInv calls below.  Once that
          multiplication is done (equation 27), we have the Kalman Gain.*/
    badUpdate += ukfUInv(syT, (int32_t) data->numObs, (int32_t) data->numObs, syInv);

    mMultM(pXY, data->numStates, data->numObs, syInv,
           data->numObs, data->numObs, kMat);
    badUpdate += ukfLInv(sy, (int32_t) data->numObs, (int32_t) data->numObs, syInv);
    mMultM(kMat, data->numStates, data->numObs, syInv,
           data->numObs, data->numObs, kMat);


    /*! - Difference the yBar and the observations to get the observed error and
          multiply by the Kalman Gain to get the state update.  Add the state update
          to the state to get the updated state value (equation 27).*/
    vSubtract(data->obs, data->numObs, yBar, tempYVec);
    mMultM(kMat, data->numStates, data->numObs, tempYVec,
        data->numObs, 1, xHat);
    vAdd(data->state, data->numStates, xHat, data->state);
    /*! - Compute the updated matrix U from equation 28.  Note that I then transpose it
         so that I can extract "columns" from adjacent memory*/
    mMultM(kMat, data->numStates, data->numObs, sy,
           data->numObs, data->numObs, Umat);
    mTranspose(Umat, data->numStates, data->numObs, Umat);
    /*! - For each column in the update matrix, perform a cholesky down-date on it to
     get the total shifted S matrix (called sBar in internal parameters*/
    for(i=0; i<data->numObs; i++)
    {
        vCopy(&(Umat[i*data->numStates]), data->numStates, Ucol);
        badUpdate += ukfCholDownDate(data->sBar, Ucol, -1.0, (int32_t) data->numStates, sBarT);
        mCopy(sBarT, data->numStates, data->numStates,
              data->sBar);
    }
    /*! - Compute equivalent covariance based on updated sBar matrix*/
    mTranspose(data->sBar, data->numStates, data->numStates,
               data->covar);
    mMultM(data->sBar, data->numStates, data->numStates,
           data->covar, data->numStates, data->numStates,
           data->covar);

    if (badUpdate<0){
        sunlineSuKFCleanUpdate(data);
        return -1;
    }
    return 0;
}


/*! This method computes the dcms necessary for the switch between the two frames.
 It the switches the states and the covariance, and sets s2 to be the new, different vector of the body frame.
 @return void
 @param bVec_B Pointer to b vector
 @param states Pointer to the states
 @param covar Pointer to the covariance
 */

void sunlineSuKFSwitch(double *bVec_B, double *states, double *covar)
{
    double dcm_BSold[SKF_N_STATES_HALF][SKF_N_STATES_HALF];
    double dcm_BSnew_T[SKF_N_STATES_HALF][SKF_N_STATES_HALF];
    double dcm_SnewSold[SKF_N_STATES_HALF][SKF_N_STATES_HALF];
    double switchMatP[SKF_N_STATES_SWITCH][SKF_N_STATES_SWITCH];
    double switchMat[SKF_N_STATES_SWITCH][SKF_N_STATES_SWITCH];

    double sun_heading_norm[SKF_N_STATES_HALF];
    double b1[SKF_N_STATES_HALF];
    double b2[SKF_N_STATES_HALF];

    /*!  Set the body frame vectors*/
    v3Set(1, 0, 0, b1);
    v3Set(0, 1, 0, b2);
    v3Normalize(&(states[0]), sun_heading_norm);

    /*! Populate the dcm_BS with the "old" S-frame*/
    sunlineSuKFComputeDCM_BS(sun_heading_norm, bVec_B, &dcm_BSold[0][0]);

    if (v3IsEqual(bVec_B, b1, 1e-10))
    {
        sunlineSuKFComputeDCM_BS(sun_heading_norm, b2, &dcm_BSnew_T[0][0]);
        v3Copy(b2, bVec_B);
    }
    else
    {
        sunlineSuKFComputeDCM_BS(sun_heading_norm, b1, &dcm_BSnew_T[0][0]);
        v3Copy(b1, bVec_B);
    }

    mTranspose(dcm_BSnew_T, SKF_N_STATES_HALF, SKF_N_STATES_HALF, dcm_BSnew_T);
    mMultM(dcm_BSnew_T, 3, 3, dcm_BSold, 3, 3, dcm_SnewSold);

    mSetIdentity(switchMat, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH);
    mSetSubMatrix(&dcm_SnewSold[1][1], 1, 2, &switchMat, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, 3, 3);
    mSetSubMatrix(&dcm_SnewSold[2][1], 1, 2, &switchMat, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, 4, 3);

    mMultV(switchMat, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, states, states);
    mMultM(switchMat, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, covar, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, switchMatP);
    mTranspose(switchMat, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, switchMat);
    mMultM(switchMatP, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, switchMat, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, covar);
    return;
}


void sunlineSuKFComputeDCM_BS(double sunheading[SKF_N_STATES_HALF], double bVec[SKF_N_STATES_HALF], double *dcm){
    double s1_B[SKF_N_STATES_HALF];
    double s2_B[SKF_N_STATES_HALF];
    double s3_B[SKF_N_STATES_HALF];

    mSetZero(dcm, SKF_N_STATES_HALF, SKF_N_STATES_HALF);
    v3SetZero(s2_B);
    v3SetZero(s3_B);

    v3Normalize(sunheading, s1_B);
    v3Cross(sunheading, bVec, s2_B);
    if (v3Norm(s2_B) < 1E-5){
        mSetIdentity(dcm, SKF_N_STATES_HALF, SKF_N_STATES_HALF);
    }
    else{
    v3Normalize(s2_B, s2_B);
    /*! Populate the dcm_BS with the "new" S-frame*/
    mSetSubMatrix(s1_B, 1, SKF_N_STATES_HALF, dcm, SKF_N_STATES_HALF, SKF_N_STATES_HALF, 0, 0);
    mSetSubMatrix(&(s2_B), 1, SKF_N_STATES_HALF, dcm, SKF_N_STATES_HALF, SKF_N_STATES_HALF, 1, 0);
    v3Cross(sunheading, s2_B, s3_B);
    v3Normalize(s3_B, s3_B);
    mSetSubMatrix(&(s3_B), 1, SKF_N_STATES_HALF, dcm, SKF_N_STATES_HALF, SKF_N_STATES_HALF, 2, 0);
    mTranspose(dcm, SKF_N_STATES_HALF, SKF_N_STATES_HALF, dcm);
    }

}

/*! This method cleans the filter states after a bad upadate on the fly.
 It removes the potentially corrupted previous estimates and puts the filter
 back to a working state.
 @return void
 @param configData The configuration data associated with the CSS estimator
 */
void sunlineSuKFCleanUpdate(SunlineSuKF *data){
    int i;
    /*! - Reset the observations, state, and covariannces to a previous safe value*/
    vSetZero(data->obs, data->numObs);
    vCopy(data->statePrev, data->numStates, data->state);
    mCopy(data->sBarPrev, data->numStates, data->numStates, data->sBar);
    mCopy(data->covarPrev, data->numStates, data->numStates, data->covar);

    /*! - Reset the wM/wC vectors to standard values for unscented kalman filters*/
    data->wM[0] = data->lambdaVal / (data->numStates +
                                                 data->lambdaVal);
    data->wC[0] = data->lambdaVal / (data->numStates +
                                                 data->lambdaVal) + (1 - data->alpha*data->alpha + data->beta);
    for (i = 1; i< ((int)data->countHalfSPs) * 2 + 1; i++)
    {
        data->wM[i] = 1.0 / 2.0*1.0 / (data->numStates +
                                             data->lambdaVal);
        data->wC[i] = data->wM[i];
    }

    return;
}
