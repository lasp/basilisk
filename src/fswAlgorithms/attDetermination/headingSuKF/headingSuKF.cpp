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

#include "fswAlgorithms/attDetermination/headingSuKF/headingSuKF.h"
#include "architecture/utilities/ukfUtilities.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"
#include <string.h>
#include <math.h>

/*! This method resets the heading attitude filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void HeadingSuKF::Reset(uint64_t callTime)
{
    
    int32_t i;
    double tempMatrix[HEAD_N_STATES_SWITCH*HEAD_N_STATES_SWITCH];

    // check if the required input message is included
    if (!this->opnavDataInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: headingSuKF.opnavDataInMsg wasn't connected.");
    }

    /*! - Check input message connections */
    if (this->cameraConfigInMsg.isLinked()){
        this->putInCameraFrame = 1;
    } else {
        this->putInCameraFrame = 0;
    }

    /*! - Zero the local configuration data structures and outputs */
    this->outputHeading = {};

    
    /*! - Initialize filter parameters to max values */
    this->timeTag = callTime*NANO2SEC;
    this->dt = 0.0;
    this->numStates = HEAD_N_STATES_SWITCH;
    this->countHalfSPs = HEAD_N_STATES_SWITCH;
    
    /*! Initalize the filter to use b_1 of the body frame to make frame*/
    v3Set(1, 0, 0, this->bVec_B);
    this->switchTresh = 0.866;
    
    /*! - Ensure that all internal filter matrices are zeroed*/
    vSetZero(this->obs, OPNAV_MEAS);
    vSetZero(this->wM, (size_t) this->countHalfSPs * 2 + 1);
    vSetZero(this->wC, (size_t) this->countHalfSPs * 2 + 1);
    mSetZero(this->sBar, (size_t) this->numStates, (size_t) this->numStates);
    mSetZero(this->SP, (size_t) this->countHalfSPs * 2 + 1,
             (size_t) this->numStates);
    mSetZero(this->sQnoise, (size_t) this->numStates, (size_t) this->numStates);
    
    /*! - Set lambda/gamma to standard value for unscented kalman filters */
    this->lambdaVal = this->alpha*this->alpha*
    (this->numStates + this->kappa) - this->numStates;
    this->gamma = sqrt(this->numStates + this->lambdaVal);
    
    
    /*! - Set the wM/wC vectors to standard values for unscented kalman filters*/
    this->wM[0] = this->lambdaVal / (this->numStates +
                                                 this->lambdaVal);
    this->wC[0] = this->lambdaVal / (this->numStates +
                                                 this->lambdaVal) + (1 - this->alpha*this->alpha + this->beta);
    for (i = 1; i<this->countHalfSPs * 2 + 1; i++)
    {
        this->wM[i] = 1.0 / 2.0*1.0 / (this->numStates +
                                             this->lambdaVal);
        this->wC[i] = this->wM[i];
    }
    
    vCopy(this->stateInit, (size_t) this->numStates, this->state);
    
    /*! - User a cholesky decomposition to obtain the sBar and sQnoise matrices for use in 
          filter at runtime*/
    mCopy(this->covarInit, (size_t) this->numStates, (size_t) this->numStates,
          this->covar);
    mCopy(this->covar, (size_t) this->numStates, (size_t) this->numStates,
          this->sBar);
    ukfCholDecomp(this->sBar, this->numStates,
                  this->numStates, tempMatrix);
    mCopy(tempMatrix, (size_t) this->numStates, (size_t) this->numStates,
          this->sBar);
    ukfCholDecomp(this->qNoise, this->numStates,
                  this->numStates, this->sQnoise);
    mTranspose(this->sQnoise, (size_t) this->numStates,
               (size_t) this->numStates, this->sQnoise);
    
    return;
}

/*! This method takes the parsed heading sensor data and outputs an estimate of the
 sun vector in the ADCS body frame
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void HeadingSuKF::UpdateState(uint64_t callTime)
{
    double newTimeTag;
    double yBar[OPNAV_MEAS];
    double tempYVec[OPNAV_MEAS];
    double heading_hat[3];
    double states_BN[HEAD_N_STATES_SWITCH];
    int i;
    uint64_t ClockTime;
    int isWritten;

    HeadingFilterMsgPayload headingDataOutBuffer = {};
    OpNavMsgPayload opnavOutputBuffer = {};
    CameraConfigMsgPayload cameraConfig = {};
    /*! - Read the input parsed heading sensor data message*/
    ClockTime = 0;

    v3SetZero(this->obs);
    v3SetZero(this->postFits);

    this->opnavInBuffer = this->opnavDataInMsg();
    ClockTime = this->opnavDataInMsg.timeWritten();
    isWritten = this->opnavDataInMsg.isWritten();
    if (this->putInCameraFrame == 1){
        cameraConfig = this->cameraConfigInMsg();
    }
    v3Normalize(&this->state[0], heading_hat);
    
    /*! - Check for switching frames */
    if (fabs(v3Dot(this->bVec_B, heading_hat)) > this->switchTresh)
    {
        headingSuKFSwitch(this->bVec_B, this->state, this->covar);
    }
    this->rNorm = v3Norm(this->opnavInBuffer.r_BN_B);
    if (this->rNorm<1){
        this->rNorm =1;
    }
    
    /*! - If the time tag from the measured data is new compared to previous step,
          propagate and update the filter*/
    newTimeTag = ClockTime * NANO2SEC;
    if(newTimeTag >= this->timeTag && isWritten && this->opnavInBuffer.valid ==1)
    {
        headingSuKFTimeUpdate(this, newTimeTag);
        headingSuKFMeasUpdate(this, newTimeTag);
    }
    
    /*! - If current clock time is further ahead than the measured time, then
          propagate to this current time-step*/
    newTimeTag = callTime*NANO2SEC;
    if(newTimeTag > this->timeTag)
    {
        headingSuKFTimeUpdate(this, newTimeTag);
    }
    
    /*! - Compute the value for the yBar parameter (equation 23)*/
    vSetZero(yBar, OPNAV_MEAS);
    for(i=0; i<this->countHalfSPs*2+1; i++)
    {
        vCopy(&(this->yMeas[i*OPNAV_MEAS]), OPNAV_MEAS,
              tempYVec);
        vScale(this->wM[i], tempYVec, OPNAV_MEAS, tempYVec);
        vAdd(yBar, OPNAV_MEAS, tempYVec, yBar);
    }
    
    /*! - The post fits are y - ybar if a measurement was read, if observations are zero,
     do not compute post fit residuals*/
    if(!v3IsZero(this->obs, 1E-10)){
        mSubtract(this->obs, OPNAV_MEAS, 1, yBar, this->postFits);}
    
    /* Switch the rates back to omega_BN instead of oemga_SB */
    vCopy(this->state, HEAD_N_STATES_SWITCH, states_BN);
    vScale(-1, &(states_BN[3]), 2, &(states_BN[3]));
    
    /*! - Populate the filter states output buffer and write the output message*/
    headingDataOutBuffer.timeTag = this->timeTag;
    mCopy(this->covar, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, headingDataOutBuffer.covar);
    vCopy(states_BN, HEAD_N_STATES_SWITCH, headingDataOutBuffer.state);
    v3Copy(this->postFits, headingDataOutBuffer.postFitRes);
    this->filtDataOutMsg.write(&headingDataOutBuffer, this->moduleID, callTime);
    
    /*! - Write the heading estimate into the copy of the OpNav message structure*/
    opnavOutputBuffer.timeTag = this->timeTag;
    m33Copy(RECAST3X3 this->covar, RECAST3X3 opnavOutputBuffer.covar_B);
    v3Copy(&states_BN[0], opnavOutputBuffer.r_BN_B);
    v3Normalize(opnavOutputBuffer.r_BN_B, opnavOutputBuffer.r_BN_B);
    v3Scale(-this->rNorm, opnavOutputBuffer.r_BN_B, opnavOutputBuffer.r_BN_B);
    if (this->putInCameraFrame ==1){
        double dcm_CB[3][3], tempCovar[3][3];
        MRP2C(cameraConfig.sigma_CB, dcm_CB);
        m33tMultV3(dcm_CB, opnavOutputBuffer.r_BN_B, opnavOutputBuffer.r_BN_C);
        m33MultM33(dcm_CB, RECAST3X3 this->covar, tempCovar);
        m33MultM33t(tempCovar, dcm_CB, RECAST3X3 opnavOutputBuffer.covar_C);
    }
    opnavOutputBuffer.valid = this->opnavInBuffer.valid;
    opnavOutputBuffer.timeTag = this->opnavInBuffer.timeTag;
    this->opnavDataOutMsg.write(&opnavOutputBuffer, this->moduleID, callTime);
    
    return;
}

/*! This method propagates a heading state vector forward in time.  Note
    that the calling parameter is updated in place to save on data copies.
	@return void
    @param stateInOut The state that is propagated
    @param b_Vec pointer to b vector
    @param dt time step
*/
void headingStateProp(double *stateInOut, double *b_Vec, double dt)
{

    double propagatedVel[HEAD_N_STATES];
    double omegaCrossd[HEAD_N_STATES];
    double omega_BN_S[HEAD_N_STATES] = {0, -stateInOut[3], -stateInOut[4]};
    double omega_BN_B[HEAD_N_STATES];
    double dcm_BS[HEAD_N_STATES][HEAD_N_STATES];

    mSetZero(dcm_BS, HEAD_N_STATES, HEAD_N_STATES);

    headingSuKFComputeDCM_BS(stateInOut, b_Vec, &dcm_BS[0][0]);
    mMultV(dcm_BS, HEAD_N_STATES, HEAD_N_STATES, omega_BN_S, omega_BN_B);
    /* Set local variables to zero*/
    vSetZero(propagatedVel, HEAD_N_STATES);
    
    /*! Begin state update steps */
    /*! Take omega cross d*/
    v3Cross(omega_BN_B, stateInOut, omegaCrossd);
    
    /*! - Multiply omega cross d by dt and add to state to propagate */
    v3Scale(-dt, omegaCrossd, propagatedVel);
    v3Add(stateInOut, propagatedVel, stateInOut);
    
	return;
}

/*! This method performs the time update for the heading kalman filter.
     It propagates the sigma points forward in time and then gets the current 
	 covariance and state estimates.
	 @return void
     @param configData The configuration data associated with the heading estimator
     @param updateTime The time that we need to fix the filter to (seconds)
*/
void headingSuKFTimeUpdate(HeadingSuKF *configData, double updateTime)
{
	int i, Index;
	double sBarT[HEAD_N_STATES_SWITCH*HEAD_N_STATES_SWITCH];
	double xComp[HEAD_N_STATES_SWITCH], AT[(2 * HEAD_N_STATES_SWITCH + HEAD_N_STATES_SWITCH)*HEAD_N_STATES_SWITCH];
	double aRow[HEAD_N_STATES_SWITCH], rAT[HEAD_N_STATES_SWITCH*HEAD_N_STATES_SWITCH], xErr[HEAD_N_STATES_SWITCH];
	double sBarUp[HEAD_N_STATES_SWITCH*HEAD_N_STATES_SWITCH];
	double *spPtr;

    configData->dt = updateTime - configData->timeTag;
    
    /*! - Copy over the current state estimate into the 0th Sigma point and propagate by dt*/
	vCopy(configData->state, (size_t) configData->numStates,
		&(configData->SP[0 * configData->numStates + 0]));
	headingStateProp(&(configData->SP[0 * configData->numStates + 0]), configData->bVec_B, configData->dt);
    /*! - Scale that Sigma point by the appopriate scaling factor (Wm[0])*/
	vScale(configData->wM[0], &(configData->SP[0 * configData->numStates + 0]),
        (size_t) configData->numStates, configData->xBar);
    /*! - Get the transpose of the sBar matrix because it is easier to extract Rows vs columns*/
    mTranspose(configData->sBar, (size_t) configData->numStates, (size_t) configData->numStates,
               sBarT);
    /*! - For each Sigma point, apply sBar-based error, propagate forward, and scale by Wm just like 0th.
          Note that we perform +/- sigma points simultaneously in loop to save loop values.*/
	for (i = 0; i<configData->countHalfSPs; i++)
	{
		Index = i + 1;
		spPtr = &(configData->SP[Index*configData->numStates]);
		vCopy(&sBarT[i*configData->numStates], (size_t) configData->numStates, spPtr);
		vScale(configData->gamma, spPtr, (size_t) configData->numStates, spPtr);
		vAdd(spPtr, (size_t) configData->numStates, configData->state, spPtr);
		headingStateProp(spPtr, configData->bVec_B, configData->dt);
		vScale(configData->wM[Index], spPtr, (size_t) configData->numStates, xComp);
		vAdd(xComp, (size_t) configData->numStates, configData->xBar, configData->xBar);
		
		Index = i + 1 + configData->countHalfSPs;
        spPtr = &(configData->SP[Index*configData->numStates]);
        vCopy(&sBarT[i*configData->numStates], (size_t) configData->numStates, spPtr);
        vScale(-configData->gamma, spPtr, (size_t) configData->numStates, spPtr);
        vAdd(spPtr, (size_t) configData->numStates, configData->state, spPtr);
        headingStateProp(spPtr, configData->bVec_B, configData->dt);
        vScale(configData->wM[Index], spPtr, (size_t) configData->numStates, xComp);
        vAdd(xComp, (size_t) configData->numStates, configData->xBar, configData->xBar);
	}
    /*! - Zero the AT matrix prior to assembly*/
    mSetZero(AT, (2 * (size_t) configData->countHalfSPs + (size_t) configData->numStates),
        (size_t) configData->countHalfSPs);
	/*! - Assemble the AT matrix.  Note that this matrix is the internals of 
          the qr decomposition call in the source design documentation.  It is 
          the inside of equation 20 in that document*/
	for (i = 0; i<2 * configData->countHalfSPs; i++)
	{
		
        vScale(-1.0, configData->xBar, (size_t) configData->numStates, aRow);
        vAdd(aRow, (size_t) configData->numStates,
             &(configData->SP[(i+1)*configData->numStates]), aRow);
        vScale(sqrt(configData->wC[i+1]), aRow, (size_t) configData->numStates, aRow);
		memcpy((void *)&AT[i*configData->numStates], (void *)aRow,
			(size_t) configData->numStates*sizeof(double));
	}
    /*! - Pop the sQNoise matrix on to the end of AT prior to getting QR decomposition*/
	memcpy(&AT[2 * configData->countHalfSPs*configData->numStates],
		configData->sQnoise, (size_t) configData->numStates* (size_t) configData->numStates
        *sizeof(double));
    /*! - QR decomposition (only R computed!) of the AT matrix provides the new sBar matrix*/
    ukfQRDJustR(AT, 2 * configData->countHalfSPs + configData->numStates,
                configData->countHalfSPs, rAT);
    mCopy(rAT, (size_t) configData->numStates, (size_t) configData->numStates, sBarT);
    mTranspose(sBarT, (size_t) configData->numStates, (size_t) configData->numStates,
        configData->sBar);
    
    /*! - Shift the sBar matrix over by the xBar vector using the appropriate weight 
          like in equation 21 in design document.*/
    vScale(-1.0, configData->xBar, (size_t) configData->numStates, xErr);
    vAdd(xErr, (size_t) configData->numStates, &configData->SP[0], xErr);
    ukfCholDownDate(configData->sBar, xErr, configData->wC[0],
        configData->numStates, sBarUp);
    
    /*! - Save current sBar matrix, covariance, and state estimate off for further use*/
    mCopy(sBarUp, (size_t) configData->numStates, (size_t) configData->numStates, configData->sBar);
    mTranspose(configData->sBar, (size_t) configData->numStates, (size_t) configData->numStates,
        configData->covar);
	mMultM(configData->sBar, (size_t) configData->numStates, (size_t) configData->numStates,
        configData->covar, (size_t) configData->numStates, (size_t) configData->numStates,
           configData->covar);
    vCopy(&(configData->SP[0]), (size_t) configData->numStates, configData->state );
	
	configData->timeTag = updateTime;
}

/*! This method computes what the expected measurement vector is for each opnave measurement.  All data is transacted from the main
    data structure for the model because there are many variables that would 
    have to be updated otherwise.
 @return void
 @param configData The configuration data associated with the heading estimator

 */
void headingSuKFMeasModel(HeadingSuKF *configData)
{
    /*! - Loop over sigma points */
    int j;
    int i;
    v3Copy(configData->opnavInBuffer.r_BN_B, configData->obs);
    v3Normalize(configData->obs, configData->obs);
    v3Scale(-1, configData->obs, configData->obs);
    for(j=0; j<configData->countHalfSPs*2+1; j++)
    {
        for(i=0; i<3; i++)
        configData->yMeas[i*(configData->countHalfSPs*2+1) + j] =
            configData->SP[i + j*HEAD_N_STATES_SWITCH];
    }
    
    /*! - yMeas matrix was set backwards deliberately so we need to transpose it through*/
    mTranspose(configData->yMeas, OPNAV_MEAS, (size_t) configData->countHalfSPs*2+1,
        configData->yMeas);
    
}

/*! This method performs the measurement update for the heading kalman filter.
 It applies the observations in the obs vectors to the current state estimate and 
 updates the state/covariance with that information.
 @return void
 @param configData The configuration data associated with the heading estimator
 @param updateTime The time that we need to fix the filter to (seconds)
 */
void headingSuKFMeasUpdate(HeadingSuKF *configData, double updateTime)
{
    double yBar[OPNAV_MEAS], syInv[OPNAV_MEAS*OPNAV_MEAS];
    double kMat[HEAD_N_STATES_SWITCH*OPNAV_MEAS];
    double xHat[HEAD_N_STATES_SWITCH], sBarT[HEAD_N_STATES_SWITCH*HEAD_N_STATES_SWITCH], tempYVec[OPNAV_MEAS];
    double AT[(2 * HEAD_N_STATES_SWITCH + OPNAV_MEAS)*OPNAV_MEAS], qChol[OPNAV_MEAS*OPNAV_MEAS];
    double rAT[OPNAV_MEAS*OPNAV_MEAS], syT[OPNAV_MEAS*OPNAV_MEAS];
    double sy[OPNAV_MEAS*OPNAV_MEAS];
    double updMat[OPNAV_MEAS*OPNAV_MEAS], pXY[HEAD_N_STATES_SWITCH*OPNAV_MEAS];
        
    /*! - Compute the valid observations and the measurement model for all observations*/
    headingSuKFMeasModel(configData);
    
    /*! - Compute the value for the yBar parameter (note that this is equation 23 in the 
          time update section of the reference document*/
    vSetZero(yBar, OPNAV_MEAS);
    for(int i=0; i<configData->countHalfSPs*2+1; i++)
    {
        vCopy(&(configData->yMeas[i*OPNAV_MEAS]), OPNAV_MEAS,
              tempYVec);
        vScale(configData->wM[i], tempYVec, OPNAV_MEAS, tempYVec);
        vAdd(yBar, OPNAV_MEAS, tempYVec, yBar);
    }
    
    /*! - Populate the matrix that we perform the QR decomposition on in the measurement 
          update section of the code.  This is based on the differenence between the yBar 
          parameter and the calculated measurement models.  Equation 24 in driving doc. */
    mSetZero(AT, (size_t) configData->countHalfSPs*2+OPNAV_MEAS,
        OPNAV_MEAS);
    for(int i=0; i<configData->countHalfSPs*2; i++)
    {
        vScale(-1.0, yBar, OPNAV_MEAS, tempYVec);
        vAdd(tempYVec, OPNAV_MEAS,
             &(configData->yMeas[(i+1)*OPNAV_MEAS]), tempYVec);
        vScale(sqrt(configData->wC[i+1]), tempYVec, OPNAV_MEAS, tempYVec);
        memcpy(&(AT[i*OPNAV_MEAS]), tempYVec,
               OPNAV_MEAS*sizeof(double));
    }
    
    /*! - This is the square-root of the Rk matrix which we treat as the Cholesky
        decomposition of the observation variance matrix constructed for our number 
        of observations*/
    mCopy(configData->opnavInBuffer.covar_B, OPNAV_MEAS, OPNAV_MEAS, configData->qObs);
    mScale(1/(configData->rNorm*configData->rNorm), configData->qObs, OPNAV_MEAS,OPNAV_MEAS, configData->qObs);
    ukfCholDecomp(configData->qObs, OPNAV_MEAS, OPNAV_MEAS, qChol);
    memcpy(&(AT[2*configData->countHalfSPs*OPNAV_MEAS]),
           qChol, OPNAV_MEAS*OPNAV_MEAS*sizeof(double));
    mScale(configData->noiseSF , AT, 2*configData->countHalfSPs, OPNAV_MEAS, AT);
    /*! - Perform QR decomposition (only R again) of the above matrix to obtain the 
          current Sy matrix*/
    ukfQRDJustR(AT, 2*configData->countHalfSPs+OPNAV_MEAS,
                OPNAV_MEAS, rAT);
    mCopy(rAT, OPNAV_MEAS, OPNAV_MEAS, syT);
    mTranspose(syT, OPNAV_MEAS, OPNAV_MEAS, sy);
    /*! - Shift the matrix over by the difference between the 0th SP-based measurement 
          model and the yBar matrix (cholesky down-date again)*/
    vScale(-1.0, yBar, OPNAV_MEAS, tempYVec);
    vAdd(tempYVec, OPNAV_MEAS, &(configData->yMeas[0]), tempYVec);
    ukfCholDownDate(sy, tempYVec, configData->wC[0],
                    OPNAV_MEAS, updMat);
    /*! - Shifted matrix represents the Sy matrix */
    mCopy(updMat, OPNAV_MEAS, OPNAV_MEAS, sy);
    mTranspose(sy, OPNAV_MEAS, OPNAV_MEAS, syT);

    /*! - Construct the Pxy matrix (equation 26) which multiplies the Sigma-point cloud 
          by the measurement model cloud (weighted) to get the total Pxy matrix*/
    mSetZero(pXY, (size_t) configData->numStates, OPNAV_MEAS);
    for(int i=0; i<2*configData->countHalfSPs+1; i++)
    {
        vScale(-1.0, yBar, OPNAV_MEAS, tempYVec);
        vAdd(tempYVec, OPNAV_MEAS,
             &(configData->yMeas[i*OPNAV_MEAS]), tempYVec);
        vSubtract(&(configData->SP[i* (size_t) configData->numStates]), (size_t) configData->numStates,
                  configData->xBar, xHat);
        vScale(configData->wC[i], xHat, (size_t) configData->numStates, xHat);
        mMultM(xHat, (size_t) configData->numStates, 1, tempYVec, 1, OPNAV_MEAS,
            kMat);
        mAdd(pXY, (size_t) configData->numStates, OPNAV_MEAS, kMat, pXY);
    }

    /*! - Then we need to invert the SyT*Sy matrix to get the Kalman gain factor.  Since
          The Sy matrix is lower triangular, we can do a back-sub inversion instead of 
          a full matrix inversion.  That is the ukfUInv and ukfLInv calls below.  Once that 
          multiplication is done (equation 27), we have the Kalman Gain.*/
    ukfUInv(syT, OPNAV_MEAS, OPNAV_MEAS, syInv);
    mMultM(pXY, (size_t) configData->numStates, OPNAV_MEAS, syInv,
           OPNAV_MEAS, OPNAV_MEAS, kMat);
    ukfLInv(sy, OPNAV_MEAS, OPNAV_MEAS, syInv);
    mMultM(kMat, (size_t) configData->numStates, OPNAV_MEAS, syInv,
           OPNAV_MEAS, OPNAV_MEAS, kMat);
    
    
    /*! - Difference the yBar and the observations to get the observed error and 
          multiply by the Kalman Gain to get the state update.  Add the state update 
          to the state to get the updated state value (equation 27).*/
    vSubtract(configData->obs, OPNAV_MEAS, yBar, tempYVec);
    mMultM(kMat, (size_t) configData->numStates, OPNAV_MEAS, tempYVec,
        OPNAV_MEAS, 1, xHat);
    vAdd(configData->state, (size_t) configData->numStates, xHat, configData->state);
    /*! - Compute the updated matrix U from equation 28.  Note that I then transpose it 
         so that I can extract "columns" from adjacent memory*/
    mMultM(kMat, (size_t) configData->numStates, OPNAV_MEAS, sy,
           OPNAV_MEAS, OPNAV_MEAS, pXY);
    mTranspose(pXY, (size_t) configData->numStates, OPNAV_MEAS, pXY);
    /*! - For each column in the update matrix, perform a cholesky down-date on it to 
          get the total shifted S matrix (called sBar in internal parameters*/
    for(int i=0; i<OPNAV_MEAS; i++)
    {
        vCopy(&(pXY[i* (size_t) configData->numStates]), (size_t) configData->numStates, tempYVec);
        ukfCholDownDate(configData->sBar, tempYVec, -1.0, configData->numStates, sBarT);
        mCopy(sBarT, (size_t) configData->numStates, (size_t) configData->numStates,
            configData->sBar);
    }
    /*! - Compute equivalent covariance based on updated sBar matrix*/
    mTranspose(configData->sBar, (size_t) configData->numStates, (size_t) configData->numStates,
               configData->covar);
    mMultM(configData->sBar, (size_t) configData->numStates, (size_t) configData->numStates,
           configData->covar, (size_t) configData->numStates, (size_t) configData->numStates,
           configData->covar);
}


/*! This method computes the dcms necessary for the switch between the two frames.
 It the switches the states and the covariance, and sets s2 to be the new, different vector of the body frame.
 @return void
 @param bVec_B Pointer to b-vector
 @param states Pointer to the states
 @param covar Pointer to the covariance
 */

void headingSuKFSwitch(double *bVec_B, double *states, double *covar)
{
    double dcm_BSold[HEAD_N_STATES][HEAD_N_STATES];
    double dcm_BSnew_T[HEAD_N_STATES][HEAD_N_STATES];
    double dcm_SnewSold[HEAD_N_STATES][HEAD_N_STATES];
    double switchMatP[HEAD_N_STATES_SWITCH][HEAD_N_STATES_SWITCH];
    double switchMat[HEAD_N_STATES_SWITCH][HEAD_N_STATES_SWITCH];
    
    double sun_heading_norm[HEAD_N_STATES];
    double b1[HEAD_N_STATES];
    double b2[HEAD_N_STATES];
    
    /*!  Set the body frame vectors*/
    v3Set(1, 0, 0, b1);
    v3Set(0, 1, 0, b2);
    v3Normalize(&(states[0]), sun_heading_norm);
    
    /*! Populate the dcm_BS with the "old" S-frame*/
    headingSuKFComputeDCM_BS(sun_heading_norm, bVec_B, &dcm_BSold[0][0]);
    
    if (v3IsEqual(bVec_B, b1, 1e-10))
    {
        headingSuKFComputeDCM_BS(sun_heading_norm, b2, &dcm_BSnew_T[0][0]);
        v3Copy(b2, bVec_B);
    }
    else
    {
        headingSuKFComputeDCM_BS(sun_heading_norm, b1, &dcm_BSnew_T[0][0]);
        v3Copy(b1, bVec_B);
    }
    
    mTranspose(dcm_BSnew_T, HEAD_N_STATES, HEAD_N_STATES, dcm_BSnew_T);
    mMultM(dcm_BSnew_T, 3, 3, dcm_BSold, 3, 3, dcm_SnewSold);
    
    mSetIdentity(switchMat, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH);
    mSetSubMatrix(&dcm_SnewSold[1][1], 1, 2, &switchMat, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, 3, 3);
    mSetSubMatrix(&dcm_SnewSold[2][1], 1, 2, &switchMat, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, 4, 3);
    
    mMultV(switchMat, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, states, states);
    mMultM(switchMat, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, covar, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, switchMatP);
    mTranspose(switchMat, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, switchMat);
    mMultM(switchMatP, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, switchMat, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, covar);
    return;
}

void headingSuKFComputeDCM_BS(double sunheading[HEAD_N_STATES], double bVec[HEAD_N_STATES], double *dcm){
    double s1_B[HEAD_N_STATES];
    double s2_B[HEAD_N_STATES];
    double s3_B[HEAD_N_STATES];
    
    mSetZero(dcm, HEAD_N_STATES, HEAD_N_STATES);
    v3SetZero(s2_B);
    v3SetZero(s3_B);
    
    v3Normalize(sunheading, s1_B);
    v3Cross(sunheading, bVec, s2_B);
    if (v3Norm(s2_B) < 1E-5){
        mSetIdentity(dcm, HEAD_N_STATES, HEAD_N_STATES);
    }
    else{
    v3Normalize(s2_B, s2_B);
    /*! Populate the dcm_BS with the "new" S-frame*/
    mSetSubMatrix(s1_B, 1, HEAD_N_STATES, dcm, HEAD_N_STATES, HEAD_N_STATES, 0, 0);
    mSetSubMatrix(&(s2_B), 1, HEAD_N_STATES, dcm, HEAD_N_STATES, HEAD_N_STATES, 1, 0);
    v3Cross(sunheading, s2_B, s3_B);
    v3Normalize(s3_B, s3_B);
    mSetSubMatrix(&(s3_B), 1, HEAD_N_STATES, dcm, HEAD_N_STATES, HEAD_N_STATES, 2, 0);
    mTranspose(dcm, HEAD_N_STATES, HEAD_N_STATES, dcm);
    }
    
}
