/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include <string.h>
#include <math.h>
#include "pixelLineBiasUKF.h"
#include "architecture/utilities/ukfUtilities.h"


/*! Function for two body dynamics solvers in order to use in the RK4. Only two body dynamics is used currently, but SRP, Solar Gravity, spherical harmonics can be added here.
 @return double Next state
 @param state The starting state
 @param muPlanet planet gravity constant
 @param stateDeriv derivative of state set
 */
static void pixelLineBiasUKFTwoBodyDyn(double state[PIXLINE_DYN_STATES], double muPlanet, double *stateDeriv)
{
    double rNorm;
    double dvdt[3];

    rNorm = v3Norm(state);
    v3Copy(&state[3], stateDeriv);
    v3Copy(state, dvdt);
    v3Scale(-muPlanet/pow(rNorm, 3), dvdt, &stateDeriv[3]);
    return;
}


/*! This method resets the relative OD filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void PixelLineBiasUKF::Reset(uint64_t callTime)
{
    // check if the required message has not been connected
    if (!this->circlesInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: pixelLineBiasUKF.circlesInMsg wasn't connected.");
    }
    if (!this->cameraConfigInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: pixelLineBiasUKF.cameraConfigInMsg wasn't connected.");
    }
    if (!this->attInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: pixelLineBiasUKF.attInMsg wasn't connected.");
    }

    size_t i;
    int32_t badUpdate=0; /* Negative badUpdate is faulty, */
    double tempMatrix[PIXLINE_N_STATES*PIXLINE_N_STATES];

    /*! - Initialize filter parameters to max values */
    this->timeTag = callTime*NANO2SEC;
    this->dt = 0.0;
    this->numStates = PIXLINE_N_STATES;
    this->countHalfSPs = PIXLINE_N_STATES;
    this->numObs = PIXLINE_N_MEAS;
    this->firstPassComplete = 0;
    this->planetId = this->planetIdInit;

    /*! - Ensure that all internal filter matrices are zeroed*/
    vSetZero(this->obs, this->numObs);
    vSetZero(this->wM, this->countHalfSPs * 2 + 1);
    vSetZero(this->wC, this->countHalfSPs * 2 + 1);
    mSetZero(this->sBar, this->numStates, this->numStates);
    mSetZero(this->SP, this->countHalfSPs * 2 + 1,
             this->numStates);
    mSetZero(this->sQnoise, this->numStates, this->numStates);
    mSetZero(this->measNoise, PIXLINE_N_MEAS, PIXLINE_N_MEAS);

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
        this->wM[i] = 1.0 / 2.0*1.0 / (this->numStates + this->lambdaVal);
        this->wC[i] = this->wM[i];
    }

    vCopy(this->stateInit, this->numStates, this->state);
    v6Scale(1E-3, this->state, this->state); // Convert to km
    /*! - User a cholesky decomposition to obtain the sBar and sQnoise matrices for use in filter at runtime*/
    mCopy(this->covarInit, this->numStates, this->numStates,
          this->sBar);
    vScale(1E-6, this->sBar, PIXLINE_DYN_STATES*PIXLINE_N_STATES + PIXLINE_DYN_STATES, this->sBar); // Convert to km
    mCopy(this->covarInit, this->numStates, this->numStates,
          this->covar);
    vScale(1E-6, this->covar, PIXLINE_DYN_STATES*PIXLINE_N_STATES + PIXLINE_DYN_STATES, this->covar); // Convert to km

    mSetZero(tempMatrix, this->numStates, this->numStates);
    badUpdate += ukfCholDecomp(this->sBar, (int) this->numStates,
                               (int) this->numStates, tempMatrix);

    badUpdate += ukfCholDecomp(this->qNoise, (int) this->numStates,
                               (int) this->numStates, this->sQnoise);

    mCopy(tempMatrix, this->numStates, this->numStates,
          this->sBar);
    mTranspose(this->sQnoise, this->numStates,
               this->numStates, this->sQnoise);

    this->timeTagOut = this->timeTag;

    if (badUpdate <0){
        this->bskLogger.bskLog(BSK_WARNING, "Reset method contained bad update");
    }
    return;
}

/*! This method takes the relative position measurements and outputs an estimate of the
 spacecraft states in the intertial frame.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void PixelLineBiasUKF::UpdateState(uint64_t callTime)
{
    double newTimeTag = 0.0;  /* [s] Local Time-tag variable*/
    int32_t trackerValid; /* [-] Indicates whether the star tracker was valid*/
    double yBar[PIXLINE_N_MEAS], tempYVec[PIXLINE_N_MEAS];
    uint64_t i;
    int computePostFits;
    PixelLineFilterMsgPayload opNavOutBuffer = {}; /* [-] Output filter info*/
    NavTransMsgPayload outputRelOD = {};
    OpNavCirclesMsgPayload inputCircles;
    this->moduleId = (int) moduleId;

    computePostFits = 0;
    v3SetZero(this->postFits);


    /*! - read input messages */
    inputCircles = this->circlesInMsg();
    this->cameraSpecs = this->cameraConfigInMsg();
    this->attInfo = this->attInMsg();

    /*! - Handle initializing time in filter and discard initial messages*/
    trackerValid = 0;
    /*! - If the time tag from the measured data is new compared to previous step,
     propagate and update the filter*/
    newTimeTag = this->attInMsg.timeWritten() * NANO2SEC;
    if(newTimeTag >= this->timeTag && this->attInMsg.isWritten() && inputCircles.valid ==1)
    {
        this->circlesInBuffer = inputCircles;
        this->planetId = (int) inputCircles.planetIds[0];
        pixelLineBiasUKFTimeUpdate(newTimeTag);
        pixelLineBiasUKFMeasUpdate();
        computePostFits = 1;
    }
    /*! - If current clock time is further ahead than the measured time, then
     propagate to this current time-step*/
    newTimeTag = callTime*NANO2SEC;
    if(newTimeTag > this->timeTag)
    {
        pixelLineBiasUKFTimeUpdate(newTimeTag);
    }


    /*! - The post fits are y - ybar if a measurement was read, if observations are zero, do not compute post fit residuals*/
    if(computePostFits == 1){
        /*! - Compute Post Fit Residuals, first get Y (eq 22) using the states post fit*/
        pixelLineBiasUKFMeasModel();

        /*! - Compute the value for the yBar parameter (equation 23)*/
        vSetZero(yBar, this->numObs);
        for(i=0; i<this->countHalfSPs*2+1; i++)
        {
            vCopy(&(this->yMeas[i*this->numObs]), this->numObs,
                  tempYVec);
            vScale(this->wM[i], tempYVec, this->numObs, tempYVec);
            vAdd(yBar, this->numObs, tempYVec, yBar);
        }
        mSubtract(this->obs, PIXLINE_N_MEAS, 1, yBar, this->postFits);
    }


    /*! - Write the relative OD estimate into the copy of the navigation message structure*/
    v3Copy(this->state, outputRelOD.r_BN_N);
    outputRelOD.timeTag = this->timeTag;
    v3Scale(1E3, outputRelOD.r_BN_N, outputRelOD.r_BN_N); // Convert to m
    v3Copy(&this->state[3], outputRelOD.v_BN_N);
    v3Scale(1E3, outputRelOD.v_BN_N, outputRelOD.v_BN_N); // Convert to m
    outputRelOD.timeTag = this->timeTagOut;
    this->navStateOutMsg.write(&outputRelOD, this->moduleId, callTime);

    /*! - Populate the filter states output buffer and write the output message*/
    opNavOutBuffer.timeTag = this->timeTag;
    memmove(opNavOutBuffer.covar, this->covar,
            PIXLINE_N_STATES*PIXLINE_N_STATES*sizeof(double));
    memmove(opNavOutBuffer.state, this->state, PIXLINE_N_STATES*sizeof(double));
    memmove(opNavOutBuffer.postFitRes, this->postFits, PIXLINE_N_MEAS*sizeof(double));
    v6Scale(1E3, opNavOutBuffer.state, opNavOutBuffer.state); // Convert to m
    vScale(1E6, opNavOutBuffer.covar, PIXLINE_DYN_STATES*PIXLINE_N_STATES+PIXLINE_DYN_STATES, opNavOutBuffer.covar); // Convert to m
    this->filtDataOutMsg.write(&opNavOutBuffer, this->moduleId, callTime);

    return;
}

/*! This method propagates a relative OD state vector forward in time.  Note
 that the calling parameter is updated in place to save on data copies.
 @return void
 @param stateInOut The state that is propagated
 @param dt Time step (s)
 */
void PixelLineBiasUKF::relODStateProp(double *stateInOut, double dt)
{

    double muPlanet;
    double k1[PIXLINE_DYN_STATES], k2[PIXLINE_DYN_STATES], k3[PIXLINE_DYN_STATES], k4[PIXLINE_DYN_STATES];
    double states1[PIXLINE_DYN_STATES], states2[PIXLINE_DYN_STATES], states3[PIXLINE_DYN_STATES];
    if(this->planetId ==1){muPlanet = MU_EARTH;} //in km
    if(this->planetId ==2){muPlanet = MU_MARS;} //in km
    if(this->planetId ==3){muPlanet = MU_JUPITER;} //in km

    /*! Start RK4 */
    /*! - Compute k1 */
    pixelLineBiasUKFTwoBodyDyn(stateInOut, muPlanet, &k1[0]);
    vScale(dt/2, k1, PIXLINE_DYN_STATES, k1); // k1 is now k1/2
    /*! - Compute k2 */
    vAdd(stateInOut, PIXLINE_DYN_STATES, k1, states1);
    pixelLineBiasUKFTwoBodyDyn(states1, muPlanet, &k2[0]);
    vScale(dt/2, k2, PIXLINE_DYN_STATES, k2); // k2 is now k2/2
    /*! - Compute k3 */
    vAdd(stateInOut, PIXLINE_DYN_STATES, k2, states2);
    pixelLineBiasUKFTwoBodyDyn(states2, muPlanet, &k3[0]);
    vScale(dt, k3, PIXLINE_DYN_STATES, k3);
    /*! - Compute k4 */
    vAdd(stateInOut, PIXLINE_DYN_STATES, k3, states3);
    pixelLineBiasUKFTwoBodyDyn(states3, muPlanet, &k4[0]);
    vScale(dt, k4, PIXLINE_DYN_STATES, k4);
    /*! - Gather all terms with proper scales */
    vScale(1./3., k1, PIXLINE_DYN_STATES, k1); // k1 is now k1/6
    vScale(2./3., k2, PIXLINE_DYN_STATES, k2); // k2 is now k2/3
    vScale(1./3., k3, PIXLINE_DYN_STATES, k3); // k3 is now k2/3
    vScale(1./6., k4, PIXLINE_DYN_STATES, k4); // k4 is now k2/6

    vAdd(stateInOut, PIXLINE_DYN_STATES, k1, stateInOut);
    vAdd(stateInOut, PIXLINE_DYN_STATES, k2, stateInOut);
    vAdd(stateInOut, PIXLINE_DYN_STATES, k3, stateInOut);
    vAdd(stateInOut, PIXLINE_DYN_STATES, k4, stateInOut);

    return;
}

/*! This method performs the time update for the relative OD kalman filter.
 It propagates the sigma points forward in time and then gets the current
 covariance and state estimates.
 @return void
 @param updateTime The time that we need to fix the filter to (seconds)
 */
int PixelLineBiasUKF::pixelLineBiasUKFTimeUpdate(double updateTime)
{
    uint64_t i, Index;
    double sBarT[PIXLINE_N_STATES*PIXLINE_N_STATES]; // Sbar transpose (chol decomp of covar)
    double xComp[PIXLINE_N_STATES], AT[(2 * PIXLINE_N_STATES + PIXLINE_N_STATES)*PIXLINE_N_STATES]; // Intermediate state, process noise chol decomp
    double aRow[PIXLINE_N_STATES], rAT[PIXLINE_N_STATES*PIXLINE_N_STATES], xErr[PIXLINE_N_STATES]; //Row of A mat, R of QR decomp of A, state error
    double sBarUp[PIXLINE_N_STATES*PIXLINE_N_STATES]; // S bar cholupdate
    double *spPtr; //sigma point intermediate varaible
    double procNoise[PIXLINE_N_STATES*PIXLINE_N_STATES]; //process noise
    int32_t badUpdate=0;

    this->dt = updateTime - this->timeTag;
    vCopy(this->state, this->numStates, this->statePrev);
    mCopy(this->sBar, this->numStates, this->numStates, this->sBarPrev);
    mCopy(this->covar, this->numStates, this->numStates, this->covarPrev);

    /*! - Read the planet ID from the message*/
    if(this->planetId == 0)
    {
      this->bskLogger.bskLog(BSK_ERROR, "Need a planet to navigate");
    }

    mCopy(this->sQnoise, PIXLINE_N_STATES, PIXLINE_N_STATES, procNoise);
    /*! - Copy over the current state estimate into the 0th Sigma point and propagate by dt*/
    vCopy(this->state, this->numStates,
          &(this->SP[0 * this->numStates + 0]));
    this->relODStateProp(&(this->SP[0]),this->dt);
    /*! - Scale that Sigma point by the appopriate scaling factor (Wm[0])*/
    vScale(this->wM[0], &(this->SP[0]),
           this->numStates, this->xBar);
    /*! - Get the transpose of the sBar matrix because it is easier to extract Rows vs columns*/
    mTranspose(this->sBar, this->numStates, this->numStates,
               sBarT);
    /*! - For each Sigma point, apply sBar-based error, propagate forward, and scale by Wm just like 0th.
     Note that we perform +/- sigma points simultaneously in loop to save loop values.*/
    for (i = 0; i<this->countHalfSPs; i++)
    {
        /*! - Adding covariance columns from sigma points*/
        Index = i + 1;
        spPtr = &(this->SP[Index*this->numStates]);
        vCopy(&sBarT[i*this->numStates], this->numStates, spPtr);
        vScale(this->gamma, spPtr, this->numStates, spPtr);
        vAdd(spPtr, this->numStates, this->state, spPtr);
        this->relODStateProp(spPtr, this->dt);
        vScale(this->wM[Index], spPtr, this->numStates, xComp);
        vAdd(xComp, this->numStates, this->xBar, this->xBar);
        /*! - Subtracting covariance columns from sigma points*/
        Index = i + 1 + this->countHalfSPs;
        spPtr = &(this->SP[Index*this->numStates]);
        vCopy(&sBarT[i*this->numStates], this->numStates, spPtr);
        vScale(-this->gamma, spPtr, this->numStates, spPtr);
        vAdd(spPtr, this->numStates, this->state, spPtr);
        this->relODStateProp(spPtr, this->dt);
        vScale(this->wM[Index], spPtr, this->numStates, xComp);
        vAdd(xComp, this->numStates, this->xBar, this->xBar);
    }
    /*! - Zero the AT matrix prior to assembly*/
    mSetZero(AT, (2 * this->countHalfSPs + this->numStates),
             this->countHalfSPs);
    /*! - Assemble the AT matrix.  Note that this matrix is the internals of
     the qr decomposition call in the source design documentation.  It is
     the inside of equation 20 in that document*/
    for (i = 0; i<2 * this->countHalfSPs; i++)
    {
        vScale(-1.0, this->xBar, this->numStates, aRow);
        vAdd(aRow, this->numStates,
             &(this->SP[(i+1)*this->numStates]), aRow);
        /*Check sign of wC to know if the sqrt will fail*/
        if (this->wC[i+1]<=0){
            pixelLineBiasUKFCleanUpdate();
            return -1;}
        vScale(sqrt(this->wC[i+1]), aRow, this->numStates, aRow);
        memcpy((void *)&AT[i*this->numStates], (void *)aRow,
               this->numStates*sizeof(double));

    }
    /*! - Pop the sQNoise matrix on to the end of AT prior to getting QR decomposition*/
    memcpy(&AT[2 * this->countHalfSPs*this->numStates],
           procNoise, this->numStates*this->numStates
           *sizeof(double));
    /*! - QR decomposition (only R computed!) of the AT matrix provides the new sBar matrix*/
    ukfQRDJustR(AT, (int) (2 * this->countHalfSPs + this->numStates),
                (int) this->countHalfSPs, rAT);

    mCopy(rAT, this->numStates, this->numStates, sBarT);
    mTranspose(sBarT, this->numStates, this->numStates,
               this->sBar);

    /*! - Shift the sBar matrix over by the xBar vector using the appropriate weight
     like in equation 21 in design document.*/
    vScale(-1.0, this->xBar, this->numStates, xErr);
    vAdd(xErr, this->numStates, &this->SP[0], xErr);
    badUpdate += ukfCholDownDate(this->sBar, xErr, this->wC[0],
                                 (int) this->numStates, sBarUp);


    /*! - Save current sBar matrix, covariance, and state estimate off for further use*/
    mCopy(sBarUp, this->numStates, this->numStates, this->sBar);
    mTranspose(this->sBar, this->numStates, this->numStates,
               this->covar);
    mMultM(this->sBar, this->numStates, this->numStates,
           this->covar, this->numStates, this->numStates,
           this->covar);
    vCopy(&(this->SP[0]), this->numStates, this->state);

    if (badUpdate<0){
        pixelLineBiasUKFCleanUpdate();
        return(-1);}
    else{
        this->timeTag = updateTime;
    }
    return(0);
}

/*! This method computes the measurement model.  Given that the data is coming from
 the pixelLine Converter, the transformation has already taken place from pixel data to spacecraft position.
 @return void
 @param this The configuration data associated with the OD filter
 */
void PixelLineBiasUKF::pixelLineBiasUKFMeasModel()
{
    size_t i, j;
    double dcm_CN[3][3], dcm_CB[3][3], dcm_BN[3][3];
    double reCentered[2], rNorm, denom, planetRad;
    double r_C[3];

    v3Set(this->circlesInBuffer.circlesCenters[0], this->circlesInBuffer.circlesCenters[1], this->circlesInBuffer.circlesRadii[0], this->obs);

    MRP2C(this->cameraSpecs.sigma_CB, dcm_CB);
    MRP2C(this->attInfo.sigma_BN, dcm_BN);
    m33MultM33(dcm_CB, dcm_BN, dcm_CN);
    double X, Y;
    double pX, pY;
    /* compute sensorSize/focalLength = 2*tan(FOV/2) */
    pX = 2.*tan(this->cameraSpecs.fieldOfView*this->cameraSpecs.resolution[0]/this->cameraSpecs.resolution[1]/2.0);
    pY = 2.*tan(this->cameraSpecs.fieldOfView/2.0);
    X = pX/this->cameraSpecs.resolution[0];
    Y = pY/this->cameraSpecs.resolution[1];

    if(this->circlesInBuffer.planetIds[0] > 0){
        if(this->circlesInBuffer.planetIds[0] ==1){
            planetRad = REQ_EARTH;//in km
        }
        if(this->circlesInBuffer.planetIds[0] ==2){
            planetRad = REQ_MARS;//in km
        }
        if(this->circlesInBuffer.planetIds[0] ==3){
            planetRad = REQ_JUPITER;//in km
        }
    }

    for(j=0; j<this->countHalfSPs*2+1; j++)
    {
        double centers[2], r_N_bar[3], radius=0;
        v2SetZero(centers);
        v3SetZero(r_N_bar);

        v3Copy(&this->SP[j*this->numStates], r_N_bar);
        rNorm = v3Norm(r_N_bar);

        m33MultV3(dcm_CN, r_N_bar, r_C);
        v3Scale(-1./r_C[2], r_C, r_C);

        /*! - Find pixel size using camera specs */
        reCentered[0] = r_C[0]/X;
        reCentered[1] = r_C[1]/Y;

        centers[0] = reCentered[0] + this->cameraSpecs.resolution[0]/2 - 0.5;
        centers[1] = reCentered[1] + this->cameraSpecs.resolution[1]/2 - 0.5;

        denom = planetRad/rNorm;
        radius = tan(safeAsin(denom)) / X;
        if (j==0){
            v2Subtract(centers, this->obs, &this->obs[3]);
            this->obs[5] = radius - this->obs[2];
        }
        for(i=0; i<3; i++){
            this->obs[i+3] = round(this->obs[i+3]);
            if (i<2){
                this->yMeas[i*(this->countHalfSPs*2+1) + j] = centers[i] - this->SP[j*this->numStates+PIXLINE_DYN_STATES + i];
            }
            if (i==2){
                this->yMeas[i*(this->countHalfSPs*2+1) + j] = radius - this->SP[j*this->numStates+PIXLINE_DYN_STATES + i];
            }
            this->yMeas[(i+PIXLINE_N_MEAS/2)*(this->countHalfSPs*2+1) + j] = this->SP[j*this->numStates+ PIXLINE_DYN_STATES + i];
        }
    }
    /*! - yMeas matrix was set backwards deliberately so we need to transpose it through*/
    mTranspose(this->yMeas, PIXLINE_N_MEAS, this->countHalfSPs*2+1,
               this->yMeas);

}

/*! This method performs the measurement update for the kalman filter.
 It applies the observations in the obs vectors to the current state estimate and
 updates the state/covariance with that information.
 @return void
 @param this The configuration data associated with the OD filter
 */
int PixelLineBiasUKF::pixelLineBiasUKFMeasUpdate()
{
    uint32_t i;
    double yBar[PIXLINE_N_MEAS], syInv[PIXLINE_N_MEAS*PIXLINE_N_MEAS]; //measurement, Sy inv
    double kMat[PIXLINE_N_STATES*PIXLINE_N_MEAS], cholNoise[PIXLINE_N_MEAS*PIXLINE_N_MEAS];//Kalman Gain, chol decomp of noise
    double xHat[PIXLINE_N_STATES], Ucol[PIXLINE_N_STATES], sBarT[PIXLINE_N_STATES*PIXLINE_N_STATES], tempYVec[PIXLINE_N_MEAS];// state error, U column eq 28, intermediate variables
    double AT[(2 * PIXLINE_N_STATES + PIXLINE_N_MEAS)*PIXLINE_N_MEAS]; //Process noise matrix
    double rAT[PIXLINE_N_MEAS*PIXLINE_N_MEAS], syT[PIXLINE_N_MEAS*PIXLINE_N_MEAS]; //QR R decomp, Sy transpose
    double sy[PIXLINE_N_MEAS*PIXLINE_N_MEAS]; // Chol of covariance
    double updMat[PIXLINE_N_MEAS*PIXLINE_N_MEAS], pXY[PIXLINE_N_STATES*PIXLINE_N_MEAS], Umat[PIXLINE_N_STATES*PIXLINE_N_MEAS]; // Intermediate variable, covariance eq 26, U eq 28
    int32_t badUpdate=0;

    vCopy(this->state, this->numStates, this->statePrev);
    mCopy(this->sBar, this->numStates, this->numStates, this->sBarPrev);
    mCopy(this->covar, this->numStates, this->numStates, this->covarPrev);
    /*! - Compute the valid observations and the measurement model for all observations*/
    pixelLineBiasUKFMeasModel();

    /*! - Compute the value for the yBar parameter (note that this is equation 23 in the
     time update section of the reference document*/
    vSetZero(yBar, this->numObs);
    for(i=0; i<this->countHalfSPs*2+1; i++)
    {
        vCopy(&(this->yMeas[i*this->numObs]), this->numObs,
              tempYVec);
        vScale(this->wM[i], tempYVec, this->numObs, tempYVec);
        vAdd(yBar, this->numObs, tempYVec, yBar);
    }

    /*! - Populate the matrix that we perform the QR decomposition on in the measurement
     update section of the code.  This is based on the differenence between the yBar
     parameter and the calculated measurement models.  Equation 24 in driving doc. */
    mSetZero(AT, this->countHalfSPs*2+this->numObs,
             this->numObs);
    for(i=0; i<this->countHalfSPs*2; i++)
    {
        vScale(-1.0, yBar, this->numObs, tempYVec);
        vAdd(tempYVec, this->numObs,
             &(this->yMeas[(i+1)*this->numObs]), tempYVec);
        if (this->wC[i+1]<0){return -1;}
        vScale(sqrt(this->wC[i+1]), tempYVec, this->numObs, tempYVec);
        memcpy(&(AT[i*this->numObs]), tempYVec,
               this->numObs*sizeof(double));
    }

    /*! - This is the square-root of the Rk matrix which we treat as the Cholesky
     decomposition of the observation variance matrix constructed for our number
     of observations*/
    mSetZero(this->measNoise, PIXLINE_N_MEAS, PIXLINE_N_MEAS);
    mSetSubMatrix(this->circlesInBuffer.uncertainty, 3, 3, this->measNoise, PIXLINE_N_MEAS, PIXLINE_N_MEAS, 3,3);
    mSetSubMatrix(this->circlesInBuffer.uncertainty, 3, 3, this->measNoise, PIXLINE_N_MEAS, PIXLINE_N_MEAS, 0, 0);
    badUpdate += ukfCholDecomp(this->measNoise, PIXLINE_N_MEAS, PIXLINE_N_MEAS, cholNoise);
    memcpy(&(AT[2*this->countHalfSPs*this->numObs]),
           cholNoise, this->numObs*this->numObs*sizeof(double));
    /*! - Perform QR decomposition (only R again) of the above matrix to obtain the
     current Sy matrix*/
    ukfQRDJustR(AT, (int) (2*this->countHalfSPs+this->numObs),
                (int) this->numObs, rAT);

    mCopy(rAT, this->numObs, this->numObs, syT);
    mTranspose(syT, this->numObs, this->numObs, sy);
    /*! - Shift the matrix over by the difference between the 0th SP-based measurement
     model and the yBar matrix (cholesky down-date again)*/
    vScale(-1.0, yBar, this->numObs, tempYVec);
    vAdd(tempYVec, this->numObs, &(this->yMeas[0]), tempYVec);
    badUpdate += ukfCholDownDate(sy, tempYVec, this->wC[0],
                                 (int) this->numObs, updMat);

    /*! - Shifted matrix represents the Sy matrix */
    mCopy(updMat, this->numObs, this->numObs, sy);
    mTranspose(sy, this->numObs, this->numObs, syT);

    /*! - Construct the Pxy matrix (equation 26) which multiplies the Sigma-point cloud
     by the measurement model cloud (weighted) to get the total Pxy matrix*/
    mSetZero(pXY, this->numStates, this->numObs);
    for(i=0; i<2*this->countHalfSPs+1; i++)
    {
        vScale(-1.0, yBar, this->numObs, tempYVec);
        vAdd(tempYVec, this->numObs,
             &(this->yMeas[i*this->numObs]), tempYVec);
        vSubtract(&(this->SP[i*this->numStates]), this->numStates,
                  this->xBar, xHat);
        vScale(this->wC[i], xHat, this->numStates, xHat);
        mMultM(xHat, this->numStates, 1, tempYVec, 1, this->numObs,
               kMat);
        mAdd(pXY, this->numStates, this->numObs, kMat, pXY);
    }

    /*! - Then we need to invert the SyT*Sy matrix to get the Kalman gain factor.  Since
     The Sy matrix is lower triangular, we can do a back-sub inversion instead of
     a full matrix inversion.  That is the ukfUInv and ukfLInv calls below.  Once that
     multiplication is done (equation 27), we have the Kalman Gain.*/
    ukfUInv(syT, (int) this->numObs, (int) this->numObs, syInv);

    mMultM(pXY, this->numStates, this->numObs, syInv,
           this->numObs, this->numObs, kMat);
    ukfLInv(sy, (int) this->numObs, (int) this->numObs, syInv);
    mMultM(kMat, this->numStates, this->numObs, syInv,
           this->numObs, this->numObs, kMat);


    /*! - Difference the yBar and the observations to get the observed error and
     multiply by the Kalman Gain to get the state update.  Add the state update
     to the state to get the updated state value (equation 27).*/
    vSubtract(this->obs, this->numObs, yBar, tempYVec);
    mMultM(kMat, this->numStates, this->numObs, tempYVec,
           this->numObs, 1, xHat);
    vAdd(this->state, this->numStates, xHat, this->state);
    for (i=6;i<9;i++){
        this->state[i] = round(this->state[i]);
    }
    /*! - Compute the updated matrix U from equation 28.  Note that I then transpose it
     so that I can extract "columns" from adjacent memory*/
    mMultM(kMat, this->numStates, this->numObs, sy,
           this->numObs, this->numObs, Umat);
    mTranspose(Umat, this->numStates, this->numObs, Umat);
    /*! - For each column in the update matrix, perform a cholesky down-date on it to
     get the total shifted S matrix (called sBar in internal parameters*/
    for(i=0; i<this->numObs; i++)
    {
        vCopy(&(Umat[i*this->numStates]), this->numStates, Ucol);
        badUpdate += ukfCholDownDate(this->sBar, Ucol, -1.0, (int) this->numStates, sBarT);
        mCopy(sBarT, this->numStates, this->numStates,
              this->sBar);
    }

    /*! - Compute equivalent covariance based on updated sBar matrix*/
    mTranspose(this->sBar, this->numStates, this->numStates,
               this->covar);
    mMultM(this->sBar, this->numStates, this->numStates,
           this->covar, this->numStates, this->numStates,
           this->covar);

    if (badUpdate<0){
        pixelLineBiasUKFCleanUpdate();
        return(-1);}
    return(0);
}

/*! This method cleans the filter states after a bad upadate on the fly.
 It removes the potentially corrupted previous estimates and puts the filter
 back to a working state.
 @return void
 @param this The configuration data associated with the OD filter
 */
void PixelLineBiasUKF::pixelLineBiasUKFCleanUpdate(){
    size_t i;
    /*! - Reset the observations, state, and covariannces to a previous safe value*/
    vSetZero(this->obs, this->numObs);
    vCopy(this->statePrev, this->numStates, this->state);
    mCopy(this->sBarPrev, this->numStates, this->numStates, this->sBar);
    mCopy(this->covarPrev, this->numStates, this->numStates, this->covar);

    /*! - Reset the wM/wC vectors to standard values for unscented kalman filters*/
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

    return;
}
