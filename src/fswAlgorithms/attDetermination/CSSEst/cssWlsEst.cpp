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

#include "fswAlgorithms/attDetermination/CSSEst/cssWlsEst.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include <string.h>

int computeWlsmn(int numActiveCss, double *H, double *W, double *y, double x[3]);
void computeWlsResiduals(double *cssMeas, CSSConfigMsgPayload *cssConfig, double *wlsEst, double *cssResiduals);

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void CssWlsEst::Reset(uint64_t callTime)
{

    // check that required messages have been included
    if (!this->cssConfigInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: cssWIsEst.cssConfigInMsg wasn't connected.");
    }
    if (!this->cssDataInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: cssWIsEst.cssDataInMsg wasn't connected.");
    }

    this->cssConfigInBuffer = this->cssConfigInMsg();
    if (this->cssConfigInBuffer.nCSS > MAX_N_CSS_MEAS) {
        this->bskLogger.bskLog(BSK_ERROR, "cssWIsEst.cssDataInMsg.nCSS must not be greater than "
                                                  "MAX_N_CSS_MEAS value.");
    }

    this->priorSignalAvailable = 0;
    v3SetZero(this->dOld);

    this->filtStatus.numObs = 0;
    this->filtStatus.timeTag = 0.0;
    v3SetZero(this->filtStatus.state);
    vSetZero(this->filtStatus.postFitRes, MAX_N_CSS_MEAS);


    /* Reset the prior time flag state.
     If zero, control time step not evaluated on the first function call */
    this->priorTime = 0;

    return;
}


/*! This method takes the parsed CSS sensor data and outputs an estimate of the
 sun vector in the ADCS body frame
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void CssWlsEst::UpdateState(uint64_t callTime)
{
    CSSArraySensorMsgPayload InputBuffer;        /* CSS measurements */
    double H[MAX_NUM_CSS_SENSORS*3];             /* The predicted pointing vector for each measurement */
    double y[MAX_NUM_CSS_SENSORS];               /* Measurements */
    double W[MAX_NUM_CSS_SENSORS*MAX_NUM_CSS_SENSORS];  /* Matrix of measurement weights */
    int status = 0;                              /* Quality of the module estimate */
    double dOldDotNew;                           /* Intermediate value for dot product between new and old estimates for rate estimation */
    double dHatNew[3];                           /* New normalized sun heading estimate */
    double dHatOld[3];                           /* Prior normalized sun heading estimate */
    double  dt;                                  /* [s] Control update period */
    NavAttMsgPayload sunlineOutBuffer = {};               /* Output Nav message*/

    /*! Message Read and Setup*/
    /*! - Read the input parsed CSS sensor data message*/
    InputBuffer = this->cssDataInMsg();

    /*! - Compute control update time */
    if (this->priorTime == 0) {
        dt = 0.0;
    } else {
        dt = (callTime - this->priorTime) * NANO2SEC;
    }
    this->priorTime = callTime;

    /* - Zero the observed active CSS count*/
    this->numActiveCss = 0;

    /*! - Loop over the maximum number of sensors to check for good measurements */
    /*! -# Isolate if measurement is good */
    /*! -# Set body vector for this measurement */
    /*! -# Get measurement value into observation vector */
    /*! -# Set inverse noise matrix */
    /*! -# increase the number of valid observations */
    /*! -# Otherwise just continue */
    for(uint32_t i=0; i<this->cssConfigInBuffer.nCSS; i = i+1)
    {
        if(InputBuffer.CosValue[i] > this->sensorUseThresh)
        {
            v3Scale(this->cssConfigInBuffer.cssVals[i].CBias,
                this->cssConfigInBuffer.cssVals[i].nHat_B, &H[this->numActiveCss*3]);
            y[this->numActiveCss] = InputBuffer.CosValue[i];
            this->numActiveCss = this->numActiveCss + 1;

        }
    }

    /*! Estimation Steps*/
    this->filtStatus = {};

    if(this->numActiveCss == 0) /*! - If there is no sun, just quit*/
    {
        /*! + If no CSS got a strong enough signal.  Sun estimation is not possible.  Return the zero vector instead */
        v3SetZero(sunlineOutBuffer.vehSunPntBdy);       /* zero the sun heading to indicate now CSS info is available */
        v3SetZero(sunlineOutBuffer.omega_BN_B);         /* zero the rate measure */
        this->priorSignalAvailable = 0;                       /* reset the prior heading estimate flag */
        computeWlsResiduals(InputBuffer.CosValue, &this->cssConfigInBuffer,
                            sunlineOutBuffer.vehSunPntBdy, this->filtStatus.postFitRes);
    } else {
        /*! - If at least one CSS got a strong enough signal.  Proceed with the sun heading estimation */
        /*! -# Configuration option to weight the measurements, otherwise set
         weighting matrix to identity*/
        if(this->useWeights > 0)
        {
            mDiag(y, this->numActiveCss, W);
        }
        else
        {
            mSetIdentity(W, this->numActiveCss, this->numActiveCss);
        }
        /*! -# Get least squares fit for sun pointing vector*/
        status = computeWlsmn((int) this->numActiveCss, H, W, y,
                              sunlineOutBuffer.vehSunPntBdy);
        computeWlsResiduals(InputBuffer.CosValue, &this->cssConfigInBuffer,
                            sunlineOutBuffer.vehSunPntBdy, this->filtStatus.postFitRes);

        v3Normalize(sunlineOutBuffer.vehSunPntBdy, sunlineOutBuffer.vehSunPntBdy);

        /*! -# Estimate the inertial angular velocity from the rate of the sun heading measurements */
        if (this->priorSignalAvailable && dt > 0.0) {
            v3Normalize(sunlineOutBuffer.vehSunPntBdy, dHatNew);
            v3Normalize(this->dOld, dHatOld);
            v3Cross(dHatNew, dHatOld, sunlineOutBuffer.omega_BN_B);
            v3Normalize(sunlineOutBuffer.omega_BN_B,sunlineOutBuffer.omega_BN_B);
            /* compute principal rotation angle between sun heading measurements */
            dOldDotNew = v3Dot(dHatNew,dHatOld);
            if (dOldDotNew > 1.0) dOldDotNew = 1.0;
            if (dOldDotNew < -1.0) dOldDotNew = -1.0;
            v3Scale(safeAcos(dOldDotNew)/dt, sunlineOutBuffer.omega_BN_B, sunlineOutBuffer.omega_BN_B);
        } else {
            this->priorSignalAvailable = 1;
        }
        /*! -# Store the sun heading estimate */
        v3Copy(sunlineOutBuffer.vehSunPntBdy, this->dOld);
    }

    /*! Residual Computation */
    /*! - If the residual fit output message is set, then compute the residuals and stor them in the output message */
    if (this->cssWLSFiltResOutMsg.isLinked()) {
        this->filtStatus.numObs = (int) this->numActiveCss;
        this->filtStatus.timeTag = (double) (callTime*NANO2SEC);
        v3Copy(sunlineOutBuffer.vehSunPntBdy, this->filtStatus.state);
        this->cssWLSFiltResOutMsg.write(&this->filtStatus, this->moduleID, callTime);

    }
    /*! Writing Outputs */
    if(status > 0) /*! - If the status from the WLS computation is erroneous, populate the output messages with zeros*/
    {
        /* An error was detected while attempting to compute the sunline direction */
        v3SetZero(sunlineOutBuffer.vehSunPntBdy);       /* zero the sun heading to indicate anomaly  */
        v3SetZero(sunlineOutBuffer.omega_BN_B);         /* zero the rate measure */
        this->priorSignalAvailable = 0;                       /* reset the prior heading estimate flag */
    }
    /*! - If the status from the WLS computation good, populate the output messages with the computed data*/
    this->navStateOutMsg.write(&sunlineOutBuffer, this->moduleID, callTime);
    return;
}

/*! This method computes the post-fit residuals for the WLS estimate.  Note that
    everything has to have been allocated appropriately as this function operates
    directly on the arrays.
    @return void
    @param cssMeas The measured values for the CSS sensors
    @param cssConfig The CSS configuration information
    @param wlsEst The WLS estimate computed for the CSS measurements
    @param cssResiduals The measurement residuals output by this function
*/
void computeWlsResiduals(double *cssMeas, CSSConfigMsgPayload *cssConfig,
                         double *wlsEst, double *cssResiduals)
{
    double cssDotProd;

    memset(cssResiduals, 0x0, cssConfig->nCSS*sizeof(double));
    /*! The method loops through the sensors and performs: */
    for(uint32_t i=0; i<cssConfig->nCSS; i++)
    {
        /*! -# A dot product between the computed estimate with each sensor normal */
        cssDotProd = v3Dot(wlsEst, cssConfig->cssVals[i].nHat_B);
        cssDotProd = cssDotProd > 0.0 ? cssDotProd : 0.0; /*CSS values can't be negative!*/
        /*! -# A subtraction between that post-fit measurement estimate and the actual measurement*/
        cssResiduals[i] = cssMeas[i] - cssDotProd;
        /*! -# This populates the post-fit residuals*/
    }

}

/*! This method computes a least squares fit with the given parameters.  It
 treats the inputs as though they were double dimensioned arrays but they
 are all singly dimensioned for ease of use
 @return success indicator (0 for good, 1 for fail)
 @param numActiveCss The count on input measurements
 @param H The predicted pointing vector for each measurement
 @param W the weighting matrix for the set of measurements
 @param y the observation vector for the valid sensors
 @param x The output least squares fit for the observations
 */
int computeWlsmn(int numActiveCss, double *H, double *W,
                 double *y, double x[3])
{
    double m22[2*2];
    double m32[3*2];
    int status = 0;
    double  m33[3*3];
    double  m33_2[3*3];
    double  m3N[3*MAX_NUM_CSS_SENSORS];
    double  m3N_2[3*MAX_NUM_CSS_SENSORS];
    uint32_t i;

    /*! - If we only have one sensor, output best guess (cone of possiblities)*/
    if(numActiveCss == 1) {
        /* Here's a guess.  Do with it what you will. */
        for(i = 0; i < 3; i=i+1) {
            x[i] = H[0*MAX_NUM_CSS_SENSORS+i] * y[0];
        }
    } else if(numActiveCss == 2) { /*! - If we have two, then do a 2x2 fit */

        /*!   -# Find minimum norm solution */
        mMultMt(H, 2, 3, H, 2, 3, m22);
        status = m22Inverse(RECAST2x2 m22, RECAST2x2 m22);
        mtMultM(H, 2, 3, m22, 2, 2, m32);
        /*!   -# Multiply the Ht(HHt)^-1 by the observation vector to get fit*/
        mMultV(m32, 3, 2, y, x);
    } else if(numActiveCss > 2) {/*! - If we have more than 2, do true LSQ fit*/
        /*!    -# Use the weights to compute (HtWH)^-1HW*/
        mtMultM(H, (size_t) numActiveCss, 3, W, (size_t) numActiveCss, (size_t) numActiveCss, m3N);
        mMultM(m3N, 3, (size_t) numActiveCss, H, (size_t) numActiveCss, 3, m33);
        status = m33Inverse(RECAST3X3 m33, RECAST3X3 m33_2);
        mMultMt(m33_2, 3, 3, H, (size_t) numActiveCss, 3, m3N);
        mMultM(m3N, 3, (size_t) numActiveCss, W, (size_t) numActiveCss, (size_t) numActiveCss, m3N_2);
        /*!    -# Multiply the LSQ matrix by the obs vector for best fit*/
        mMultV(m3N_2, 3, (size_t) numActiveCss, y, x);
    }

    return(status);
}
