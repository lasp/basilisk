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

#include "fswAlgorithms/sensorInterfaces/CSSSensorData/cssComm.h"
#include "architecture/utilities/linearAlgebra.h"
#include <string.h>
#include <stdio.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void CSSComm::Reset(uint64_t callTime)
{
    // check if the required message has not been connected
    if (!this->sensorListInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: cssComm.sensorListInMsg wasn't connected.");
    }

    /*! - Check to make sure that number of sensors is less than the max and warn if none are set*/
    if(this->numSensors > MAX_NUM_CSS_SENSORS)
    {
        char info[MAX_LOGGING_LENGTH];
        snprintf(info, sizeof(info), "The configured number of CSS sensors exceeds the maximum, %d > %d! Changing the number of sensors to the max.", this->numSensors, MAX_NUM_CSS_SENSORS);
        this->bskLogger.bskLog(BSK_WARNING, info);
        this->numSensors = MAX_NUM_CSS_SENSORS;
    }
    else if (this->numSensors == 0)
    {
        this->bskLogger.bskLog(BSK_WARNING, "There are zero CSS configured!");
    }
    
    if (this->maxSensorValue == 0)
    {
        this->bskLogger.bskLog(BSK_WARNING, "Max CSS sensor value configured to zero! CSS sensor values will be normalized by zero, inducing faux saturation!");
    }


    return;
}


/*! This method takes the raw sensor data from the coarse sun sensors and
 converts that information to the format used by the CSS nav.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void CSSComm::UpdateState(uint64_t callTime)
{
    uint32_t i, j;
    double inputValues[MAX_NUM_CSS_SENSORS]; /* [-] Current measured CSS value for the constellation of CSS sensor */
    double ChebyDiffFactor, ChebyPrev, ChebyNow, ChebyLocalPrev, ValueMult; /* Parameters used for the Chebyshev Recursion Forumula */

    CSSArraySensorMsgPayload outputBuffer = {};

    // read sensor list input msg
    CSSArraySensorMsgPayload inMsgBuffer = this->sensorListInMsg();
    vCopy(inMsgBuffer.CosValue, MAX_NUM_CSS_SENSORS, inputValues);

    /*! - Loop over the sensors and compute data
         -# Check appropriate range on sensor and calibrate
         -# If Chebyshev polynomials are configured:
             - Seed polynominal computations
             - Loop over polynominals to compute estimated correction factor
             - Output is base value plus the correction factor
         -# If sensor output range is incorrect, set output value to zero
     */
    for(i=0; i<this->numSensors; i++)
    {
        outputBuffer.CosValue[i] = (float) inputValues[i]/this->maxSensorValue; /* Scale Sensor Data */
        
        /* Seed the polynomial computations */
        ValueMult = 2.0*outputBuffer.CosValue[i];
        ChebyPrev = 1.0;
        ChebyNow = outputBuffer.CosValue[i];
        ChebyDiffFactor = 0.0;
        ChebyDiffFactor = this->chebyCount > 0 ? ChebyPrev*this->kellyCheby[0] : ChebyDiffFactor; /* if only first order correction */
        ChebyDiffFactor = this->chebyCount > 1 ? ChebyNow*this->kellyCheby[1] + ChebyDiffFactor : ChebyDiffFactor; /* if higher order (> first) corrections */
        
        /* Loop over remaining polynomials and add in values */
        for(j=2; j<this->chebyCount; j = j+1)
        {
            ChebyLocalPrev = ChebyNow;
            ChebyNow = ValueMult*ChebyNow - ChebyPrev;
            ChebyPrev = ChebyLocalPrev;
            ChebyDiffFactor += this->kellyCheby[j]*ChebyNow;
        }
        
        outputBuffer.CosValue[i] = outputBuffer.CosValue[i] + ChebyDiffFactor;
        
        if(outputBuffer.CosValue[i] > 1.0)
        {
            outputBuffer.CosValue[i] = 1.0;
        }
        else if(outputBuffer.CosValue[i] < 0.0)
        {
            outputBuffer.CosValue[i] = 0.0;
        }
    }
    
    /*! - Write aggregate output into output message */
    this->cssArrayOutMsg.write(&outputBuffer, this->moduleID, callTime);
    
    return;
}
