/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric Space Physics, University of Colorado at Boulder

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

#ifndef _OE_STATE_EPHEM_H_
#define _OE_STATE_EPHEM_H_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/TDBVehicleClockCorrelationMsgPayload.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"

#define MAX_OE_RECORDS 10
#define MAX_OE_COEFF 20



/*! @brief Structure that defines the layout of an Ephemeris "record."  This is
           basically the set of coefficients for the ephemeris elements and
           the time factors associated with those coefficients
*/
typedef struct {
    uint32_t nChebCoeff;                  //!< [-] Number chebyshev coefficients loaded into record
    double ephemTimeMid;                  //!< [s] Ephemeris time (TDB) associated with the mid-point of the curve
    double ephemTimeRad;                  //!< [s] "Radius" of time that curve is valid for (half of total range
    double rPeriapCoeff[MAX_OE_COEFF];    //!< [-] Set of chebyshev coefficients for radius at periapses
    double eccCoeff[MAX_OE_COEFF];        //!< [-] Set of chebyshev coefficients for eccentrity
    double incCoeff[MAX_OE_COEFF];        //!< [-] Set of chebyshev coefficients for inclination
    double argPerCoeff[MAX_OE_COEFF];     //!< [-] Set of chebyshev coefficients for argument of periapses
    double RAANCoeff[MAX_OE_COEFF];       //!< [-] Set of chebyshev coefficients for right ascention of the ascending node
    double anomCoeff[MAX_OE_COEFF];       //!< [-] Set of chebyshev coefficients for true anomaly angle
    uint32_t anomalyFlag;                 //!< [-] Flag indicating if the anomaly angle is true (0), mean (1)
}ChebyOERecord;

/*! @brief Top level structure for the Chebyshev position ephemeris
           fit system.  Allows the user to specify a set of chebyshev
           coefficients and then use the input time to determine where
           a given body is in space
*/
class OEStateEphem : public SysModel {
public:
    void UpdateState(uint64_t callTime) override;
    void Reset(uint64_t callTime) override;

    Message<EphemerisMsgPayload> stateFitOutMsg; //!< [-] output navigation message for pos/vel
    ReadFunctor<TDBVehicleClockCorrelationMsgPayload> clockCorrInMsg; //!< clock correlation input message

    double muCentral;                             //!< [m3/s^2] Gravitational parameter for center of orbital elements
    ChebyOERecord ephArray[MAX_OE_RECORDS];       //!< [-] Array of Chebyshev records for ephemeris
    uint32_t coeffSelector;                       //!< [-] Index in the ephArray that we are currently using
    BSKLogger *bskLogger;                         //!< BSK Logging
};

#endif
