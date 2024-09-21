/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _ONE_AXIS_SOLAR_ARRAY_POINT_
#define _ONE_AXIS_SOLAR_ARRAY_POINT_

#include <stdint.h>
#include "architecture/utilities/bskLogging.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "msgPayloadDef/AttRefMsgPayload.h"
#include "msgPayloadDef/BodyHeadingMsgPayload.h"
#include "msgPayloadDef/InertialHeadingMsgPayload.h"
#include "msgPayloadDef/NavTransMsgPayload.h"
#include "msgPayloadDef/EphemerisMsgPayload.h"
#include "msgPayloadDef/NavAttMsgPayload.h"

typedef enum celestialBody{
    notSun = 0,
    Sun = 1
} CelestialBody;

typedef enum alignmentPriority{
    prioritizeAxisAlignment = 0,
    prioritizeSolarArrayAlignment = 1
} AlignmentPriority;

typedef enum bodyAxisInput{
    inputBodyHeadingParameter = 0,
    inputBodyHeadingMsg = 1
} BodyAxisInput;

typedef enum inertialAxisInput{
    inputInertialHeadingParameter = 0,
    inputInertialHeadingMsg = 1,
    inputEphemerisMsg = 2
} InertialAxisInput;

typedef enum refFrameSolution{
    determinate = 0,
    indeterminate = 1
} RefFrameSolution;

/*! @brief Top level structure for the sub-module routines. */
class OneAxisSolarArrayPoint : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;

    /*! declare these quantities that always must be specified as flight software parameters */
    double a1Hat_B[3];                           //!< arrays axis direction in B frame
    AlignmentPriority  alignmentPriority;        //!< flag to indicate which constraint must be prioritized

    /*! declare these optional quantities */
    double h1Hat_B[3];                           //!< main heading in B frame coordinates
    double h2Hat_B[3];                           //!< secondary heading in B frame coordinates
    double hHat_N[3];                            //!< main heading in N frame coordinates
    double a2Hat_B[3];                           //!< body frame heading that should remain as close as possible to Sun heading
    CelestialBody celestialBodyInput;

    /*! declare these internal variables that are used by the module and should not be declared by the user */
    BodyAxisInput      bodyAxisInput;            //!< flag variable to determine how the body axis input is specified
    InertialAxisInput  inertialAxisInput;        //!< flag variable to determine how the inertial axis input is specified
    int      updateCallCount;                    //!< count variable used in the finite difference logic
    uint64_t T1NanoSeconds;                      //!< callTime one update step prior
    uint64_t T2NanoSeconds;                      //!< callTime two update steps prior
    double   sigma_RN_1[3];                      //!< reference attitude one update step prior
    double   sigma_RN_2[3];                      //!< reference attitude two update steps prior
    ReadFunctor<NavAttMsgPayload>          attNavInMsg;             //!< input msg measured attitude
    ReadFunctor<BodyHeadingMsgPayload>     bodyHeadingInMsg;        //!< input body heading msg
    ReadFunctor<InertialHeadingMsgPayload> inertialHeadingInMsg;    //!< input inertial heading msg
    ReadFunctor<NavTransMsgPayload>        transNavInMsg;           //!< input msg measured position
    ReadFunctor<EphemerisMsgPayload>       ephemerisInMsg;          //!< input ephemeris msg
    Message<AttRefMsgPayload>          attRefOutMsg;            //!< output attitude reference message

    BSKLogger bskLogger{};                         //!< BSK Logging

};

#endif
