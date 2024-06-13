/*
 ISC License

 Copyright (c) 2024, Laboratory For Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef _TWOAXISGIMBALCONTROLLER_
#define _TWOAXISGIMBALCONTROLLER_

#include <stdint.h>
#include <Eigen/Core>
#include "architecture/utilities/bskLogging.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "cMsgCInterface/TwoAxisGimbalMsg_C.h"
#include "cMsgCInterface/HingedRigidBodyMsg_C.h"

const double DEG2RAD = M_PI / 180.0;

/*! @brief Two-Axis Gimbal Controller Class. */
class TwoAxisGimbalController: public SysModel {
public:

    TwoAxisGimbalController(std::string pathToMotor1Table, std::string pathToMotor2Table);       //!< Constructor
    ~TwoAxisGimbalController() = default;                                                        //!< Destructor
    void Reset(uint64_t CurrentSimNanos) override;                                               //!< Reset member function
    void UpdateState(uint64_t CurrentSimNanos) override;                                         //!< Update member function
    void loadGimbalToMotor1AngleLookupTable(std::string pathToMotor1Table);                      //!< Method to load the gimbal-to-motor 1 angle lookup table
    void loadGimbalToMotor2AngleLookupTable(std::string pathToMotor2Table);                      //!< Method to load the gimbal-to-motor 2 angle lookup table

    BSKLogger* bskLogger;                                                                        //!< BSK Logging

    ReadFunctor<TwoAxisGimbalMsgPayload> twoAxisGimbalInMsg;                                     //!< Input msg for the gimbal tip and tilt angles
    Message<HingedRigidBodyMsgPayload> motor1AngleOutMsg;                                        //!< Output message for the motor 1 angle
    Message<HingedRigidBodyMsgPayload> motor2AngleOutMsg;                                        //!< Output message for the motor 1 angle

private:

    void gimbalAnglesToMotorAngles();
    bool bilinearInterpolationRequired();
    void bilinearlyInterpolateMotorAngles();
    bool noInterpolationRequired();
    bool linearInterpolationRequired(double gimbalAngle);
    void linearlyInterpolateMotorAnglesTipAngleFixed();
    void linearlyInterpolateMotorAnglesTiltAngleFixed();
    double pullMotor1Angle(double gimbalTipAngle,
                           double gimbalTiltAngle);                                              //!< Method used to pull a specific motor 1 angle from the motor 1 interpolation table given specific gimbal angles
    double pullMotor2Angle(double gimbalTipAngle,
                           double gimbalTiltAngle);                                              //!< Method used to pull a specific motor 2 angle from the motor 2 interpolation table given specific gimbal angles

    /* Gimbal parameters */
    double gimbalTipAngleRef{};                                                                  //!< [rad] Gimbal tip reference angle
    double gimbalTiltAngleRef{};                                                                 //!< [rad] Gimbal tilt reference angle

    /* Motor parameters */
    double motor1Angle{};                                                                        //!< [rad] Motor 1 angle
    double motor2Angle{};                                                                        //!< [rad] Motor 2 angle

    /* Interpolation table parameters */
    double tableStepAngle{0.5 * DEG2RAD};                                                        //!< [rad] Interpolation table gimbal discretization angle
    double gimbal_to_motor_1_angle[109][74];                                                     //!< [rad] Gimbal-to-motor 1 angle interpolation table storage array
    double gimbal_to_motor_2_angle[109][74];                                                     //!< [rad] Gimbal-to-motor 2 angle interpolation table storage array

};

#endif /* TWOAXISGIMBALCONTROLLER */
