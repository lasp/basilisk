/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef _TWOAXISGIMBAL_
#define _TWOAXISGIMBAL_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "cMsgCInterface/HingedRigidBodyMsg_C.h"
#include "cMsgCInterface/MotorStepCommandMsg_C.h"
#include "cMsgCInterface/StepperMotorMsg_C.h"
#include "cMsgCInterface/TwoAxisGimbalMsg_C.h"
#include "cMsgCInterface/PrescribedRotationMsg_C.h"
#include <Eigen/Dense>

/*! @brief Two Axis Gimbal Class */
class TwoAxisGimbal: public SysModel {
public:
    TwoAxisGimbal() = default;                                                      //!< Constructor
    ~TwoAxisGimbal() = default;                                                     //!< Destructor

    void Reset(uint64_t CurrentSimNanos) override;                                  //!< Reset member function
    void UpdateState(uint64_t CurrentSimNanos) override;                            //!< Update member function

    void setGimbalRotHat1_M(const Eigen::Vector3d &rotHat1_M);                      //!< Setter for the gimbal rotation axis 1
    void setGimbalRotHat2_F(const Eigen::Vector3d &rotHat2_F);                      //!< Setter for the gimbal rotation axis 2
    void setMotorStepAngle(const double stepAngle);                                 //!< Setter method for the motor step angle
    void setMotorStepTime(const double stepTime);                                   //!< Setter method for the motor step time
    const Eigen::Vector3d &getGimbalRotHat1_M() const;                              //!< Getter for the gimbal rotation axis 1
    const Eigen::Vector3d &getGimbalRotHat2_F() const;                              //!< Getter for the gimbal rotation axis 2
    double getMotorStepAngle() const;                                               //!< Getter method for the motor step angle
    double getMotorStepTime() const;                                                //!< Getter method for the motor step time

    ReadFunctor<HingedRigidBodyMsgPayload> motor1InitStateInMsg;                    //!< Input msg for stepper motor 1 initial state
    ReadFunctor<HingedRigidBodyMsgPayload> motor2InitStateInMsg;                    //!< Input msg for stepper motor 2 initial state
    ReadFunctor<MotorStepCommandMsgPayload> motor1StepCmdInMsg;                     //!< Input msg for stepper motor 1 steps commanded
    ReadFunctor<MotorStepCommandMsgPayload> motor2StepCmdInMsg;                     //!< Input msg for stepper motor 2 steps commanded
    ReadFunctor<StepperMotorMsgPayload> motor1StateInMsg;                           //!< Input msg for stepper motor 1 state information
    ReadFunctor<StepperMotorMsgPayload> motor2StateInMsg;                           //!< Input msg for stepper motor 2 state information
    Message<TwoAxisGimbalMsgPayload> twoAxisGimbalOutMsg;                           //!< Output msg for the gimbal tip and tilt angles
    Message<PrescribedRotationMsgPayload> prescribedRotationOutMsg;                 //!< Output msg for the hub-relative prescribed rotational states

    BSKLogger *bskLogger;                                                           //!< BSK Logging

private:
    // Stepper motor state data
    double motorStepAngle;                                                          //!< [rad] Angle the stepper motor moves through for a single step
    double motorStepTime;                                                           //!< [s] Time required for a single motor step (constant)
    double motor1ThetaInit;                                                         //!< [rad] Initial stepper motor 1 angle
    double motor2ThetaInit;                                                         //!< [rad] Initial stepper motor 2 angle
    int motor1StepsCommanded;
    int motor2StepsCommanded;
    double motor1ThetaRef;
    double motor2ThetaRef;

    // Gimbal state data
    Eigen::Vector3d gimbalPRV_F0M;
    Eigen::Vector3d gimbalPRVRotHat;                                                //!< Gimbal prv Eigen axis of rotation
    Eigen::Vector3d gimbalRotHat1_M;                                                //!< Gimbal tip angle axis of rotation 1 expressed in M frame components
    Eigen::Vector3d gimbalRotHat2_F;                                                //!< Gimbal tilt angle axis of rotation 2 expressed in F frame components
    int gimbalStepCount;
    int gimbalStepsCommanded;
    double gimbalStepAngle;
    double gimbalPRVThetaDDotMax;                                                   //!< [rad/s^2] Current profiled gimbal prv angular acceleration
    double gimbalPRVThetaRef;
    double intermediateGimbalPRVThetaInit;
    double intermediateGimbalPRVThetaRef;
    double gimbalPRVTheta;                                                          //!< [rad] Current profiled gimbal prv angle
    double gimbalPRVThetaDot;                                                       //!< [rad/s] Current profiled gimbal prv angle rate
    double gimbalPRVThetaDDot;                                                      //!< [rad/s^2] Current profiled gimbal prv angular acceleration
    double gimbalTheta1;                                                            //!< [rad] Current gimbal tip angle
    double gimbalTheta2;                                                            //!< [rad] Current gimbal tilt angle

    /* Temporal parameters */
    double previousWrittenTime;                                                     //!< [ns] Time the last input message was written
    double tInit;                                                                   //!< [s] Simulation time at the beginning of the maneuver
    double ts;                                                                      //!< [s] The simulation time halfway through the maneuver (switch time for ang accel)
    double tf;                                                                      //!< [s] Simulation time when the maneuver is finished

    /* Boolean parameters */
    bool segment1Complete;
    bool segment2Complete;
    bool completion;                                                                //!< Boolean designating a fully completed maneuver
    bool gimbalStepComplete;                                                        //!< Boolean designating a completed step
    bool newMsg;                                                                    //!< Boolean designating a new message was written

    /* Constant parameters */
    double a;                                                                       //!< Parabolic constant for the first half of a step
    double b;                                                                       //!< Parabolic constant for the second half of a step
};

#endif
