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
#include "cMsgCInterface/MotorStepCommandMsg_C.h"
#include "cMsgCInterface/StepperMotorMsg_C.h"
#include "cMsgCInterface/TwoAxisGimbalMsg_C.h"
#include "cMsgCInterface/PrescribedRotationMsg_C.h"
#include <Eigen/Dense>

const double DEG2RAD = M_PI / 180.0;

/*! @brief Two Axis Gimbal Class */
class TwoAxisGimbal: public SysModel {
public:

    TwoAxisGimbal(std::string pathToTipTable, std::string pathToTiltTable);         //!< Constructor
    ~TwoAxisGimbal() = default;                                                     //!< Destructor
    void Reset(uint64_t CurrentSimNanos) override;                                  //!< Reset member function
    void UpdateState(uint64_t CurrentSimNanos) override;                            //!< Update member function
    void setMotorStepAngle(const double stepAngle);                                 //!< Setter method for the motor step angle
    void setMotorStepTime(const double stepTime);                                   //!< Setter method for the motor step time
    const Eigen::Vector3d &getGimbalRotHat1_M() const;                              //!< Getter for the gimbal rotation axis 1
    const Eigen::Vector3d &getGimbalRotHat2_F() const;                              //!< Getter for the gimbal rotation axis 2
    double getMotorStepAngle() const;                                               //!< Getter method for the motor step angle
    double getMotorStepTime() const;                                                //!< Getter method for the motor step time
    void loadMotorToGimbalTipAngleLookupTable(std::string pathToTipTable);          //!< Method to load the motor-to-gimbal tip angle lookup table
    void loadMotorToGimbalTiltAngleLookupTable(std::string pathToTiltTable);        //!< Method to load the motor-to-gimbal tilt angle lookup table

    ReadFunctor<MotorStepCommandMsgPayload> motor1StepCmdInMsg;                     //!< Input msg for stepper motor 1 steps commanded
    ReadFunctor<MotorStepCommandMsgPayload> motor2StepCmdInMsg;                     //!< Input msg for stepper motor 2 steps commanded
    ReadFunctor<StepperMotorMsgPayload> motor1StateInMsg;                           //!< Input msg for stepper motor 1 state information
    ReadFunctor<StepperMotorMsgPayload> motor2StateInMsg;                           //!< Input msg for stepper motor 2 state information
    Message<TwoAxisGimbalMsgPayload> twoAxisGimbalOutMsg;                           //!< Output msg for the gimbal tip and tilt angles
    Message<PrescribedRotationMsgPayload> prescribedRotationOutMsg;                 //!< Output msg for the hub-relative prescribed rotational states

    BSKLogger *bskLogger;                                                           //!< BSK Logging

private:

    Eigen::Vector3d motorAnglesToGimbalPRV(double motor1Angle,
                                           double motor2Angle);                     //!< Function to determine the gimbal PRV attitude given the stepper motor angles
    std::pair<double, double> motorAnglesToGimbalAngles(double motor1Angle,
                                                        double motor2Angle);        //!< Function to determine the sequential gimbal tip and tilt angles given the stepper motor angles
    void computeGimbalActuationParameters();                                        //!< Function used to compute and update the gimbal actuation parameters for each segment of required gimbal motion
    double pullGimbalTipAngle(double motor1Angle,
                              double motor2Angle);                                  //!< Function used to pull a specific gimbal tip angle from the tip interpolation table given specific motor angles
    double pullGimbalTiltAngle(double motor1Angle,
                               double motor2Angle);                                 //!< Function used to pull a specific gimbal tilt angle from the tilt interpolation table given specific motor angles
    void actuateGimbal(double t);                                                   //!< High-level method used to simulate the gimbal states in time
    void resetGimbal(double t);                                                     //!< Method used to reset the gimbal states when the current request is complete and a new request is received
    void updateGimbalRotationParameters();                                          //!< Method used to update the gimbal rotation parameters after a step is completed
    bool isInGimbalStepFirstHalf(double t);                                         //!< Method used to determine if the gimbal is in the first half of a step
    void computeGimbalStepFirstHalf(double t);                                      //!< Method used to compute the gimbal states during the first half of each step
    bool isInGimbalStepSecondHalf(double t);                                        //!< Method used to determine if the gimbal is in the second half of a step
    void computeGimbalStepSecondHalf(double t);                                     //!< Method used to compute the gimbal states during the second half of each step
    void computeGimbalStepComplete(double t);                                       //!< Method used to compute the gimbal states when a step is complete
    void writeOutputMessages(uint64_t CurrentSimNanos);                             //!< Method for writing the module output messages and computing the output message data

    // Stepper motor state data
    double motorStepAngle{1.0 * DEG2RAD};                                           //!< [rad] Angle the stepper motor moves through for a single step
    double motorStepTime{0.1};                                                      //!< [s] Time required for a single motor step (constant)
    double motor1ThetaInit{};                                                       //!< [rad] Initial stepper motor 1 angle
    double motor2ThetaInit{};                                                       //!< [rad] Initial stepper motor 2 angle
    int motor1StepsCommanded{};                                                     //!< Number of steps commanded for stepper motor 1
    int motor2StepsCommanded{};                                                     //!< Number of steps commanded for stepper motor 2
    double motor1ThetaRef{};                                                        //!< [rad] Motor 1 reference angle
    double motor2ThetaRef{};                                                        //!< [rad] Motor 2 reference angle

    // Gimbal state data
    Eigen::Vector3d gimbalPRV_F0M{};                                                //!< Initial hub-relative gimbal prv attitude
    Eigen::Vector3d gimbalPRV_FIntM{};                                              //!< Intermediate hub-relative gimbal prv attitude
    Eigen::Vector3d gimbalPRVRotHat{};                                              //!< Gimbal prv Eigen axis of rotation
    Eigen::Vector3d gimbalRotHat1_M{1.0, 0.0, 0.0};                      //!< Gimbal tip angle axis of rotation 1 expressed in M frame components
    Eigen::Vector3d gimbalRotHat2_F{0.0, 1.0, 0.0};                      //!< Gimbal tilt angle axis of rotation 2 expressed in F frame components
    int gimbalStepCount{};                                                          //!< Current number of gimbal steps taken
    int gimbalStepsCommanded{};                                                     //!< Number of gimbal steps commanded
    double gimbalStepAngle{};                                                       //!< [rad] Angle the gimbal moves through for a single step
    double gimbalPRVThetaDDotMax{};                                                 //!< [rad/s^2] Current profiled gimbal prv angular acceleration
    double gimbalPRVThetaRef{};                                                     //!< [rad] Reference PRV gimbal angle
    double intermediateGimbalPRVThetaInit{};                                        //!< [rad] Intermediate initial gimbal PRV angle
    double intermediateGimbalPRVThetaRef{};                                         //!< [rad] Intermediate gimbal PRV reference angle
    double gimbalPRVTheta{};                                                        //!< [rad] Current profiled gimbal prv angle
    double gimbalPRVThetaDot{};                                                     //!< [rad/s] Current profiled gimbal prv angle rate
    double gimbalPRVThetaDDot{};                                                    //!< [rad/s^2] Current profiled gimbal prv angular acceleration

    /* Temporal parameters */
    double previousWrittenTime{-1};                                                 //!< [ns] Time the last input message was written
    double tInit{};                                                                 //!< [s] Simulation time at the beginning of the maneuver
    double ts{};                                                                    //!< [s] The simulation time halfway through the maneuver (switch time for ang accel)
    double tf{};                                                                    //!< [s] Simulation time when the maneuver is finished

    /* Boolean parameters */
    bool newMsg{};                                                                  //!< Boolean designating a new gimbal actuation command
    bool segment1Complete{};                                                        //!< Boolean designating completion of the first actuation segment
    bool segment2Complete{};                                                        //!< Boolean designating completion of the second actuation segment
    bool gimbalStepComplete{true};                                                  //!< Boolean designating a completed gimbal step
    bool completion{true};                                                          //!< Boolean designating completion of the full gimbal actuation (Both segment 1 and segment 2 are complete)

    /* Constant parameters */
    double a{};                                                                     //!< Parabolic constant for the first half of a gimbal step
    double b{};                                                                     //!< Parabolic constant for the second half of a gimbal step

    /* Interpolation table parameters */
    double tableStepAngle{0.5 * DEG2RAD};                                           //!< [rad] Interpolation table motor discretization angle
    double motor_to_gimbal_tip_angle[319][319];                                     //!< [rad] Motor-to-gimbal tip angle interpolation table storage array
    double motor_to_gimbal_tilt_angle[319][319];                                    //!< [rad] Motor-to-gimbal tilt angle interpolation table storage array
};

#endif
