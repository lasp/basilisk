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

/* Import the module header file */
#include "sepGimbal.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Other required files to import */
#include <stdbool.h>
#define _USE_MATH_DEFINES 
#include <math.h>
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"

/*! This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_sepGimbal(SepGimbalConfig* configData, int64_t moduleID)
{
    // Initialize the output messages
    HingedRigidBodyMsg_C_init(&configData->desiredGimbalTipAngleOutMsg);
    HingedRigidBodyMsg_C_init(&configData->desiredGimbalTiltAngleOutMsg);
}

/*! This method performs a complete reset of the module. The input messages are checked to ensure they are linked.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/

void Reset_sepGimbal(SepGimbalConfig* configData, uint64_t callTime, int64_t moduleID)
{
    // Check if the required input message is linked
    if (!BodyHeadingMsg_C_isLinked(&configData->desiredThrustMountFrameInMsg))
    {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: sepGimbal.desiredThrustMountFrameInMsg wasn't connected.");
    }

    // Initialise parameters:

    // Set the initial desired thrust unit vector to zero
    configData->desiredThrustUnitVector[0] = 0.0;
    configData->desiredThrustUnitVector[1] = 0.0;
    configData->desiredThrustUnitVector[2] = 0.0;

    // Set the initial tip angle to zero
    configData->Tip_angle = 0.0;

    // Set the initial tilt angle to zero
    configData->Tilt_angle = 0.0;

    // Set the initial time were desired theta message was given. It is assigned to be -1 because we don't want its condition to overlap with the call time as we can recieve a message at call time 0.0, where call time is an integer that can't be negative
    configData->previousWrittenTime = 0.0;
    configData->firstCall = true;
}


/*! This method recieves the desired platform tip and tilt angles and outputs the desired motor1 and motor2 angles.
The desired motor1 and motor2 angles will be found using the lookup tables where each of them has a corresponding platform tip and tilt angles.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] The current time of simulation
 @param moduleID The module identifier
*/

void Update_sepGimbal(SepGimbalConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // Create the buffer messages
    HingedRigidBodyMsgPayload desiredGimbalTipAngleOut;
    HingedRigidBodyMsgPayload desiredGimbalTiltAngleOut;
    BodyHeadingMsgPayload desiredThrustMountFrameIn;

    // Zero the output messages
    desiredGimbalTipAngleOut = HingedRigidBodyMsg_C_zeroMsgPayload();
    desiredGimbalTiltAngleOut = HingedRigidBodyMsg_C_zeroMsgPayload();

    // Read the input message
    desiredThrustMountFrameIn = BodyHeadingMsg_C_zeroMsgPayload();
    if (BodyHeadingMsg_C_isWritten(&configData->desiredThrustMountFrameInMsg)){
        desiredThrustMountFrameIn = BodyHeadingMsg_C_read(&configData->desiredThrustMountFrameInMsg);
        printf("\n desiredThrustMountFrameIn.rHat_XB_B: ");

    for (int i = 0; i < 3; i++) {
        printf("%f ", desiredThrustMountFrameIn.rHat_XB_B[i]);
    }
    printf("\n");
    }

    printf("Tip agnle is:  %.2f\n", configData->Tip_angle);
    printf("Tilt agnle is:  %.2f\n", configData->Tilt_angle);

    double bodyHeadingMsgTimeWritten = NANO2SEC * BodyHeadingMsg_C_timeWritten(&configData->desiredThrustMountFrameInMsg);

        // Check if we have a new message of desired angle to excute the number of steps commanded (no interaption) 
    if (configData->previousWrittenTime <  bodyHeadingMsgTimeWritten || configData->firstCall){   
        configData->firstCall = false;

        // Assign the previous time to be the new written time
        configData->previousWrittenTime = bodyHeadingMsgTimeWritten;

        // Assign the input thrust unit vector as the desired thrust unit vector
        v3Copy(desiredThrustMountFrameIn.rHat_XB_B, configData->desiredThrustUnitVector);
        printf("\n In loop: desiredThrustMountFrameIn %f", desiredThrustMountFrameIn.rHat_XB_B[0]);
        printf("\n In loop: desiredThrustMountFrameIn %f", desiredThrustMountFrameIn.rHat_XB_B[1]);
        printf("\n In loop: desiredThrustMountFrameIn %f", desiredThrustMountFrameIn.rHat_XB_B[2]);
    }

    configData->Tip_angle = asin(desiredThrustMountFrameIn.rHat_XB_B[0]);
    
    configData->Tilt_angle = -atan(desiredThrustMountFrameIn.rHat_XB_B[1] /desiredThrustMountFrameIn.rHat_XB_B[2] );

    printf("\nTip agnle is:  %.2f\n", configData->Tip_angle);
    printf("Tilt agnle is:  %.2f\n", configData->Tilt_angle);


    // Output the desired tip angle 
    desiredGimbalTipAngleOut.theta = (int)configData->Tip_angle; 
    HingedRigidBodyMsg_C_write(&desiredGimbalTipAngleOut, &configData->desiredGimbalTipAngleOutMsg, moduleID, callTime);

    // Output the desired tilt angle 
    desiredGimbalTiltAngleOut.theta = (int)configData->Tilt_angle;
    HingedRigidBodyMsg_C_write(&desiredGimbalTiltAngleOut, &configData->desiredGimbalTiltAngleOutMsg, moduleID, callTime);
}    
