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

#include "twoAxisGimbal.h"

/*! This method checks the input messages to ensure they are linked.
 @return void
 @param callTime [ns] Time the method is called
*/
void TwoAxisGimbal::Reset(uint64_t callTime) {
    if (!this->motor1InitStateInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "twoAxisGimbal.motor1InitStateInMsg wasn't connected.");
    }
    if (!this->motor2InitStateInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "twoAxisGimbal.motor2InitStateInMsg wasn't connected.");
    }
    if (!this->motor1StepCmdInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "twoAxisGimbal.motor1StepCmdInMsg wasn't connected.");
    }
    if (!this->motor2StepCmdInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "twoAxisGimbal.motor2StepCmdInMsg wasn't connected.");
    }
    if (!this->motor1StateInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "twoAxisGimbal.motor1StateInMsg wasn't connected.");
    }
    if (!this->motor2StateInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "twoAxisGimbal.motor2StateInMsg wasn't connected.");
    }
}

/*! This method determines the two-axis gimbal tip and tilt angles and profiles its prescribed hub-relative rotational
states using the two stepper motor input state messages. The gimbal tip and tilt angles and the hub-relative prescribed
rotational states are then written to the output messages.
 @return void
 @param callTime [ns] Time the method is called
*/
void TwoAxisGimbal::UpdateState(uint64_t callTime) {
}
