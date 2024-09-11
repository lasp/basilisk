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

#ifndef _THR_FIRING_REMAINDER_
#define _THR_FIRING_REMAINDER_

#include <stdint.h>
#include "fswAlgorithms/fswUtilities/fswDefinitions.h"

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/THRArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayCmdForceMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayOnTimeCmdMsgPayload.h"

#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/bskLogging.h"



/*! @brief Top level structure for the sub-module routines. */
class ThrFiringRemainder : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;
	double              pulseRemainder[MAX_EFF_CNT];            //!< [-] Unimplemented thrust pulses (number of minimum pulses)
	double              thrMinFireTime;              			//!< [s] Minimum fire time
	int      			numThrusters;							//!< [-] The number of thrusters available on vehicle
	double				maxThrust[MAX_EFF_CNT];					//!< [N] Max thrust
	int					baseThrustState;						//!< [-] Indicates on-pulsing (0) or off-pulsing (1)
	double              defaultControlPeriod;                   //!< [s] Default control period used for first call

	uint64_t			prevCallTime;							//!< callTime from previous function call


	/* declare module IO interfaces */
    ReadFunctor<THRArrayCmdForceMsgPayload> thrForceInMsg;        	            //!< The name of the Input message
    Message<THRArrayOnTimeCmdMsgPayload> onTimeOutMsg;       	                //!< The name of the output message, onTimeOutMsg
    ReadFunctor<THRArrayConfigMsgPayload> thrConfInMsg;			                //!< The name of the thruster cluster Input message
	BSKLogger bskLogger={};                             //!< BSK Logging

};

#endif
