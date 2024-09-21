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

#ifndef _THRUSTER_FORCE_MAPPING_H_
#define _THRUSTER_FORCE_MAPPING_H_

#include <stdint.h>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "msgPayloadDef/THRArrayConfigMsgPayload.h"
#include "msgPayloadDef/VehicleConfigMsgPayload.h"
#include "msgPayloadDef/THRArrayCmdForceMsgPayload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgPayload.h"

#include "architecture/utilities/bskLogging.h"


/*!@brief Data structure for module to map a command torque onto thruster forces. */
class ThrForceMapping : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;
    void findMinimumNormForce(double D[3][MAX_EFF_CNT],
                              double Lr_B[3],
                              uint32_t numForces,
                              double F[MAX_EFF_CNT]);

    /* declare module public variables */
    double   controlAxes_B[3*3];                    //!< []      array of the control unit axes
    double   rThruster_B[MAX_EFF_CNT][3];           //!< [m]     local copy of the thruster locations
    double   gtThruster_B[MAX_EFF_CNT][3];          //!< []      local copy of the thruster force unit direction vectors
    int32_t  thrForceSign;                          //!< []      Flag indicating if pos (+1) or negative (-1) thruster solutions are found
    double angErrThresh;                            //!< [r]     Angular error at which thruster forces are scaled to not be super-saturated
    double   epsilon;                               //!<         variable specifying what is considered a small number
    uint32_t use2ndLoop;                            //!< []      flag indicating if the 2nd least squares fitting loop should be used (1) or not used (0 - default)

    /* declare module private variables */
    uint32_t numControlAxes;                        //!< []      counter indicating how many orthogonal axes are controlled
    uint32_t numThrusters;                          //!< []      The number of thrusters available on vehicle
    double outTorqAngErr;                           //!< [r]     Angular error of effector torque
    double thrForcMag[MAX_EFF_CNT];                 //!<         vector of thruster force magnitudes

    /* declare module IO interfaces */
    Message<THRArrayCmdForceMsgPayload> thrForceCmdOutMsg;        //!< The name of the output thruster force message
    ReadFunctor<CmdTorqueBodyMsgPayload> cmdTorqueInMsg;              //!< The name of the vehicle control (Lr) Input message
    ReadFunctor<THRArrayConfigMsgPayload> thrConfigInMsg;             //!< The name of the thruster cluster Input message
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;              //!< The name of the Input message
    VehicleConfigMsgPayload   sc;                   //!< spacecraft configuration message

    BSKLogger bskLogger={};                             //!< BSK Logging

};


#endif
