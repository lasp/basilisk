/*
 ISC License

 Copyright (c) 2024, Laboratory of Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "fswAlgorithms/attControl/thrMomentumManagementCpp/thrMomentumManagementCpp.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <Eigen/Core>

void ThrMomentumManagementCpp::Reset(uint64_t currentSimNanos)
{
    // Check if the required input messages are included
    if (!this->rwConfigDataInMsg.isLinked()) {
        this->bskLogger->bskLog(BSK_ERROR, "thrMomentumManagementCpp.rwConfigDataInMsg wasn't connected.");
    }
    if (!this->rwSpeedsInMsg.isLinked()) {
        this->bskLogger->bskLog(BSK_ERROR, "thrMomentumManagementCpp.rwSpeedsInMsg wasn't connected.");
    }

    // Read in the RW configuration message
    this->rwConfigParams = this->rwConfigDataInMsg();

    // Reset the momentum dumping request flag
    this->initRequest = 1;
}

void ThrMomentumManagementCpp::UpdateState(uint64_t currentSimNanos)
{
    if (this->initRequest == 1) {
        Eigen::Vector3d hs_B = Eigen::Vector3d::Zero();
        RWSpeedMsgPayload rwSpeedMsg = this->rwSpeedsInMsg();
        for (int i=0; i<this->rwConfigParams.numRW; i++) {
            hs_B += this->rwConfigParams.JsList[i] * rwSpeedMsg.wheelSpeeds[i] * cArray2EigenVector3d(&this->rwConfigParams.GsMatrix_B[i * 3]);
        }

        Eigen::Vector3d Delta_H_B = Eigen::Vector3d::Zero();
        if (double hs = hs_B.norm(); hs >= this->hs_min) {
            Delta_H_B = - hs_B * (hs - this->hs_min) / hs;
        }

        CmdTorqueBodyMsgPayload controlOutMsg = this->deltaHOutMsg.zeroMsgPayload;
        eigenVector3d2CArray(Delta_H_B, controlOutMsg.torqueRequestBody);
        this->deltaHOutMsg.write(&controlOutMsg, this->moduleID, currentSimNanos);

        this->initRequest = 0;
    }
}
