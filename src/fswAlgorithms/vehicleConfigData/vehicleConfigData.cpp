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

#include "fswAlgorithms/vehicleConfigData/vehicleConfigData.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"

void VehicleConfigData::Reset(uint64_t callTime)
{
    /*! - Zero the output message data */
    auto localConfigData = VehicleConfigMsgPayload();

    /*! - Copy over the center of mass location */
    v3Copy(this->CoM_B, localConfigData.CoM_B);

    /*! - Copy over the inertia */
    m33Copy(RECAST3X3 this->ISCPntB_B, RECAST3X3 localConfigData.ISCPntB_B);

    /*! - Copy over the mass */
    localConfigData.massSC = this->massSC;

    /*! - Write output properties to the messaging system*/
    this->vecConfigOutMsg.write(&localConfigData, this->moduleID, callTime);
}
