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

#ifndef VEHICLECONFIGDATACPP_H
#define VEHICLECONFIGDATACPP_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "msgPayloadDef/VehicleConfigMsgPayload.h"
#include "architecture/utilities/macroDefinitions.h"

class VehicleConfigData : public SysModel {
public:
    void Reset(uint64_t callTime) override;

    double ISCPntB_B[9];          /*!< [kg m^2] Spacecraft Inertia */
    double CoM_B[3];              /*!< [m] Center of mass of spacecraft in body*/
    double massSC;                /*!< [kg] Spacecraft mass */
    Message<VehicleConfigMsgPayload> vecConfigOutMsg; /*!< [-] Name of the output properties message*/

    BSKLogger *bskLogger;                             //!< BSK Logging
};


#endif //VEHICLECONFIGDATACPP_H
