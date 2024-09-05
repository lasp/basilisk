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
/*
    FSW MODULE Template

 */

/* modify the path to reflect the new module names */
#include "fswAlgorithms/rwConfigData/rwConfigData.h"
#include "architecture/utilities/linearAlgebra.h"
#include <string.h>

/*
 Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory.
 */


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void RwConfig::Reset(uint64_t callTime)
{
    int i;
    RWConstellationMsgPayload rwConstellation = {}; /*!< struct to populate input RW config parameters in structural S frame */
    RWArrayConfigMsgPayload  rwConfigParamsOut = {}; /*!< struct to populate ouput RW config parameters in body B frame */
    if(this->rwConstellationInMsg.isLinked())
    {
        rwConstellation = this->rwConstellationInMsg();
    }
    rwConfigParamsOut.numRW = rwConstellation.numRW;

    for(i=0; i<rwConfigParamsOut.numRW; i=i+1)
    {
        rwConfigParamsOut.JsList[i] = rwConstellation.reactionWheels[i].Js;
        rwConfigParamsOut.uMax[i] = rwConstellation.reactionWheels[i].uMax;
        v3Copy(rwConstellation.reactionWheels[i].gsHat_B, &rwConfigParamsOut.GsMatrix_B[i*3]);
    }

    /*! - Write output RW config data to the messaging system*/
    this->rwParamsOutMsg.write(&rwConfigParamsOut, moduleID, callTime);

}
