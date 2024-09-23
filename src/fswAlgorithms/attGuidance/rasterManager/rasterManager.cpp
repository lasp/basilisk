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
 Inertial 3D Spin Module

 * University of Colorado, Autonomous Vehicle Systems (AVS) Lab
 * Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved

 */

#include "fswAlgorithms/attGuidance/rasterManager/rasterManager.h"
#include <stdio.h>

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "architecture/utilities/linearAlgebra.h"

void RasterManager::Reset(uint64_t callTime)
{
    this->mnvrActive = 0;
    this->scanSelector = 0;
}


void RasterManager::UpdateState(uint64_t callTime)
{
    double currentMnvrTime;
    this->scanSelector = this->scanSelector % this->numRasters;
    if (this->mnvrActive == 0)
    {
        this->mnvrStartTime = callTime;
        this->mnvrActive = 1;
    }
    currentMnvrTime = (callTime - this->mnvrStartTime) * 1E-9;
    if (currentMnvrTime < this->rasterTimes[this->scanSelector])
    {
        v3Copy(&this->scanningAngles[3 * this->scanSelector], this->attOutSet.state);
        v3Copy(&this->scanningRates[3 * this->scanSelector], this->attOutSet.rate);
    } else {
        this->mnvrActive = 0;
        this->scanSelector += 1;

        char info[MAX_LOGGING_LENGTH];
        snprintf(info, sizeof(info), "Raster: %i. AngleSet = [%f, %f, %f], RateSet = [%f, %f, %f] ", this->scanSelector,
               this->attOutSet.state[0],
               this->attOutSet.state[1],
               this->attOutSet.state[2],
               this->attOutSet.rate[0],
               this->attOutSet.rate[1],
               this->attOutSet.rate[2]);
        this->bskLogger.bskLog(BSK_INFORMATION, info);
    }

    this->attStateOutMsg.write(&this->attOutSet, this->moduleID, callTime);

    return;
}
