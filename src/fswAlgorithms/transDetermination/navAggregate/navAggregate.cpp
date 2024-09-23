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

#include "navAggregate.h"
#include "architecture/utilities/linearAlgebra.h"
#include <cstdio>


/*! This resets the module to original states.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void NavAggregate::Reset(uint64_t callTime)
{

    /*! - ensure incoming message counters are not larger than MAX_AGG_NAV_MSG */
    if (this->attMsgCount > MAX_AGG_NAV_MSG) {
        char info[MAX_LOGGING_LENGTH];
        snprintf(info, MAX_LOGGING_LENGTH, "The attitude message count %d is larger than allowed (%d). Setting count to max value.",
                  this->attMsgCount, MAX_AGG_NAV_MSG);
        this->bskLogger.bskLog(BSK_ERROR, info);

        this->attMsgCount = MAX_AGG_NAV_MSG;
    }
    if (this->transMsgCount > MAX_AGG_NAV_MSG) {
        char info[MAX_LOGGING_LENGTH];
        snprintf(info, sizeof(info), "The translation message count %d is larger than allowed (%d). Setting count to max value.",
                  this->transMsgCount, MAX_AGG_NAV_MSG);
        this->bskLogger.bskLog(BSK_ERROR, info);

        this->transMsgCount = MAX_AGG_NAV_MSG;
    }

    /*! - loop over the number of attitude input messages and make sure they are linked */
    for(uint32_t i=0; i<this->attMsgCount; i=i+1)
    {
        if (!this->attMsgs[i].navAttInMsg.isLinked()) {
            this->bskLogger.bskLog(BSK_ERROR, "An attitude input message name was not linked.  Be sure that attMsgCount is set properly.");
        }
    }
    /*! - loop over the number of translational input messages and make sure they are linked */
    for(uint32_t i=0; i<this->transMsgCount; i=i+1)
    {
        if (!this->transMsgs[i].navTransInMsg.isLinked()) {
            this->bskLogger.bskLog(BSK_ERROR, "A translation input message name was not specified.  Be sure that transMsgCount is set properly.");
        }
    }

    /*! - ensure the attitude message index locations are less than MAX_AGG_NAV_MSG */
    if (this->attTimeIdx >= MAX_AGG_NAV_MSG) {
        char info[MAX_LOGGING_LENGTH];
        snprintf(info, sizeof(info), "The attTimeIdx variable %d is too large. Must be less than %d. Setting index to max value.",
              this->attTimeIdx, MAX_AGG_NAV_MSG);
        this->bskLogger.bskLog(BSK_ERROR, info);

        this->attTimeIdx = MAX_AGG_NAV_MSG - 1;
    }
    if (this->attIdx >= MAX_AGG_NAV_MSG) {
        char info[MAX_LOGGING_LENGTH];
        snprintf(info, sizeof(info), "The attIdx variable %d is too large. Must be less than %d. Setting index to max value.",
                  this->attIdx, MAX_AGG_NAV_MSG);
        this->bskLogger.bskLog(BSK_ERROR, info);

        this->attIdx = MAX_AGG_NAV_MSG - 1;
    }
    if (this->rateIdx >= MAX_AGG_NAV_MSG) {
        char info[MAX_LOGGING_LENGTH];
        snprintf(info, sizeof(info), "The rateIdx variable %d is too large. Must be less than %d. Setting index to max value.",
                  this->rateIdx, MAX_AGG_NAV_MSG);
        this->bskLogger.bskLog(BSK_ERROR, info);

        this->rateIdx = MAX_AGG_NAV_MSG - 1;
    }
    if (this->sunIdx >= MAX_AGG_NAV_MSG) {
        char info[MAX_LOGGING_LENGTH];
        snprintf(info, sizeof(info), "The sunIdx variable %d is too large. Must be less than %d. Setting index to max value.",
                this->sunIdx, MAX_AGG_NAV_MSG);
        this->bskLogger.bskLog(BSK_ERROR, info);

        this->sunIdx = MAX_AGG_NAV_MSG - 1;
    }

    /*! - ensure the translational message index locations are less than MAX_AGG_NAV_MSG */
    if (this->transTimeIdx >= MAX_AGG_NAV_MSG) {
        char info[MAX_LOGGING_LENGTH];
        snprintf(info, sizeof(info), "The transTimeIdx variable %d is too large. Must be less than %d. Setting index to max value.",
                this->transTimeIdx, MAX_AGG_NAV_MSG);
        this->bskLogger.bskLog(BSK_ERROR, info);

        this->transTimeIdx = MAX_AGG_NAV_MSG - 1;
    }
    if (this->posIdx >= MAX_AGG_NAV_MSG) {
        char info[MAX_LOGGING_LENGTH];
        snprintf(info, sizeof(info), "The posIdx variable %d is too large. Must be less than %d. Setting index to max value.",
                  this->posIdx, MAX_AGG_NAV_MSG);
        this->bskLogger.bskLog(BSK_ERROR, info);

        this->posIdx = MAX_AGG_NAV_MSG - 1;
    }
    if (this->velIdx >= MAX_AGG_NAV_MSG) {
        char info[MAX_LOGGING_LENGTH];
        snprintf(info, sizeof(info), "The velIdx variable %d is too large. Must be less than %d. Setting index to max value.",
                  this->velIdx, MAX_AGG_NAV_MSG);
        this->bskLogger.bskLog(BSK_ERROR, info);

        this->velIdx = MAX_AGG_NAV_MSG - 1;
    }
    if (this->dvIdx >= MAX_AGG_NAV_MSG) {
        char info[MAX_LOGGING_LENGTH];
        snprintf(info, sizeof(info), "The dvIdx variable %d is too large. Must be less than %d. Setting index to max value.",
                this->dvIdx, MAX_AGG_NAV_MSG);
        this->bskLogger.bskLog(BSK_ERROR, info);

        this->dvIdx = MAX_AGG_NAV_MSG - 1;
    }

    //! - zero the arrays of input messages
    for (uint32_t i=0; i< MAX_AGG_NAV_MSG; i++) {
        this->attMsgs[i].msgStorage = NavAttMsgPayload();
        this->transMsgs[i].msgStorage = NavTransMsgPayload();
    }

}


/*! This method takes the navigation message snippets created by the various
    navigation components in the FSW and aggregates them into a single complete
    navigation message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void NavAggregate::UpdateState(uint64_t callTime)
{
    uint32_t i;
    NavAttMsgPayload navAttOutMsgBuffer = NavAttMsgPayload();     /* [-] The local storage of the outgoing attitude navibation message data*/
    NavTransMsgPayload navTransOutMsgBuffer = NavTransMsgPayload(); /* [-] The local storage of the outgoing message data*/

    /*! - check that attitude navigation messages are present */
    if (this->attMsgCount) {
        /*! - Iterate through all of the attitude input messages, clear local Msg buffer and archive the new nav data */
        for(i=0; i<this->attMsgCount; i=i+1)
        {
            this->attMsgs[i].msgStorage = this->attMsgs[i].navAttInMsg();
        }

        /*! - Copy out each part of the attitude source message into the target output message*/
        navAttOutMsgBuffer.timeTag = this->attMsgs[this->attTimeIdx].msgStorage.timeTag;
        v3Copy(this->attMsgs[this->attIdx].msgStorage.sigma_BN, navAttOutMsgBuffer.sigma_BN);
        v3Copy(this->attMsgs[this->rateIdx].msgStorage.omega_BN_B, navAttOutMsgBuffer.omega_BN_B);
        v3Copy(this->attMsgs[this->sunIdx].msgStorage.vehSunPntBdy, navAttOutMsgBuffer.vehSunPntBdy);

    }

    /*! - check that translation navigation messages are present */
    if (this->transMsgCount) {
        /*! - Iterate through all of the translation input messages, clear local Msg buffer and archive the new nav data */
        for(i=0; i<this->transMsgCount; i=i+1)
        {
            this->transMsgs[i].msgStorage = this->transMsgs[i].navTransInMsg();
        }

        /*! - Copy out each part of the translation source message into the target output message*/
        navTransOutMsgBuffer.timeTag = this->transMsgs[this->transTimeIdx].msgStorage.timeTag;
        v3Copy(this->transMsgs[this->posIdx].msgStorage.r_BN_N, navTransOutMsgBuffer.r_BN_N);
        v3Copy(this->transMsgs[this->velIdx].msgStorage.v_BN_N, navTransOutMsgBuffer.v_BN_N);
        v3Copy(this->transMsgs[this->dvIdx].msgStorage.vehAccumDV, navTransOutMsgBuffer.vehAccumDV);
    }

    /*! - Write the total message out for everyone else to pick up */
    this->navAttOutMsg.write(&navAttOutMsgBuffer, this->moduleID, callTime);
    this->navTransOutMsg.write(&navTransOutMsgBuffer, this->moduleID, callTime);
}
