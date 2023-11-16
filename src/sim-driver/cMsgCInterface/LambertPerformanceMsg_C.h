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

/* All of the files in this folder (dist3/autoSource) are autocoded by the script
architecture/messaging/msgAutoSource/GenCMessages.py.
The script checks for the line "INSTANTIATE_TEMPLATES" in the file architecture/messaging/messaging.i. This
ensures that if a c++ message is instantiated that we also have a C equivalent of that message.
*/

#ifndef LambertPerformanceMsg_C_H
#define LambertPerformanceMsg_C_H

#include <stdint.h>
#include "architecture/msgPayloadDefC/LambertPerformanceMsgPayload.h"
#include "architecture/messaging/msgHeader.h"

//! structure definition
typedef struct {
    MsgHeader header;              //!< message header, zero'd on construction
    LambertPerformanceMsgPayload payload;		        //!< message copy, zero'd on construction
    LambertPerformanceMsgPayload *payloadPointer;	    //!< pointer to message
    MsgHeader *headerPointer;      //!< pointer to message header
} LambertPerformanceMsg_C;

#ifdef __cplusplus
extern "C" {
#endif

void LambertPerformanceMsg_cpp_subscribe(LambertPerformanceMsg_C *subscriber, void* source);

void LambertPerformanceMsg_C_subscribe(LambertPerformanceMsg_C *subscriber, LambertPerformanceMsg_C *source);

int8_t LambertPerformanceMsg_C_isSubscribedTo(LambertPerformanceMsg_C *subscriber, LambertPerformanceMsg_C *source);
int8_t LambertPerformanceMsg_cpp_isSubscribedTo(LambertPerformanceMsg_C *subscriber, void* source);

void LambertPerformanceMsg_C_addAuthor(LambertPerformanceMsg_C *coowner, LambertPerformanceMsg_C *data);

void LambertPerformanceMsg_C_init(LambertPerformanceMsg_C *owner);

int LambertPerformanceMsg_C_isLinked(LambertPerformanceMsg_C *data);

int LambertPerformanceMsg_C_isWritten(LambertPerformanceMsg_C *data);

uint64_t LambertPerformanceMsg_C_timeWritten(LambertPerformanceMsg_C *data);

int64_t LambertPerformanceMsg_C_moduleID(LambertPerformanceMsg_C *data);

void LambertPerformanceMsg_C_write(LambertPerformanceMsgPayload *data, LambertPerformanceMsg_C *destination, int64_t moduleID, uint64_t callTime);

LambertPerformanceMsgPayload LambertPerformanceMsg_C_read(LambertPerformanceMsg_C *source);

LambertPerformanceMsgPayload LambertPerformanceMsg_C_zeroMsgPayload();

#ifdef __cplusplus
}
#endif
#endif