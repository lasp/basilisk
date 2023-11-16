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

#ifndef LambertProblemMsg_C_H
#define LambertProblemMsg_C_H

#include <stdint.h>
#include "architecture/msgPayloadDefC/LambertProblemMsgPayload.h"
#include "architecture/messaging/msgHeader.h"

//! structure definition
typedef struct {
    MsgHeader header;              //!< message header, zero'd on construction
    LambertProblemMsgPayload payload;		        //!< message copy, zero'd on construction
    LambertProblemMsgPayload *payloadPointer;	    //!< pointer to message
    MsgHeader *headerPointer;      //!< pointer to message header
} LambertProblemMsg_C;

#ifdef __cplusplus
extern "C" {
#endif

void LambertProblemMsg_cpp_subscribe(LambertProblemMsg_C *subscriber, void* source);

void LambertProblemMsg_C_subscribe(LambertProblemMsg_C *subscriber, LambertProblemMsg_C *source);

int8_t LambertProblemMsg_C_isSubscribedTo(LambertProblemMsg_C *subscriber, LambertProblemMsg_C *source);
int8_t LambertProblemMsg_cpp_isSubscribedTo(LambertProblemMsg_C *subscriber, void* source);

void LambertProblemMsg_C_addAuthor(LambertProblemMsg_C *coowner, LambertProblemMsg_C *data);

void LambertProblemMsg_C_init(LambertProblemMsg_C *owner);

int LambertProblemMsg_C_isLinked(LambertProblemMsg_C *data);

int LambertProblemMsg_C_isWritten(LambertProblemMsg_C *data);

uint64_t LambertProblemMsg_C_timeWritten(LambertProblemMsg_C *data);

int64_t LambertProblemMsg_C_moduleID(LambertProblemMsg_C *data);

void LambertProblemMsg_C_write(LambertProblemMsgPayload *data, LambertProblemMsg_C *destination, int64_t moduleID, uint64_t callTime);

LambertProblemMsgPayload LambertProblemMsg_C_read(LambertProblemMsg_C *source);

LambertProblemMsgPayload LambertProblemMsg_C_zeroMsgPayload();

#ifdef __cplusplus
}
#endif
#endif