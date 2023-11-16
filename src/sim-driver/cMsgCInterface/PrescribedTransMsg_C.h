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

#ifndef PrescribedTransMsg_C_H
#define PrescribedTransMsg_C_H

#include <stdint.h>
#include "architecture/msgPayloadDefC/PrescribedTransMsgPayload.h"
#include "architecture/messaging/msgHeader.h"

//! structure definition
typedef struct {
    MsgHeader header;              //!< message header, zero'd on construction
    PrescribedTransMsgPayload payload;		        //!< message copy, zero'd on construction
    PrescribedTransMsgPayload *payloadPointer;	    //!< pointer to message
    MsgHeader *headerPointer;      //!< pointer to message header
} PrescribedTransMsg_C;

#ifdef __cplusplus
extern "C" {
#endif

void PrescribedTransMsg_cpp_subscribe(PrescribedTransMsg_C *subscriber, void* source);

void PrescribedTransMsg_C_subscribe(PrescribedTransMsg_C *subscriber, PrescribedTransMsg_C *source);

int8_t PrescribedTransMsg_C_isSubscribedTo(PrescribedTransMsg_C *subscriber, PrescribedTransMsg_C *source);
int8_t PrescribedTransMsg_cpp_isSubscribedTo(PrescribedTransMsg_C *subscriber, void* source);

void PrescribedTransMsg_C_addAuthor(PrescribedTransMsg_C *coowner, PrescribedTransMsg_C *data);

void PrescribedTransMsg_C_init(PrescribedTransMsg_C *owner);

int PrescribedTransMsg_C_isLinked(PrescribedTransMsg_C *data);

int PrescribedTransMsg_C_isWritten(PrescribedTransMsg_C *data);

uint64_t PrescribedTransMsg_C_timeWritten(PrescribedTransMsg_C *data);

int64_t PrescribedTransMsg_C_moduleID(PrescribedTransMsg_C *data);

void PrescribedTransMsg_C_write(PrescribedTransMsgPayload *data, PrescribedTransMsg_C *destination, int64_t moduleID, uint64_t callTime);

PrescribedTransMsgPayload PrescribedTransMsg_C_read(PrescribedTransMsg_C *source);

PrescribedTransMsgPayload PrescribedTransMsg_C_zeroMsgPayload();

#ifdef __cplusplus
}
#endif
#endif