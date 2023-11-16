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

#ifndef CameraLocalizationMsg_C_H
#define CameraLocalizationMsg_C_H

#include <stdint.h>
#include "architecture/msgPayloadDefC/CameraLocalizationMsgPayload.h"
#include "architecture/messaging/msgHeader.h"

//! structure definition
typedef struct {
    MsgHeader header;              //!< message header, zero'd on construction
    CameraLocalizationMsgPayload payload;		        //!< message copy, zero'd on construction
    CameraLocalizationMsgPayload *payloadPointer;	    //!< pointer to message
    MsgHeader *headerPointer;      //!< pointer to message header
} CameraLocalizationMsg_C;

#ifdef __cplusplus
extern "C" {
#endif

void CameraLocalizationMsg_cpp_subscribe(CameraLocalizationMsg_C *subscriber, void* source);

void CameraLocalizationMsg_C_subscribe(CameraLocalizationMsg_C *subscriber, CameraLocalizationMsg_C *source);

int8_t CameraLocalizationMsg_C_isSubscribedTo(CameraLocalizationMsg_C *subscriber, CameraLocalizationMsg_C *source);
int8_t CameraLocalizationMsg_cpp_isSubscribedTo(CameraLocalizationMsg_C *subscriber, void* source);

void CameraLocalizationMsg_C_addAuthor(CameraLocalizationMsg_C *coowner, CameraLocalizationMsg_C *data);

void CameraLocalizationMsg_C_init(CameraLocalizationMsg_C *owner);

int CameraLocalizationMsg_C_isLinked(CameraLocalizationMsg_C *data);

int CameraLocalizationMsg_C_isWritten(CameraLocalizationMsg_C *data);

uint64_t CameraLocalizationMsg_C_timeWritten(CameraLocalizationMsg_C *data);

int64_t CameraLocalizationMsg_C_moduleID(CameraLocalizationMsg_C *data);

void CameraLocalizationMsg_C_write(CameraLocalizationMsgPayload *data, CameraLocalizationMsg_C *destination, int64_t moduleID, uint64_t callTime);

CameraLocalizationMsgPayload CameraLocalizationMsg_C_read(CameraLocalizationMsg_C *source);

CameraLocalizationMsgPayload CameraLocalizationMsg_C_zeroMsgPayload();

#ifdef __cplusplus
}
#endif
#endif