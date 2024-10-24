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
%module mtbMomentumManagement
%{
   #include "mtbMomentumManagement.h"
%}

%pythoncode %{
    from Basilisk.architecture.swig_common_model import *
%}

%include "sys_model.i"
%include "swig_conly_data.i"

%include "mtbMomentumManagement.h"

// sample Module support file to be included in this sub-module
%include "architecture/msgPayloadDefC/RWArrayConfigMsgPayload.h"
%include "architecture/msgPayloadDefC/MTBArrayConfigMsgPayload.h"
%include "architecture/msgPayloadDefC/TAMSensorBodyMsgPayload.h"
%include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
%include "architecture/msgPayloadDefC/MTBCmdMsgPayload.h"
%include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
