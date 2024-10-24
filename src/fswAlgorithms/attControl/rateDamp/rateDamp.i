/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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
%module rateDamp
%{
   #include "rateDamp.h"
%}

%include "std_string.i"

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "swig_conly_data.i"
%include "swig_eigen.i"

%include "sys_model.h"
%include "rateDamp.h"

%include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
%include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
