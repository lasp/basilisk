%module miruLowPassFilterConverter
%{
    #include "miruLowPassFilterConverter.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_conly_data.i"

%include "sys_model.i"
%include "miruLowPassFilterConverter.h"

%include "architecture/msgPayloadDefC/AccDataMsgPayload.h"
%include "architecture/msgPayloadDefC/IMUSensorMsgPayload.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
