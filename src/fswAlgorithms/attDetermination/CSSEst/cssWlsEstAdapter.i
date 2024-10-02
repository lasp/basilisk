%module cssWlsEstAdapter
%{
   #include "cssWlsEstAdapter.hpp"
      %}

%pythoncode %{
    from Basilisk.architecture.swig_common_model import *
%}

%include "sys_model.i"
%include "swig_conly_data.i"
%import "cssWlsEst.h"
%include "cssWlsEstAdapter.hpp"

%include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
%include "architecture/msgPayloadDefC/CSSConfigMsgPayload.h"
%include "architecture/msgPayloadDefC/CSSUnitConfigMsgPayload.h"
%include "architecture/msgPayloadDefC/SunlineFilterMsgPayload.h"
%include "architecture/msgPayloadDefC/CSSArraySensorMsgPayload.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
