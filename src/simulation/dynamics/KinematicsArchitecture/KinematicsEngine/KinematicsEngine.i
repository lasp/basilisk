/*
 ISC License

 Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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
%module KinematicsEngine
%{
    #include "../../_GeneralModuleFiles/KinematicsEngine.h"
    #include "../../_GeneralModuleFiles/Joint.h"
    #include "../../_GeneralModuleFiles/Hinge.h"
    #include "../../_GeneralModuleFiles/Part.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_eigen.i"
%include "swig_conly_data.i"
%include "std_vector.i"

%include "../Vector/Vector.i"
%include "../Joint/Joint.i"
%include "../Tensor/Tensor.i"

%include <std_shared_ptr.i>
%shared_ptr(KinematicsEngine)
%shared_ptr(Joint)
%shared_ptr(RotaryOneDOF)
%shared_ptr(RotaryTwoDOF)

%include "../../_GeneralModuleFiles/KinematicsEngine.h"
%include "../../_GeneralModuleFiles/Assembly.h"
%include "../../_GeneralModuleFiles/Part.h"
%include "../../_GeneralModuleFiles/Joint.h"
%include "../../_GeneralModuleFiles/Point.h"
%include "../../_GeneralModuleFiles/Vector.h"
%include "../../_GeneralModuleFiles/Frame.h"

namespace std {
        %template(HingeVector) vector<shared_ptr<Hinge>>;
        %template(JointVector) vector<shared_ptr<Joint>>;
        %template(PartVector) vector<shared_ptr<Part>>;
        %template(PointVector) vector<shared_ptr<Point>>;
        %template(PositionVectorVector) vector<shared_ptr<PositionVector>>;
        %template(FrameVector) vector<shared_ptr<Frame>>;
}

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
