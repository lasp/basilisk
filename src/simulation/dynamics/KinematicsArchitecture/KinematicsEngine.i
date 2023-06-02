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
%pythoncode %{
    from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_eigen.i"
%include "swig_conly_data.i"
%include "std_vector.i"
%include <std_shared_ptr.i>

%{
    #include "simulation/dynamics/KinematicsArchitecture/KinematicsEngine.h"
%}

%shared_ptr(Assembly)
%shared_ptr(Frame)
%shared_ptr(InertiaTensor)
%shared_ptr(Joint)
%shared_ptr(KinematicsEngine)
%shared_ptr(Node)
%shared_ptr(Part)
%shared_ptr(Point)
%shared_ptr(RotaryOneDOF)
%shared_ptr(RotaryTwoDOF)
%shared_ptr(Rotation)
%shared_ptr(Tensor)
%shared_ptr(Translation)
%shared_ptr(Vector)

%include "simulation/dynamics/KinematicsArchitecture/KinematicsEngine.h"

namespace std {
        %template(JointVector) vector<shared_ptr<Joint>>;
        %template(PartVector) vector<shared_ptr<Part>>;
        %template(PointVector) vector<shared_ptr<Point>>;
        %template(TranslationVector) vector<shared_ptr<Translation>>;
        %template(RotationVector) vector<shared_ptr<Rotation>>;
        %template(FrameVector) vector<shared_ptr<Frame>>;
        %template(NodeVector) vector<shared_ptr<Node>>;
}

%pythoncode %{
    import sys
    protectAllClasses(sys.modules[__name__])
%}
