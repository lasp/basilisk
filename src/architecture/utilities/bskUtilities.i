/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
%module bskUtilities
%{
   #include "architecture/utilities/macroDefinitions.h"
   #include "fswAlgorithms/fswUtilities/fswDefinitions.h"
   #include "simulation/dynamics/reactionWheels/reactionWheelSupport.h"

   #include <Eigen/Dense>
%}

// For some reason, "std_vector.i" needs to be included here,
// even though it's also included in "swig_common_model.i".
// Also, it *has* to appear before "swig_common_model.i";
// it can't appear after. *shrug*.
// Otherwise, we get weird errors involving an unknown type SWIGPY_SLICEOBJECT.
%include "std_vector.i"

%import "swig_common_model.i"

%include "architecture/utilities/macroDefinitions.h"
%include "fswAlgorithms/fswUtilities/fswDefinitions.h"
%include "simulation/dynamics/reactionWheels/reactionWheelSupport.h"

%template(Eigen3dVector) std::vector<Eigen::Vector3d, std::allocator<Eigen::Vector3d>>;

