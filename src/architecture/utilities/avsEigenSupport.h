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


#ifndef _AVSEIGENSUPPORT_
#define _AVSEIGENSUPPORT_
#include <Eigen/Dense>
#include "avsEigenMRP.h"


//!@brief General conversion between any Eigen matrix and output array
void eigenMatrixXf2CArray(Eigen::MatrixXf inMat, float *outArray);
//!@brief General conversion between any Eigen matrix and output array
void eigenMatrixXi2CArray(Eigen::MatrixXi inMat, int *outArray);
//!@brief Rapid conversion between 3-vector and output array
void eigenVector3f2CArray(Eigen::Vector3f & inMat, float *outArray);
//!@brief Rapid conversion between MRP and output array
void eigenMRPd2CArray(Eigen::Vector3f& inMat, float* outArray);
//!@brief Rapid conversion between 3x3 matrix and output array
void eigenMatrix3f2CArray(Eigen::Matrix3f & inMat, float *outArray);
//!@brief General conversion between a C array and an Eigen matrix
Eigen::MatrixXf cArray2EigenMatrixXf(float *inArray, int nRows, int nCols);
//!@brief Specific conversion between a C array and an Eigen 3-vector
Eigen::Vector3f cArray2EigenVector3f(float *inArray);
//!@brief Specific conversion between a C array and an Eigen MRPs
Eigen::MRPd cArray2EigenMRPd(float* inArray);
//!@brief Specfici conversion between a C array and an Eigen 3x3 matrix
Eigen::Matrix3f cArray2EigenMatrix3f(float *inArray);
//!@brief Specfici conversion between a C 2D array and an Eigen 3x3 matrix
Eigen::Matrix3f c2DArray2EigenMatrix3f(float in2DArray[3][3]);
//!@brief returns the first axis DCM with the input angle 
Eigen::Matrix3f eigenM1(float angle);
//!@brief returns the second axis DCM with the input angle
Eigen::Matrix3f eigenM2(float angle);
//!@brief returns the third axis DCM with the input angle
Eigen::Matrix3f eigenM3(float angle);
//!@brief returns the tilde matrix representation of a vector (equivalent to a vector cross product)
Eigen::Matrix3f eigenTilde(Eigen::Vector3f vec);
//!@brief converts MRPd to an Vector3f variable
Eigen::Vector3f eigenMRPd2Vector3f(Eigen::MRPd vec);
//!@brief maps the DCM to MRPs using Eigen variables
Eigen::MRPd eigenC2MRP(Eigen::Matrix3f);

//!@brief solves for the zero of the provided function
float newtonRaphsonSolve(const float& initialEstimate, const float& accuracy, const std::function<float(float)>& f, const std::function<float(float)>& fPrime);


#endif /* _AVSEIGENSUPPORT_ */
