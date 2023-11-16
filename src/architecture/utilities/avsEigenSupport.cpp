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

#include <iostream>
#include <math.h>
#include "avsEigenSupport.h"
#include "rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"

/*

 Contains various support algorithms related to using the Eigen Library

 */

/*! This function provides a general conversion between an Eigen matrix and
an output C array. Note that this routine would convert an inbound type
to a MatrixXd and then transpose the matrix which would be inefficient
in a lot of cases.
@return void
@param inMat The source Eigen matrix that we are converting
@param outArray The destination array (sized by the user!) we copy into
*/
void eigenMatrixXd2CArray(Eigen::MatrixXd inMat, float *outArray)
{
	Eigen::MatrixXd tempMat = inMat.transpose();
	memcpy(outArray, tempMat.data(), inMat.rows()*inMat.cols()*sizeof(float));
}

/*! This function provides a general conversion between an Eigen matrix and
an output C array. Note that this routine would convert an inbound type
to a MatrixXd and then transpose the matrix which would be inefficient
in a lot of cases.
@return void
@param inMat The source Eigen matrix that we are converting
@param outArray The destination array (sized by the user!) we copy into
*/
void eigenMatrixXi2CArray(Eigen::MatrixXi inMat, int *outArray)
{
    Eigen::MatrixXi tempMat = inMat.transpose();
    memcpy(outArray, tempMat.data(), inMat.rows()*inMat.cols()*sizeof(int));
}

/*! This function provides a direct conversion between a 3-vector and an
output C array. We are providing this function to save on the  inline conversion
and the transpose that would have been performed by the general case.
@return void
@param inMat The source Eigen matrix that we are converting
@param outArray The destination array we copy into
*/
void eigenVector3f2CArray(Eigen::Vector3f & inMat, float *outArray)
{
	memcpy(outArray, inMat.data(), 3 * sizeof(float));
}

/*! This function provides a direct conversion between an MRP and an
output C array. We are providing this function to save on the inline conversion
and the transpose that would have been performed by the general case.
@return void
@param inMat The source Eigen MRP that we are converting
@param outArray The destination array we copy into
*/
void eigenMRPd2CArray(Eigen::Vector3f& inMat, float* outArray)
{
    memcpy(outArray, inMat.data(), 3 * sizeof(float));
}

/*! This function provides a direct conversion between a 3x3 matrix and an
output C array. We are providing this function to save on the inline conversion
that would have been performed by the general case.
@return void
@param inMat The source Eigen matrix that we are converting
@param outArray The destination array we copy into
*/
void eigenMatrix3f2CArray(Eigen::Matrix3f & inMat, float *outArray)
{
	Eigen::MatrixXd tempMat = inMat.transpose();
	memcpy(outArray, tempMat.data(), 9 * sizeof(float));
}

/*! This function performs the general conversion between an input C array
and an Eigen matrix. Note that to use this function the user MUST size
the Eigen matrix ahead of time so that the internal map call has enough
information to ingest the C array.
@return Eigen::MatrixXd
@param inArray The input array (row-major)
@param nRows
@param nCols
*/
Eigen::MatrixXd cArray2EigenMatrixXd(float *inArray, int nRows, int nCols)
{
    Eigen::MatrixXd outMat;
    outMat.resize(nRows, nCols);
	outMat = Eigen::Map<Eigen::MatrixXd>(inArray, outMat.rows(), outMat.cols());
    return outMat;
}

/*! This function performs the conversion between an input C array
3-vector and an output Eigen vector3d. This function is provided
in order to save an unnecessary conversion between types.
@return Eigen::Vector3f
@param inArray The input array (row-major)
*/
Eigen::Vector3f cArray2EigenVector3f(float *inArray)
{
    return Eigen::Map<Eigen::Vector3f>(inArray, 3, 1);
}

/*! This function performs the conversion between an input C array
3-vector and an output Eigen MRPd. This function is provided
in order to save an unnecessary conversion between types.
@return Eigen::MRPd
@param inArray The input array (row-major)
*/
Eigen::MRPd cArray2EigenMRPd(float* inArray)
{
    Eigen::MRPd sigma_Eigen;
    sigma_Eigen = cArray2EigenVector3f(inArray);

    return sigma_Eigen;
}

/*! This function performs the conversion between an input C array
3x3-matrix and an output Eigen vector3d. This function is provided
in order to save an unnecessary conversion between types.
@return Eigen::Matrix3f
@param inArray The input array (row-major)
*/
Eigen::Matrix3f cArray2EigenMatrix3f(float *inArray)
{
	return Eigen::Map<Eigen::Matrix3f>(inArray, 3, 3).transpose();
}

/*! This function performs the conversion between an input C 3x3 
2D-array and an output Eigen vector3d. This function is provided
in order to save an unnecessary conversion between types
@return Eigen::Matrix3f
@param in2DArray The input 2D-array
*/
Eigen::Matrix3f c2DArray2EigenMatrix3f(float in2DArray[3][3])
{
    Eigen::Matrix3f outMat;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            outMat(i, j) = in2DArray[i][j];
        }
    }

    return outMat;
}

/*! This function returns the Eigen DCM that corresponds to a 1-axis rotation
 by the angle theta.  The DCM is the positive theta rotation from the original
 frame to the final frame.
 @return Eigen::Matrix3f
 @param angle The input rotation angle
 */
Eigen::Matrix3f eigenM1(float angle)
{
    Eigen::Matrix3f mOut;

    mOut.setIdentity();

    mOut(1,1) = cos(angle);
    mOut(1,2) = sin(angle);
    mOut(2,1) = -mOut(1,2);
    mOut(2,2) = mOut(1,1);

    return mOut;
}

/*! This function returns the Eigen DCM that corresponds to a 2-axis rotation
 by the angle theta.  The DCM is the positive theta rotation from the original
 frame to the final frame.
 @return Eigen::Matrix3f
 @param angle The input rotation angle
 */
Eigen::Matrix3f eigenM2(float angle)
{
    Eigen::Matrix3f mOut;

    mOut.setIdentity();

    mOut(0,0) = cos(angle);
    mOut(0,2) = -sin(angle);
    mOut(2,0) = -mOut(0,2);
    mOut(2,2) = mOut(0,0);

    return mOut;
}

/*! This function returns the Eigen DCM that corresponds to a 3-axis rotation
 by the angle theta.  The DCM is the positive theta rotation from the original
 frame to the final frame.
 @return Eigen::Matrix3f
 @param angle The input rotation angle
 */
Eigen::Matrix3f eigenM3(float angle)
{
    Eigen::Matrix3f mOut;

    mOut.setIdentity();

    mOut(0,0) = cos(angle);
    mOut(0,1) = sin(angle);
    mOut(1,0) = -mOut(0,1);
    mOut(1,1) = mOut(0,0);

    return mOut;
}

/*! This function returns the tilde matrix version of a vector. The tilde
 matrix is the matrixi equivalent of a vector cross product, where
 [tilde_a] b == a x b
 @return Eigen::Matrix3f
 @param vec The input vector
 */
Eigen::Matrix3f eigenTilde(Eigen::Vector3f vec)
{
    Eigen::Matrix3f mOut;

    mOut(0,0) = mOut(1,1) = mOut(2,2) = 0.0;

    mOut(0,1) = -vec(2);
    mOut(1,0) =  vec(2);
    mOut(0,2) =  vec(1);
    mOut(2,0) = -vec(1);
    mOut(1,2) = -vec(0);
    mOut(2,1) =  vec(0);

    return mOut;
}

/*! This function converts the Eigen DCM to an Eigen MRPd
 @return Eigen::MRPd
 @param dcm_Eigen The input DCM
 */
Eigen::MRPd eigenC2MRP(Eigen::Matrix3f dcm_Eigen)
{
    Eigen::MRPd sigma_Eigen;  // output Eigen MRP
    float dcm_Array[9];      // C array DCM
    float sigma_Array[3];    // C array MRP

    eigenMatrix3f2CArray(dcm_Eigen, dcm_Array);
    C2MRP(RECAST3X3 dcm_Array, sigma_Array);
    sigma_Eigen = cArray2EigenVector3f(sigma_Array);

    return sigma_Eigen;
}

/*! This function converts the Eigen MRPd to Vector3f
 @return Eigen::Vector3f
 @param mrp The input Vector3f variable
 */
Eigen::Vector3f eigenMRPd2Vector3f(Eigen::MRPd mrp)
{
    Eigen::Vector3f vec3d;

    vec3d[0] = mrp.x();
    vec3d[1] = mrp.y();
    vec3d[2] = mrp.z();

    return vec3d;
}

/*! This function solves for the zero of the passed function using the Newton Raphson Method
@return float
@param initialEstimate The initial value to use for newton-raphson
@param accuracy The desired upper bound for the error
@param f Function to find the zero of
@param fPrime First derivative of the function
*/
float newtonRaphsonSolve(const float& initialEstimate, const float& accuracy, const std::function<float(float)>& f, const std::function<float(float)>& fPrime) {
	float currentEstimate = initialEstimate;
	for (int i = 0; i < 100; i++) {
        if (std::abs(f(currentEstimate)) < accuracy)
            break;

		float functionVal = f(currentEstimate);
		float functionDeriv = fPrime(currentEstimate);
		currentEstimate = currentEstimate - functionVal/functionDeriv;
	}
	return currentEstimate;
}

