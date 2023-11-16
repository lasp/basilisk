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

#ifndef _LINEARALGEBRA_H_
#define _LINEARALGEBRA_H_

#include <stdio.h>
#include <architecture/utilities/bskLogging.h>

/* Divide by zero epsilon value */
#define DB0_EPS 1e-30

/* define a maximum array size for the functions that need
 to allocate memory within their routine */
#define LINEAR_ALGEBRA_MAX_ARRAY_SIZE (64*64)

#define MXINDEX(dim2, row, col) ((row)*(dim2) + (col))

/* General vectors */
#ifdef __cplusplus
extern "C" {
#endif

    /* N element vectors */
    void    vElementwiseMult(float *v1, size_t dim, float *v2, float *result);
    void    vCopy(float *v, size_t dim, float *result);
    void    vSetZero(float *v, size_t dim);
    void    vSetOnes(float *v, size_t dim);
    void    vAdd(float *v1, size_t dim, float *v2, float *result);
    void    vSubtract(float *v1, size_t dim, float *v2, float *result);
    void    vScale(float scaleFactor, float *v, size_t dim, float *result);
    float  vDot(float *v1, size_t dim, float *v2);
    void    vOuterProduct(float *v1, size_t dim1, float *v2, size_t dim2, void *result);
    void    vtMultM(float *v, void *mx, size_t dim1, size_t dim2, void *result);
    void    vtMultMt(float *v, void *mx, size_t dim1, size_t dim2, void *result);
    float  vNorm(float *v, size_t dim);
    float  vMax(float *v, size_t dim); /* Non-sorted, non-optimized algorithm for finding the max of a small 1-D array*/
    float  vMaxAbs(float *v, size_t dim); /* Non-sorted, non-optimized algorithm for finding the max of the absolute values of the elements of a small 1-D array*/
    void    vNormalize(float *v, size_t dim, float *result);
    int     vIsEqual(float *v1, size_t dim, float *v2, float accuracy);
    int     vIsZero(float *v, size_t dim, float accuracy);
    void    vPrint(FILE *pFile, const char *name, float *v, size_t dim);
    void    vSort(float *Input, float *Output, size_t dim);

    /* 2 element vectors */
    void    v2Set(float v0, float v1, float result[2]);
    void    v2Copy(float v[2], float result[2]);
    void    v2Scale(float scaleFactor, float v[2], float result[2]);
    void    v2SetZero(float v[2]);
    float  v2Dot(float v1[2], float v2[2]);
    int     v2IsEqual(float v1[2], float v2[2], float accuracy);
    int     v2IsZero(float v[2], float accuracy);
    void    v2Add(float v1[2], float v2[2], float result[2]);
    void    v2Subtract(float v1[2], float v2[2], float result[2]);
    float  v2Norm(float v1[2]);
    void    v2Normalize(float v1[2], float result[2]);

    /* 3 element vectors */
    void    v3Set(float v0, float v1, float v2, float result[3]);
    void    v3Copy(float v[3], float result[3]);
    void    v3SetZero(float v[3]);
    void    v3Add(float v1[3], float v2[3], float result[3]);
    void    v3Subtract(float v1[3], float v2[3], float result[3]);
    void    v3Scale(float scaleFactor, float v[3], float result[3]);
    float  v3Dot(float v1[3], float v2[3]);
    void    v3OuterProduct(float v1[3], float v2[3], float result[3][3]);
    void    v3tMultM33(float v[3], float mx[3][3], float result[3]);
    void    v3tMultM33t(float v[3], float mx[3][3], float result[3]);
    float  v3Norm(float v[3]);
    void    v3Normalize(float v[3], float result[3]);
    int     v3IsEqual(float v1[3], float v2[3], float accuracy);
    int     v3IsEqualRel(float v1[3], float v2[3], float accuracy);
    int     v3IsZero(float v[3], float accuracy);
    void    v3Print(FILE *pFile, const char *name, float v[3]);
    void    v3Cross(float v1[3], float v2[3], float result[3]);
    void    v3Perpendicular(float v[3], float result[3]);
    void    v3Tilde(float v[3], float result[3][3]);
    void    v3Sort(float v[3], float result[3]);
    void    v3PrintScreen(const char *name, float v[3]);

    /* 4 element vectors */
    void    v4Set(float v0, float v1, float v2, float v3, float result[4]);
    void    v4Copy(float v[4], float result[4]);
    void    v4SetZero(float v[4]);
    float  v4Dot(float v1[4], float v2[4]);
    float  v4Norm(float v[4]);
    int     v4IsEqual(float v1[4], float v2[4], float accuracy);
    int     v4IsZero(float v[4], float accuracy);

    /* 6 element vectors */
    void    v6Set(float v0, float v1, float v2, float v3, float v4, float v5, float result[6]);
    void    v6Copy(float v[6], float result[6]);
    float  v6Dot(float v1[6], float v2[6]);
    void    v6Scale(float scaleFactor, float v[6], float result[6]);
    void    v6OuterProduct(float v1[6], float v2[6], float result[6][6]);
    int     v6IsEqual(float v1[6], float v2[6], float accuracy);

    /* NxM matrices */
    void    mLeastSquaresInverse(void *mx, size_t dim1, size_t dim2, void *result);
    void    mMinimumNormInverse(void *mx, size_t dim1, size_t dim2, void *result);
    void    mCopy(void *mx, size_t dim1, size_t dim2, void *result);
    void    mSetZero(void *result, size_t dim1, size_t dim2);
    void    mSetIdentity(void *result, size_t dim1, size_t dim2);
    void    mDiag(void *v, size_t dim, void *result);
    void    mTranspose(void *mx, size_t dim1, size_t dim2, void *result);
    void    mAdd(void *mx1, size_t dim1, size_t dim2, void *mx2, void *result);
    void    mSubtract(void *mx1, size_t dim1, size_t dim2, void *mx2, void *result);
    void    mScale(float scaleFactor, void *mx, size_t dim1, size_t dim2, void *result);
    void    mMultM(void *mx1, size_t dim11, size_t dim12,
                   void *mx2, size_t dim21, size_t dim22,
                   void *result);
    void    mtMultM(void *mx1, size_t dim11, size_t dim12,
                    void *mx2, size_t dim21, size_t dim22,
                    void *result);
    void    mMultMt(void *mx1, size_t dim11, size_t dim12,
                    void *mx2, size_t dim21, size_t dim22,
                    void *result);
    void    mtMultMt(void *mx1, size_t dim11, size_t dim12,
                     void *mx2, size_t dim21, size_t dim22,
                     void *result);
    void    mMultV(void *mx, size_t dim1, size_t dim2,
                   void *v,
                   void *result);
    void    mtMultV(void *mx, size_t dim1, size_t dim2,
                    void *v,
                    void *result);
    float  mTrace(void *mx, size_t dim);
    float  mDeterminant(void *mx, size_t dim);
    void    mCofactor(void *mx, size_t dim, void *result);
    int     mInverse(void *mx, size_t dim, void *result);
    int     mIsEqual(void *mx1, size_t dim1, size_t dim2, void *mx2, float accuracy);
    int     mIsZero(void *mx, size_t dim1, size_t dim2, float accuracy);
    void mPrintScreen(const char *name, void *mx, size_t dim1, size_t dim2);
    void    mPrint(FILE *pFile, const char *name, void *mx, size_t dim1, size_t dim2);
    void    mGetSubMatrix(void *mx, size_t dim1, size_t dim2,
                          size_t dim1Start, size_t dim2Start,
                          size_t dim1Result, size_t dim2Result, void *result);
    void    mSetSubMatrix(void *mx, size_t dim1, size_t dim2,
                          void *result, size_t dim1Result, size_t dim2Result,
                          size_t dim1Start, size_t dim2Start);

    /* 2x2 matrices */
    void    m22Set(float m00, float m01,
                   float m10, float m11,
                   float m[2][2]);
    void    m22Copy(float mx[2][2], float result[2][2]);
    void    m22SetZero(float result[2][2]);
    void    m22SetIdentity(float result[2][2]);
    void    m22Transpose(float mx[2][2], float result[2][2]);
    void    m22Add(float mx1[2][2], float mx2[2][2], float result[2][2]);
    void    m22Subtract(float mx1[2][2], float mx2[2][2], float result[2][2]);
    void    m22Scale(float scaleFactor, float mx[2][2], float result[2][2]);
    void    m22MultM22(float mx1[2][2], float mx2[2][2], float result[2][2]);
    void    m22tMultM22(float mx1[2][2], float mx2[2][2], float result[2][2]);
    void    m22MultM22t(float mx1[2][2], float mx2[2][2], float result[2][2]);
    void    m22MultV2(float mx[2][2], float v[2], float result[2]);
    void    m22tMultV2(float mx[2][2], float v[2], float result[2]);
    float  m22Trace(float mx[2][2]);
    float  m22Determinant(float mx[2][2]);
    int     m22IsEqual(float mx1[2][2], float mx2[2][2], float accuracy);
    int     m22IsZero(float mx[2][2], float accuracy);
    void    m22Print(FILE *pFile, const char *name, float mx[2][2]);
    int     m22Inverse(float mx[2][2], float result[2][2]);
    void    m22PrintScreen(const char *name, float mx[2][2]);

    /* 3x3 matrices */
    void    m33Set(float m00, float m01, float m02,
                   float m10, float m11, float m12,
                   float m20, float m21, float m22,
                   float m[3][3]);
    void    m33Copy(float mx[3][3], float result[3][3]);
    void    m33SetZero(float result[3][3]);
    void    m33SetIdentity(float result[3][3]);
    void    m33Transpose(float mx[3][3], float result[3][3]);
    void    m33Add(float mx1[3][3], float mx2[3][3], float result[3][3]);
    void    m33Subtract(float mx1[3][3], float mx2[3][3], float result[3][3]);
    void    m33Scale(float scaleFactor, float mx[3][3], float result[3][3]);
    void    m33MultM33(float mx1[3][3], float mx2[3][3], float result[3][3]);
    void    m33tMultM33(float mx1[3][3], float mx2[3][3], float result[3][3]);
    void    m33MultM33t(float mx1[3][3], float mx2[3][3], float result[3][3]);
    void    m33MultV3(float mx[3][3], float v[3], float result[3]);
    void    m33tMultV3(float mx[3][3], float v[3], float result[3]);
    float  m33Trace(float mx[3][3]);
    float  m33Determinant(float mx[3][3]);
    int     m33IsEqual(float mx1[3][3], float mx2[3][3], float accuracy);
    int     m33IsZero(float mx[3][3], float accuracy);
    void    m33Print(FILE *pfile, const char *name, float mx[3][3]);
    int     m33Inverse(float mx[3][3], float result[3][3]);
    void    m33SingularValues(float mx[3][3], float result[3]);
    void    m33EigenValues(float mx[3][3], float result[3]);
    float  m33ConditionNumber(float mx[3][3]);
    void    m33PrintScreen(const char *name, float mx[3][3]);

    /* 4x4 matrices */
    void    m44Set(float m00, float m01, float m02, float m03,
                   float m10, float m11, float m12, float m13,
                   float m20, float m21, float m22, float m23,
                   float m30, float m31, float m32, float m33,
                   float m[4][4]);
    void    m44Copy(float mx[4][4], float result[4][4]);
    void    m44SetZero(float result[4][4]);
    void    m44MultV4(float mx[4][4], float v[4], float result[4]);
    float  m44Determinant(float mx[4][4]);
    int     m44IsEqual(float mx1[4][4], float mx2[4][4], float accuracy);
    int     m44Inverse(float mx[4][4], float result[4][4]);

    /* 6x6 matrices */
    void    m66Set(float m00, float m01, float m02, float m03, float m04, float m05,
                   float m10, float m11, float m12, float m13, float m14, float m15,
                   float m20, float m21, float m22, float m23, float m24, float m25,
                   float m30, float m31, float m32, float m33, float m34, float m35,
                   float m40, float m41, float m42, float m43, float m44, float m45,
                   float m50, float m51, float m52, float m53, float m54, float m55,
                   float m[6][6]);
    void    m66Copy(float mx[6][6], float result[6][6]);
    void    m66SetZero(float result[6][6]);
    void    m66SetIdentity(float result[6][6]);
    void    m66Transpose(float mx[6][6], float result[6][6]);
    void    m66Get33Matrix(size_t row, size_t col, float m[6][6], float mij[3][3]);
    void    m66Set33Matrix(size_t row, size_t col, float mij[3][3], float m[6][6]);
    void    m66Scale(float scaleFactor, float mx[6][6], float result[6][6]);
    void    m66Add(float mx1[6][6], float mx2[6][6], float result[6][6]);
    void    m66Subtract(float mx1[6][6], float mx2[6][6], float result[6][6]);
    void    m66MultM66(float mx1[6][6], float mx2[6][6], float result[6][6]);
    void    m66tMultM66(float mx1[6][6], float mx2[6][6], float result[6][6]);
    void    m66MultM66t(float mx1[6][6], float mx2[6][6], float result[6][6]);
    void    m66MultV6(float mx[6][6], float v[6], float result[6]);
    void    m66tMultV6(float mx[6][6], float v[6], float result[6]);
    int     m66IsEqual(float mx1[6][6], float mx2[6][6], float accuracy);
    int     m66IsZero(float mx[6][6], float accuracy);

    /* 9x9 matrices */
    void    m99SetZero(float result[9][9]);

    /* additional routines */
    /* Solve cubic formula for x^3 + a[2]*x^2 + a[1]*x + a[0] = 0 */
    void    cubicRoots(float a[3], float result[3]);

    float safeAcos(float x);
    float safeAsin(float x);
    float safeSqrt(float x);

#ifdef __cplusplus
}
#endif

#endif

