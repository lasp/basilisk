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

#include "linearAlgebra.h"
#include "architecture/utilities/bsk_Print.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>



#define MOVE_FLOAT(source, dim, destination) (memmove((void*)(destination), (void*)(source), sizeof(float)*(dim)))

void vElementwiseMult(float *v1, size_t dim,
                       float *v2, float *result)
{
    size_t i;
    for(i = 0; i < dim; i++) {
        result[i] = v1[i] * v2[i];
    }
}

void vCopy(float *v, size_t dim,
           float *result)
{
    size_t i;
    for(i = 0; i < dim; i++) {
        result[i] = v[i];
    }
}

void vSetZero(float *v,
              size_t dim)
{
    size_t i;
    for(i = 0; i < dim; i++) {
        v[i] = 0.0;
    }
}

void vSetOnes(float *v,
              size_t dim)
{
    size_t i;
    for(i = 0; i < dim; i++) {
        v[i] = 1.0;
    }
}

void vAdd(float *v1, size_t dim,
          float *v2,
          float *result)
{
    size_t i;
    for(i = 0; i < dim; i++) {
        result[i] = v1[i] + v2[i];
    }
}

void vSubtract(float *v1, size_t dim,
               float *v2,
               float *result)
{
    size_t i;
    for(i = 0; i < dim; i++) {
        result[i] = v1[i] - v2[i];
    }
}

void vScale(float scaleFactor, float *v,
            size_t dim,
            float *result)
{
    size_t i;
    for(i = 0; i < dim; i++) {
        result[i] = v[i] * scaleFactor;
    }
}

float vDot(float *v1, size_t dim,
            float *v2)
{
    size_t i;
    float result = 0.0;
    for(i = 0; i < dim; i++) {
        result += v1[i] * v2[i];
    }

    return result;
}

void vOuterProduct(float *v1, size_t dim1,
                   float *v2, size_t dim2,
                   void *result)
{
    float *m_result = (float *)result;

    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            m_result[MXINDEX(dim2, i, j)] = v1[i] * v2[j];
        }
    }

}

void vtMultM(float *v,
             void *mx, size_t dim1, size_t dim2,
             void *result)
{
    size_t dim11 = 1;
    size_t dim12 = dim1;
    size_t dim22 = dim2;
    float *m_mx1 = (float *)v;
    float *m_mx2 = (float *)mx;
    float m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    if (dim11*dim22 > LINEAR_ALGEBRA_MAX_ARRAY_SIZE)
    {
        BSK_PRINT(MSG_ERROR,"Linear Algegra library array dimension input is too large.");
    }

    size_t i;
    size_t j;
    size_t k;
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[MXINDEX(dim22, i, j)] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[MXINDEX(dim22, i, j)] += m_mx1[MXINDEX(dim12, i, k)] * m_mx2[MXINDEX(dim22, k, j)];
            }
        }
    }

    MOVE_FLOAT(m_result, dim11 * dim22, result);
}

void vtMultMt(float *v,
              void *mx, size_t dim1, size_t dim2,
              void *result)
{
    size_t dim11 = 1;
    size_t dim12 = dim2;
    size_t dim22 = dim1;
    float *m_mx1 = (float *)v;
    float *m_mx2 = (float *)mx;
    float m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    if (dim11*dim22 > LINEAR_ALGEBRA_MAX_ARRAY_SIZE)
    {
        BSK_PRINT(MSG_ERROR,"Linear Algegra library array dimension input is too large.");
    }

    size_t i;
    size_t j;
    size_t k;
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[MXINDEX(dim22, i, j)] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[MXINDEX(dim22, i, j)] += m_mx1[MXINDEX(dim12, i, k)] * m_mx2[MXINDEX(dim22, j, k)];
            }
        }
    }

    MOVE_FLOAT(m_result, dim11 * dim22, result);
}

float vNorm(float *v, size_t dim)
{
    return sqrt(vDot(v, dim, v));
}

float vMax(float *array, size_t dim)
{
    size_t i;
    float result;

    result = array[0];
    for(i=1; i<dim; i++){
        if (array[i]>result){
            result = array[i];
        }
    }
    return result;
}


float vMaxAbs(float *array, size_t dim)
{
    size_t i;
    float result;

    result = fabs(array[0]);
    for(i=1; i<dim; i++){
        if (fabs(array[i])>result){
            result = fabs(array[i]);
        }
    }
    return result;
}


void vNormalize(float *v, size_t dim, float *result)
{
    float norm = vNorm(v, dim);

    if(norm > DB0_EPS) {
        vScale(1.0 / norm, v, dim, result);
    } else {
        vSetZero(result, dim);
    }
}

int vIsEqual(float *v1, size_t dim,
             float *v2,
             float accuracy)
{
    size_t i;
    for(i = 0; i < dim; i++) {
        if(fabs(v1[i] - v2[i]) > accuracy) {
            return 0;
        }
    }
    return 1;
}

int vIsZero(float *v, size_t dim, float accuracy)
{
    size_t i;
    int result = 1;
    for(i = 0; i < dim; i++) {
        if(fabs(v[i]) > accuracy) {
            result = 0;
            break;
        }
    }

    return result;
}

void vPrint(FILE *pFile, const char *name, float *v, size_t dim)
{
    size_t i;
    fprintf(pFile, "%s = [", name);
    for(i = 0; i < dim; i++) {
        fprintf(pFile, "%20.15g", v[i]);
        if(i != dim - 1) {
            fprintf(pFile, ", ");
        }
    }
    fprintf(pFile, "];\n");
}

/*I hope you allocated the output prior to calling this!*/
void vSort(float *Input, float *Output, size_t dim)
{
    size_t i, j;
    memcpy(Output, Input, dim*sizeof(float));
    for(i=0; i<dim; i++)
    {
        for(j=0; j<dim-1; j++)
        {
            if(Output[j]>Output[j+1])
            {
                float temp = Output[j+1];
                Output[j+1] = Output[j];
                Output[j] = temp;
            }
        }
    }
}


void v2Set(float v0, float v1,
           float result[2])
{
    result[0] = v0;
    result[1] = v1;
}

void v2SetZero(float v[2])
{
    size_t dim = 2;
    size_t i;
    for(i = 0; i < dim; i++) {
        v[i] = 0.0;
    }
}

void v2Copy(float v[2],
            float result[2])
{
    size_t dim = 2;
    size_t i;
    for(i = 0; i < dim; i++) {
        result[i] = v[i];
    }
}

void v2Scale(float scaleFactor,
             float v[2],
             float result[2])
{
    size_t dim = 2;
    size_t i;
    for(i = 0; i < dim; i++) {
        result[i] = v[i] * scaleFactor;
    }
}

float v2Dot(float v1[2],
             float v2[2])
{
    size_t dim = 2;
    size_t i;
    float result = 0.0;
    for(i = 0; i < dim; i++) {
        result += v1[i] * v2[i];
    }
    return result;
}

int v2IsEqual(float v1[2],
              float v2[2],
              float accuracy)
{
    size_t dim = 2;
    size_t i;
    for(i = 0; i < dim; i++) {
        if(fabs(v1[i] - v2[i]) > accuracy) {
            return 0;
        }
    }
    return 1;
}

int v2IsZero(float v[2],
             float accuracy)
{
    size_t dim = 2;
    size_t i;
    for(i = 0; i < dim; i++) {
        if(fabs(v[i]) > accuracy) {
            return 0;
        }
    }
    return 1;
}

void v2Add(float v1[2],
           float v2[2],
           float result[2])
{
    size_t dim = 2;
    size_t i;
    for(i = 0; i < dim; i++) {
        result[i] = v1[i] + v2[i];
    }
}

void v2Subtract(float v1[2],
                float v2[2],
                float result[2])
{
    size_t dim = 2;
    size_t i;
    for(i = 0; i < dim; i++) {
        result[i] = v1[i] - v2[i];
    }
}

float v2Norm(float v[2])
{
    return sqrt(v2Dot(v, v));
}

void v2Normalize(float v[2], float result[2])
{
    float norm = v2Norm(v);
    if(norm > DB0_EPS) {
        v2Scale(1. / norm, v, result);
    } else {
        v2SetZero(result);
    }
}






void v3Set(float v0, float v1, float v2,
           float result[3])
{
    result[0] = v0;
    result[1] = v1;
    result[2] = v2;
}

void v3Copy(float v[3],
            float result[3])
{
    size_t dim = 3;
    size_t i;
    for(i = 0; i < dim; i++) {
        result[i] = v[i];
    }
}

void v3SetZero(float v[3])
{
    size_t dim = 3;
    size_t i;
    for(i = 0; i < dim; i++) {
        v[i] = 0.0;
    }
}

void v3Add(float v1[3],
           float v2[3],
           float result[3])
{
    size_t dim = 3;
    size_t i;
    for(i = 0; i < dim; i++) {
        result[i] = v1[i] + v2[i];
    }
}

void v3Subtract(float v1[3],
                float v2[3],
                float result[3])
{
    size_t dim = 3;
    size_t i;
    for(i = 0; i < dim; i++) {
        result[i] = v1[i] - v2[i];
    }
}

void v3Scale(float scaleFactor,
             float v[3],
             float result[3])
{
    size_t dim = 3;
    size_t i;
    for(i = 0; i < dim; i++) {
        result[i] = v[i] * scaleFactor;
    }
}

float v3Dot(float v1[3],
             float v2[3])
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

void v3OuterProduct(float v1[3],
                    float v2[3],
                    float result[3][3])
{
    size_t dim = 3;
    size_t i;
    size_t j;
    for(i = 0; i < dim; i++) {
        for(j = 0; j < dim; j++) {
            result[i][j] = v1[i] * v2[j];
        }
    }
}

void v3tMultM33(float v[3],
                float mx[3][3],
                float result[3])
{
    size_t dim11 = 1;
    size_t dim12 = 3;
    size_t dim22 = 3;
    size_t i;
    size_t j;
    size_t k;
    float m_result[3];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[j] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[j] += v[k] * mx[k][j];
            }
        }
    }
    v3Copy(m_result, result);
}

void v3tMultM33t(float v[3],
                 float mx[3][3],
                 float result[3])
{
    size_t dim11 = 1;
    size_t dim12 = 3;
    size_t dim22 = 3;
    size_t i;
    size_t j;
    size_t k;
    float m_result[3];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[j] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[j] += v[k] * mx[j][k];
            }
        }
    }
    v3Copy(m_result, result);
}

float v3Norm(float v[3])
{
    return sqrt(v3Dot(v, v));
}

void v3Normalize(float v[3], float result[3])
{
    float norm = v3Norm(v);
    if(norm > DB0_EPS) {
        v3Scale(1. / norm, v, result);
    } else {
        v3SetZero(result);
    }
}

int v3IsEqual(float v1[3],
              float v2[3],
              float accuracy)
{
    size_t dim = 3;
    size_t i;
    for(i = 0; i < dim; i++) {
        if(fabs(v1[i] - v2[i]) > accuracy) {
            return 0;
        }
    }
    return 1;
}

int v3IsEqualRel(float v1[3],
              float v2[3],
              float accuracy)
{
    size_t dim = 3;
    size_t i;
    float norm;
    norm = v3Norm(v1);
    for(i = 0; i < dim; i++) {
        if(fabs(v1[i] - v2[i])/norm > accuracy) {
            return 0;
        }
    }
    return 1;
}



int v3IsZero(float v[3],
             float accuracy)
{
    size_t dim = 3;
    size_t i;
    for(i = 0; i < dim; i++) {
        if(fabs(v[i]) > accuracy) {
            return 0;
        }
    }
    return 1;
}

void v3Print(FILE *pFile, const char *name, float v[3])
{
    size_t dim = 3;
    size_t i;
    fprintf(pFile, "%s = [", name);
    for(i = 0; i < dim; i++) {
        fprintf(pFile, "%20.15g", v[i]);
        if(i != dim - 1) {
            fprintf(pFile, ", ");
        }
    }
    fprintf(pFile, "]\n");
}

void v3Cross(float v1[3],
             float v2[3],
             float result[3])
{
    float v1c[3];
    float v2c[3];
    v3Copy(v1, v1c);
    v3Copy(v2, v2c);
    result[0] = v1c[1] * v2c[2] - v1c[2] * v2c[1];
    result[1] = v1c[2] * v2c[0] - v1c[0] * v2c[2];
    result[2] = v1c[0] * v2c[1] - v1c[1] * v2c[0];
}

void v3Perpendicular(float v[3],
                     float result[3])
{
    if (fabs(v[0]) > DB0_EPS) {
        result[0] = -(v[1]+v[2]) / v[0];
        result[1] = 1;
        result[2] = 1;
    }
    else if (fabs(v[1]) > DB0_EPS) {
        result[0] = 1;
        result[1] = -(v[0]+v[2]) / v[1];
        result[2] = 1;
    }
    else {
        result[0] = 1;
        result[1] = 1;
        result[2] = -(v[0]+v[1]) / v[2];
    }
    v3Normalize(result, result);
}


void v3Tilde(float v[3],
             float result[3][3])
{
    result[0][0] = 0.0;
    result[0][1] = -v[2];
    result[0][2] = v[1];
    result[1][0] = v[2];
    result[1][1] = 0.0;
    result[1][2] = -v[0];
    result[2][0] = -v[1];
    result[2][1] = v[0];
    result[2][2] = 0.0;
}

void v3Sort(float v[3],
            float result[3])
{
    float temp;
    v3Copy(v, result);
    if(result[0] < result[1]) {
        temp = result[0];
        result[0] = result[1];
        result[1] = temp;
    }
    if(result[1] < result[2]) {
        if(result[0] < result[2]) {
            temp = result[2];
            result[2] = result[1];
            result[1] = result[0];
            result[0] = temp;
        } else {
            temp = result[1];
            result[1] = result[2];
            result[2] = temp;
        }
    }
}

void    v3PrintScreen(const char *name, float vec[3])
{
    printf("%s (%20.15g, %20.15g, %20.15g)\n", name, vec[0], vec[1], vec[2]);
}

void v4Set(float v0, float v1, float v2, float v3,
           float result[4])
{
    result[0] = v0;
    result[1] = v1;
    result[2] = v2;
    result[3] = v3;
}

void v4Copy(float v[4],
            float result[4])
{
    size_t dim = 4;
    size_t i;
    for(i = 0; i < dim; i++) {
        result[i] = v[i];
    }
}

void v4SetZero(float v[4])
{
    size_t dim = 4;
    size_t i;
    for(i = 0; i < dim; i++) {
        v[i] = 0.0;
    }
}

float v4Dot(float v1[4],
             float v2[4])
{
    size_t dim = 4;
    size_t i;
    float result = 0.0;
    for(i = 0; i < dim; i++) {
        result += v1[i] * v2[i];
    }
    return result;
}

float v4Norm(float v[4])
{
    return sqrt(v4Dot(v, v));
}

int v4IsEqual(float v1[4],
              float v2[4],
              float accuracy)
{
    size_t dim = 4;
    size_t i;
    for(i = 0; i < dim; i++) {
        if(fabs(v1[i] - v2[i]) > accuracy) {
            return 0;
        }
    }
    return 1;
}

int v4IsZero(float v[4],
             float accuracy)
{
    size_t dim = 4;
    size_t i;
    for(i = 0; i < dim; i++) {
        if(fabs(v[i]) > accuracy) {
            return 0;
        }
    }
    return 1;
}

void v6Set(float v0, float v1, float v2, float v3, float v4, float v5,
           float result[6])
{
    result[0] = v0;
    result[1] = v1;
    result[2] = v2;
    result[3] = v3;
    result[4] = v4;
    result[5] = v5;
}

void v6Copy(float v[6],
            float result[6])
{
    size_t dim = 6;
    size_t i;
    for(i = 0; i < dim; i++) {
        result[i] = v[i];
    }
}

float v6Dot(float v1[6],
             float v2[6])
{
    size_t dim = 6;
    size_t i;
    float result = 0.0;
    for(i = 0; i < dim; i++) {
        result += v1[i] * v2[i];
    }
    return result;
}

void v6Scale(float scaleFactor,
             float v[6],
             float result[6])
{
    size_t dim = 6;
    size_t i;
    for(i = 0; i < dim; i++) {
        result[i] = v[i] * scaleFactor;
    }
}

void v6OuterProduct(float v1[6],
                    float v2[6],
                    float result[6][6])
{
    size_t dim = 6;
    size_t i;
    size_t j;
    for(i = 0; i < dim; i++) {
        for(j = 0; j < dim; j++) {
            result[i][j] = v1[i] * v2[j];
        }
    }
}

int v6IsEqual(float v1[6],
              float v2[6],
              float accuracy)
{
    size_t dim = 6;
    size_t i;
    for(i = 0; i < dim; i++) {
        if(fabs(v1[i] - v2[i]) > accuracy) {
            return 0;
        }
    }
    return 1;
}

void mLeastSquaresInverse(void *mx, size_t dim1, size_t dim2, void *result)
{
    /*
     * Computes the least squares inverse.
     */
    float *m_result = (float *)result;
    float mxTranspose[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    float mxGrammian[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    float mxGrammianInverse[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    
    mTranspose(mx, dim1, dim2, mxTranspose);
    mMultM(mxTranspose, dim2, dim1, mx, dim1, dim2, mxGrammian);
    mInverse(mxGrammian, dim2, mxGrammianInverse);
    mMultM(mxGrammianInverse, dim2, dim2, mxTranspose, dim2, dim1, m_result);

}

void mMinimumNormInverse(void *mx, size_t dim1, size_t dim2, void *result)
{
    /*
     * Computes the minumum norm inverse.
     */
    float *m_mx = (float *)mx;
    float *m_result = (float *)result;
    float mxTranspose[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    float mxMxTranspose[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    float mxMxTransposeInverse[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    
    mTranspose(m_mx, dim1, dim2, mxTranspose);
    mMultM(m_mx, dim1, dim2, mxTranspose, dim2, dim1, mxMxTranspose);
    mInverse(mxMxTranspose, dim1, mxMxTransposeInverse);
    mMultM(mxTranspose, dim2, dim1, mxMxTransposeInverse, dim1, dim1, m_result);
}

void mCopy(void *mx, size_t dim1, size_t dim2,
           void *result)
{
    float *m_mx = (float *)mx;
    float *m_result = (float *)result;

    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            m_result[MXINDEX(dim2, i, j)] = m_mx[MXINDEX(dim2, i, j)];
        }
    }
}

void mSetZero(void *result, size_t dim1, size_t dim2)
{
    float *m_result = (float *)result;

    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            m_result[MXINDEX(dim2, i, j)] = 0.0;
        }
    }
}

void mSetIdentity(void *result, size_t dim1, size_t dim2)
{
    float *m_result = (float *)result;

    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            m_result[MXINDEX(dim2, i, j)] = (i == j) ? 1.0 : 0.0;
        }
    }
}

void mDiag(void *v, size_t dim, void *result)
{
    float *m_v = (float *)v;
    float *m_result = (float *)result;

    size_t i;
    size_t j;
    for(i = 0; i < dim; i++) {
        for(j = 0; j < dim; j++) {
            m_result[MXINDEX(dim, i, j)] = (i == j) ? m_v[i] : 0.0;
        }
    }
}

void mTranspose(void *mx, size_t dim1, size_t dim2,
                void *result)
{
    float *m_mx = (float *)mx;
    float m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    if (dim1*dim2 > LINEAR_ALGEBRA_MAX_ARRAY_SIZE)
    {
        BSK_PRINT(MSG_ERROR,"Linear Algegra library array dimension input is too large.");
    }

    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            m_result[MXINDEX(dim1, j, i)] = m_mx[MXINDEX(dim2, i, j)];
        }
    }

    MOVE_FLOAT(m_result, dim2 * dim1, result);
}

void mAdd(void *mx1, size_t dim1, size_t dim2,
          void *mx2,
          void *result)
{
    float *m_mx1 = (float *)mx1;
    float *m_mx2 = (float *)mx2;
    float *m_result = (float *)result;

    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            m_result[MXINDEX(dim2, i, j)] = m_mx1[MXINDEX(dim2, i, j)] + m_mx2[MXINDEX(dim2, i, j)];
        }
    }
}

void mSubtract(void *mx1, size_t dim1, size_t dim2,
               void *mx2,
               void *result)
{
    float *m_mx1 = (float *)mx1;
    float *m_mx2 = (float *)mx2;
    float *m_result = (float *)result;

    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            m_result[MXINDEX(dim2, i, j)] = m_mx1[MXINDEX(dim2, i, j)] - m_mx2[MXINDEX(dim2, i, j)];
        }
    }
}

void mScale(float scaleFactor,
            void *mx, size_t dim1, size_t dim2,
            void *result)
{
    float *m_mx = (float *)mx;
    float *m_result = (float *)result;

    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            m_result[MXINDEX(dim2, i, j)] = scaleFactor * m_mx[MXINDEX(dim2, i, j)];
        }
    }
}

void mMultM(void *mx1, size_t dim11, size_t dim12,
            void *mx2, size_t dim21, size_t dim22,
            void *result)
{
    float *m_mx1 = (float *)mx1;
    float *m_mx2 = (float *)mx2;
    float m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    if (dim11*dim22 > LINEAR_ALGEBRA_MAX_ARRAY_SIZE)
    {
        BSK_PRINT(MSG_ERROR,"Linear Algegra library array dimension input is too large.");
    }

    size_t i;
    size_t j;
    size_t k;
    if(dim12 != dim21) {
        BSK_PRINT(MSG_ERROR, "Error: mMultM dimensions don't match.");
        return;
    }
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[MXINDEX(dim22, i, j)] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[MXINDEX(dim22, i, j)] += m_mx1[MXINDEX(dim12, i, k)] * m_mx2[MXINDEX(dim22, k, j)];
            }
        }
    }

    MOVE_FLOAT(m_result, dim11 * dim22, result);
}

void mtMultM(void *mx1, size_t dim11, size_t dim12,
             void *mx2, size_t dim21, size_t dim22,
             void *result)
{
    float *m_mx1 = (float *)mx1;
    float *m_mx2 = (float *)mx2;
    float m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    if (dim12*dim22 > LINEAR_ALGEBRA_MAX_ARRAY_SIZE)
    {
        BSK_PRINT(MSG_ERROR,"Linear Algegra library array dimension input is too large.");
    }

    size_t i;
    size_t j;
    size_t k;
    if(dim11 != dim21) {
        BSK_PRINT(MSG_ERROR, "Error: mtMultM dimensions don't match.");
        return;
    }
    for(i = 0; i < dim12; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[MXINDEX(dim22, i, j)] = 0.0;
            for(k = 0; k < dim11; k++) {
                m_result[MXINDEX(dim22, i, j)] += m_mx1[MXINDEX(dim12, k, i)] * m_mx2[MXINDEX(dim22, k, j)];
            }
        }
    }

    MOVE_FLOAT(m_result, dim12 * dim22, result);
}

void mMultMt(void *mx1, size_t dim11, size_t dim12,
             void *mx2, size_t dim21, size_t dim22,
             void *result)
{
    float *m_mx1 = (float *)mx1;
    float *m_mx2 = (float *)mx2;
    float m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    if (dim11*dim21 > LINEAR_ALGEBRA_MAX_ARRAY_SIZE)
    {
        BSK_PRINT(MSG_ERROR,"Linear Algegra library array dimension input is too large.");
    }

    size_t i;
    size_t j;
    size_t k;
    if(dim12 != dim22) {
        BSK_PRINT(MSG_ERROR, "Error: mMultMt dimensions don't match.");
        return;
    }
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim21; j++) {
            m_result[MXINDEX(dim21, i, j)] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[MXINDEX(dim21, i, j)] += m_mx1[MXINDEX(dim12, i, k)] * m_mx2[MXINDEX(dim22, j, k)];
            }
        }
    }

    MOVE_FLOAT(m_result, dim11 * dim21, result);
}

void mtMultMt(void *mx1, size_t dim11, size_t dim12,
              void *mx2, size_t dim21, size_t dim22,
              void *result)
{
    float *m_mx1 = (float *)mx1;
    float *m_mx2 = (float *)mx2;
    float m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    if (dim12*dim21 > LINEAR_ALGEBRA_MAX_ARRAY_SIZE)
    {
        BSK_PRINT(MSG_ERROR,"Linear Algegra library array dimension input is too large.");
    }

    size_t i;
    size_t j;
    size_t k;
    if(dim11 != dim22) {
        BSK_PRINT(MSG_ERROR, "Error: mtMultMt dimensions don't match.");
        return;
    }
    for(i = 0; i < dim12; i++) {
        for(j = 0; j < dim21; j++) {
            m_result[MXINDEX(dim21, i, j)] = 0.0;
            for(k = 0; k < dim11; k++) {
                m_result[MXINDEX(dim21, i, j)] += m_mx1[MXINDEX(dim12, k, i)] * m_mx2[MXINDEX(dim22, j, k)];
            }
        }
    }

    MOVE_FLOAT(m_result, dim12 * dim21, result);
}

void mMultV(void *mx, size_t dim1, size_t dim2,
            void *v,
            void *result)
{
    size_t dim11 = dim1;
    size_t dim12 = dim2;
    size_t dim22 = 1;
    float *m_mx1 = (float *)mx;
    float *m_mx2 = (float *)v;
    float m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    if (dim11*dim22 > LINEAR_ALGEBRA_MAX_ARRAY_SIZE)
    {
        BSK_PRINT(MSG_ERROR,"Linear Algegra library array dimension input is too large.");
    }

    size_t i;
    size_t j;
    size_t k;
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[MXINDEX(dim22, i, j)] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[MXINDEX(dim22, i, j)] += m_mx1[MXINDEX(dim12, i, k)] * m_mx2[MXINDEX(dim22, k, j)];
            }
        }
    }

    MOVE_FLOAT(m_result, dim11 * dim22, result);
}

void mtMultV(void *mx, size_t dim1, size_t dim2,
             void *v,
             void *result)
{
    size_t dim11 = dim1;
    size_t dim12 = dim2;
    size_t dim22 = 1;
    float *m_mx1 = (float *)mx;
    float *m_mx2 = (float *)v;
    float m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    if (dim12*dim22 > LINEAR_ALGEBRA_MAX_ARRAY_SIZE)
    {
        BSK_PRINT(MSG_ERROR,"Linear Algegra library array dimension input is too large.");
    }

    size_t i;
    size_t j;
    size_t k;
    for(i = 0; i < dim12; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[MXINDEX(dim22, i, j)] = 0.0;
            for(k = 0; k < dim11; k++) {
                m_result[MXINDEX(dim22, i, j)] += m_mx1[MXINDEX(dim12, k, i)] * m_mx2[MXINDEX(dim22, k, j)];
            }
        }
    }

    MOVE_FLOAT(m_result, dim12 * dim22, result);
}

float mTrace(void *mx, size_t dim)
{
    float *m_mx = (float *)mx;

    size_t i;
    float result = 0.0;
    for(i = 0; i < dim; i++) {
        result += m_mx[MXINDEX(dim, i, i)];
    }

    return result;
}

float mDeterminant(void *mx, size_t dim)
{
    float *m_mx = (float *)mx;

    size_t i;
    size_t j;
    size_t k;
    size_t ii;
    float result = 0;
    float mxTemp[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    if ((dim-1)*(dim-1) > LINEAR_ALGEBRA_MAX_ARRAY_SIZE)
    {
        BSK_PRINT(MSG_ERROR,"Linear Algegra library array dimension input is too large.");
    }

    if(dim < 1) {
        return 0;
    } else if(dim == 1) {
        result = m_mx[MXINDEX(dim, 0, 0)];
    } else if(dim == 2) {
        result = m_mx[MXINDEX(dim, 0, 0)] * m_mx[MXINDEX(dim, 1, 1)]
                 - m_mx[MXINDEX(dim, 1, 0)] * m_mx[MXINDEX(dim, 0, 1)];
    } else {
        for(k = 0; k < dim; k++) {
            for(i = 1; i < dim; i++) {
                ii = 0;
                for(j = 0; j < dim; j++) {
                    if(j == k) {
                        continue;
                    }
                    mxTemp[MXINDEX(dim - 1, i - 1, ii)] = m_mx[MXINDEX(dim, i, j)];
                    ii++;
                }
            }
            result += pow(-1.0, 1.0 + k + 1.0) * m_mx[MXINDEX(dim, 0, k)] * mDeterminant(mxTemp, dim - 1);
        }
    }
    return(result);
}

void mCofactor(void *mx, size_t dim, void *result)
{
    /* The (j,i)th cofactor of A is defined as (-1)^(i + j)*det(A_(i,j))
       where A_(i,j) is the submatrix of A obtained from A by removing the ith row and jth column */
    size_t  i;
    size_t  i0;
    size_t  i1;
    size_t  j;
    size_t  j0;
    size_t  j1;
    float *m_mx = (float *)mx;
    float m_mxij[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    float m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    float  det;
    if (dim*dim > LINEAR_ALGEBRA_MAX_ARRAY_SIZE)
    {
        BSK_PRINT(MSG_ERROR,"Linear Algegra library array dimension input is too large.");
    }

    for(i = 0; i < dim; i++) {
        for(j = 0; j < dim; j++) {
            /* Form mx_(i,j) */
            i1 = 0;
            for(i0 = 0; i0 < dim; i0++) {
                if(i0 == i) {
                    continue;
                }
                j1 = 0;
                for(j0 = 0; j0 < dim; j0++) {
                    if(j0 == j) {
                        continue;
                    }
                    m_mxij[MXINDEX(dim - 1, i1, j1)] = m_mx[MXINDEX(dim, i0, j0)];
                    j1++;
                }
                i1++;
            }

            /* Calculate the determinant */
            det = mDeterminant(m_mxij, dim - 1);

            /* Fill in the elements of the cofactor */
            m_result[MXINDEX(dim, i, j)] = pow(-1.0, i + j + 2.0) * det;
        }
    }

    MOVE_FLOAT(m_result, dim * dim, result);
}

int mInverse(void *mx, size_t dim, void *result)
{
    /* Inverse of a square matrix A with non zero determinant is adjoint matrix divided by determinant */
    /* The adjoint matrix is the square matrix X such that the (i,j)th entry of X is the (j,i)th cofactor of A */

    size_t  i;
    size_t  j;
    int     status = 0;
    float  det = mDeterminant(mx, dim);
    float  m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    if (dim*dim > LINEAR_ALGEBRA_MAX_ARRAY_SIZE)
    {
        BSK_PRINT(MSG_ERROR,"Linear Algegra library array dimension input is too large.");
    }
    
    if(fabs(det) > DB0_EPS) {
        /* Find adjoint matrix */
        float m_adjoint[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
        mCofactor(mx, dim, m_adjoint);
        mTranspose(m_adjoint, dim, dim, m_adjoint);
        /* Find inverse */
        mScale(1.0 / det, m_adjoint, dim, dim, m_result);
    } else {
        BSK_PRINT(MSG_ERROR, "Error: cannot invert singular matrix");
        for(i = 0; i < dim; i++) {
            for(j = 0; j < dim; j++) {
                m_result[MXINDEX(dim, i, j)] = 0.0;
            }
        }
        status = 1;
    }

    MOVE_FLOAT(m_result, dim * dim, result);
    return status;
}

int mIsEqual(void *mx1, size_t dim1, size_t dim2,
             void *mx2,
             float accuracy)
{
    float *m_mx1 = (float *)mx1;
    float *m_mx2 = (float *)mx2;

    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            if(fabs(m_mx1[MXINDEX(dim2, i, j)] - m_mx2[MXINDEX(dim2, i, j)]) > accuracy) {
                return 0;
            }
        }
    }
    return 1;
}

int mIsZero(void *mx, size_t dim1, size_t dim2,
            float accuracy)
{
    float *m_mx = (float *)mx;

    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            if(fabs(m_mx[MXINDEX(dim2, i, j)]) > accuracy) {
                return 0;
            }
        }
    }
    return 1;
}

void mPrintScreen(const char *name, void *mx, size_t dim1, size_t dim2)
{
    float *m_mx = (float *)mx;

    size_t i;
    size_t j;
    printf("%s = [", name);
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            printf("%20.15g", m_mx[MXINDEX(dim2, i, j)]);
            if(j != dim2 - 1) {
                printf(", ");
            }
        }
        if(i != dim1 - 1) {
            printf(";\n");
        }
    }
    printf("];\n");
}


void mPrint(FILE *pFile, const char *name, void *mx, size_t dim1, size_t dim2)
{
    float *m_mx = (float *)mx;

    size_t i;
    size_t j;
    fprintf(pFile, "%s = [", name);
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            fprintf(pFile, "%20.15g", m_mx[MXINDEX(dim2, i, j)]);
            if(j != dim2 - 1) {
                fprintf(pFile, ", ");
            }
        }
        if(i != dim1 - 1) {
            fprintf(pFile, ";\n");
        }
    }
    fprintf(pFile, "];\n");
}

void mGetSubMatrix(void *mx, size_t dim1, size_t dim2,
                   size_t dim1Start, size_t dim2Start,
                   size_t dim1Result, size_t dim2Result, void *result)
{
    float *m_mx = (float *)mx;
    float *m_result = (float *)result;

    size_t i;
    size_t j;
    for(i = dim1Start; i < dim1Start + dim1Result; i++) {
        for(j = dim2Start; j < dim2Start + dim2Result; j++) {
            m_result[MXINDEX(dim2Result, i - dim1Start, j - dim2Start)] = m_mx[MXINDEX(dim2, i, j)];
        }
    }
}

void mSetSubMatrix(void *mx, size_t dim1, size_t dim2,
                   void *result, size_t dim1Result, size_t dim2Result,
                   size_t dim1Start, size_t dim2Start)
{
    float *m_mx = (float *)mx;
    float *m_result = (float *)result;

    size_t i;
    size_t j;
    for(i = dim1Start; i < dim1Start + dim1; i++) {
        for(j = dim2Start; j < dim2Start + dim2; j++) {
            m_result[MXINDEX(dim2Result, i, j)] = m_mx[MXINDEX(dim2, i - dim1Start, j - dim2Start)];
        }
    }
}

void m22Set(float m00, float m01,
            float m10, float m11,
            float m[2][2])
{
    m[0][0] = m00;
    m[0][1] = m01;
    m[1][0] = m10;
    m[1][1] = m11;
}

void m22Copy(float mx[2][2],
             float result[2][2])
{
    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = mx[i][j];
        }
    }
}

void m22SetZero(float result[2][2])
{
    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = 0.0;
        }
    }
}

void m22SetIdentity(float result[2][2])
{
    size_t dim = 2;
    size_t i;
    size_t j;
    for(i = 0; i < dim; i++) {
        for(j = 0; j < dim; j++) {
            result[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
}

void m22Transpose(float mx[2][2],
                  float result[2][2])
{
    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    float m_result[2][2];
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            m_result[j][i] = mx[i][j];
        }
    }
    m22Copy(m_result, result);
}

void m22Add(float mx1[2][2],
            float mx2[2][2],
            float result[2][2])
{
    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = mx1[i][j] + mx2[i][j];
        }
    }
}

void m22Subtract(float mx1[2][2],
                 float mx2[2][2],
                 float result[2][2])
{
    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = mx1[i][j] - mx2[i][j];
        }
    }
}

void m22Scale(float scaleFactor,
              float mx[2][2],
              float result[2][2])
{
    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = scaleFactor * mx[i][j];
        }
    }
}

void m22MultM22(float mx1[2][2],
                float mx2[2][2],
                float result[2][2])
{
    size_t dim11 = 2;
    size_t dim12 = 2;
    size_t dim22 = 2;
    size_t i;
    size_t j;
    size_t k;
    float m_result[2][2];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[i][j] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[i][k] * mx2[k][j];
            }
        }
    }
    m22Copy(m_result, result);
}

void m22tMultM22(float mx1[2][2],
                 float mx2[2][2],
                 float result[2][2])
{
    size_t dim11 = 2;
    size_t dim12 = 2;
    size_t dim22 = 2;
    size_t i;
    size_t j;
    size_t k;
    float m_result[2][2];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[i][j] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[k][i] * mx2[k][j];
            }
        }
    }
    m22Copy(m_result, result);
}

void m22MultM22t(float mx1[2][2],
                 float mx2[2][2],
                 float result[2][2])
{
    size_t dim11 = 2;
    size_t dim12 = 2;
    size_t dim21 = 2;
    size_t i;
    size_t j;
    size_t k;
    float m_result[2][2];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim21; j++) {
            m_result[i][j] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[i][k] * mx2[j][k];
            }
        }
    }
    m22Copy(m_result, result);
}

void m22MultV2(float mx[2][2],
               float v[2],
               float result[2])
{
    size_t dim11 = 2;
    size_t dim12 = 2;
    size_t dim22 = 1;
    size_t i;
    size_t j;
    size_t k;
    float m_result[2];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[i] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[i] += mx[i][k] * v[k];
            }
        }
    }
    v2Copy(m_result, result);
}

void m22tMultV2(float mx[2][2],
                float v[2],
                float result[2])
{
    size_t dim11 = 2;
    size_t dim12 = 2;
    size_t dim22 = 1;
    size_t i;
    size_t j;
    size_t k;
    float m_result[2];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[i] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[i] += mx[k][i] * v[k];
            }
        }
    }
    v2Copy(m_result, result);
}

float m22Trace(float mx[2][2])
{
    size_t dim = 2;
    size_t i;
    float result = 0.0;
    for(i = 0; i < dim; i++) {
        result += mx[i][i];
    }

    return result;
}

float m22Determinant(float mx[2][2])
{
    float value;
    value = mx[0][0] * mx[1][1] - mx[1][0] * mx[0][1];
    return value;
}

int m22IsEqual(float mx1[2][2],
               float mx2[2][2],
               float accuracy)
{
    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            if(fabs(mx1[i][j] - mx2[i][j]) > accuracy) {
                return 0;
            }
        }
    }
    return 1;
}

int m22IsZero(float mx[2][2],
              float accuracy)
{
    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            if(fabs(mx[i][j]) > accuracy) {
                return 0;
            }
        }
    }
    return 1;
}

void m22Print(FILE *pFile, const char *name, float mx[2][2])
{
    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    fprintf(pFile, "%s = [", name);
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            fprintf(pFile, "%20.15g", mx[i][j]);
            if(j != dim2 - 1) {
                fprintf(pFile, ", ");
            }
        }
        if(i != dim1 - 1) {
            fprintf(pFile, ";\n");
        }
    }
    fprintf(pFile, "]\n");
}

int m22Inverse(float mx[2][2], float result[2][2])
{
    float det = m22Determinant(mx);
    float detInv;
    float m_result[2][2];
    int    status = 0;

    if(fabs(det) > DB0_EPS) {
        detInv = 1.0 / det;
        m_result[0][0] =  mx[1][1] * detInv;
        m_result[0][1] = -mx[0][1] * detInv;
        m_result[1][0] = -mx[1][0] * detInv;
        m_result[1][1] =  mx[0][0] * detInv;
    } else {
        BSK_PRINT(MSG_ERROR, "Error: singular 2x2 matrix inverse");
        m22Set(0.0, 0.0,
               0.0, 0.0,
               m_result);
        status = 1;
    }
    m22Copy(m_result, result);
    return status;
}

void    m22PrintScreen(const char *name, float mx[2][2])
{
    int i;
    printf("%s:\n", name);
    for (i=0;i<2;i++) {
        printf("%20.15g, %20.15g\n", mx[i][0], mx[i][1]);
    }
}

void m33Set(float m00, float m01, float m02,
            float m10, float m11, float m12,
            float m20, float m21, float m22,
            float m[3][3])
{
    m[0][0] = m00;
    m[0][1] = m01;
    m[0][2] = m02;
    m[1][0] = m10;
    m[1][1] = m11;
    m[1][2] = m12;
    m[2][0] = m20;
    m[2][1] = m21;
    m[2][2] = m22;
}

void m33Copy(float mx[3][3],
             float result[3][3])
{
    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = mx[i][j];
        }
    }
}

void m33SetZero(float result[3][3])
{
    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = 0.0;
        }
    }
}

void m33SetIdentity(float result[3][3])
{
    size_t dim = 3;
    size_t i;
    size_t j;
    for(i = 0; i < dim; i++) {
        for(j = 0; j < dim; j++) {
            result[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
}

void m33Transpose(float mx[3][3],
                  float result[3][3])
{
    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    float m_result[3][3];
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            m_result[j][i] = mx[i][j];
        }
    }
    m33Copy(m_result, result);
}

void m33Add(float mx1[3][3],
            float mx2[3][3],
            float result[3][3])
{
    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = mx1[i][j] + mx2[i][j];
        }
    }
}

void m33Subtract(float mx1[3][3],
                 float mx2[3][3],
                 float result[3][3])
{
    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = mx1[i][j] - mx2[i][j];
        }
    }
}

void m33Scale(float scaleFactor,
              float mx[3][3],
              float result[3][3])
{
    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = scaleFactor * mx[i][j];
        }
    }
}

void m33MultM33(float mx1[3][3],
                float mx2[3][3],
                float result[3][3])
{
    size_t dim11 = 3;
    size_t dim12 = 3;
    size_t dim22 = 3;
    size_t i;
    size_t j;
    size_t k;
    float m_result[3][3];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[i][j] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[i][k] * mx2[k][j];
            }
        }
    }
    m33Copy(m_result, result);
}

void m33tMultM33(float mx1[3][3],
                 float mx2[3][3],
                 float result[3][3])
{
    size_t dim11 = 3;
    size_t dim12 = 3;
    size_t dim22 = 3;
    size_t i;
    size_t j;
    size_t k;
    float m_result[3][3];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[i][j] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[k][i] * mx2[k][j];
            }
        }
    }
    m33Copy(m_result, result);
}

void m33MultM33t(float mx1[3][3],
                 float mx2[3][3],
                 float result[3][3])
{
    size_t dim11 = 3;
    size_t dim12 = 3;
    size_t dim21 = 3;
    size_t i;
    size_t j;
    size_t k;
    float m_result[3][3];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim21; j++) {
            m_result[i][j] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[i][k] * mx2[j][k];
            }
        }
    }
    m33Copy(m_result, result);
}

void m33MultV3(float mx[3][3],
               float v[3],
               float result[3])
{
    size_t dim11 = 3;
    size_t dim12 = 3;
    size_t dim22 = 1;
    size_t i;
    size_t j;
    size_t k;
    float m_result[3];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[i] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[i] += mx[i][k] * v[k];
            }
        }
    }
    v3Copy(m_result, result);
}

void m33tMultV3(float mx[3][3],
                float v[3],
                float result[3])
{
    size_t dim11 = 3;
    size_t dim12 = 3;
    size_t dim22 = 1;
    size_t i;
    size_t j;
    size_t k;
    float m_result[3];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[i] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[i] += mx[k][i] * v[k];
            }
        }
    }
    v3Copy(m_result, result);
}

float m33Trace(float mx[3][3])
{
    size_t dim = 3;
    size_t i;
    float result = 0.0;
    for(i = 0; i < dim; i++) {
        result += mx[i][i];
    }

    return result;
}

float m33Determinant(float mx[3][3])
{
    float value;
    value = mx[0][0] * mx[1][1] * mx[2][2]
            + mx[0][1] * mx[1][2] * mx[2][0]
            + mx[0][2] * mx[1][0] * mx[2][1]
            - mx[0][0] * mx[1][2] * mx[2][1]
            - mx[0][1] * mx[1][0] * mx[2][2]
            - mx[0][2] * mx[1][1] * mx[2][0];
    return value;
}

int m33IsEqual(float mx1[3][3],
               float mx2[3][3],
               float accuracy)
{
    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            if(fabs(mx1[i][j] - mx2[i][j]) > accuracy) {
                return 0;
            }
        }
    }
    return 1;
}

int m33IsZero(float mx[3][3],
              float accuracy)
{
    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            if(fabs(mx[i][j]) > accuracy) {
                return 0;
            }
        }
    }
    return 1;
}

void m33Print(FILE *pFile, const char *name, float mx[3][3])
{
    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    fprintf(pFile, "%s = [", name);
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            fprintf(pFile, "%20.15g", mx[i][j]);
            if(j != dim2 - 1) {
                fprintf(pFile, ", ");
            }
        }
        if(i != dim1 - 1) {
            fprintf(pFile, ";\n");
        }
    }
    fprintf(pFile, "]\n");
}

int m33Inverse(float mx[3][3], float result[3][3])
{
    float det = m33Determinant(mx);
    float detInv;
    float m_result[3][3];
    int    status = 0;

    if(fabs(det) > DB0_EPS) {
        detInv = 1.0 / det;
        m_result[0][0] = (mx[1][1] * mx[2][2] - mx[1][2] * mx[2][1]) * detInv;
        m_result[0][1] = -(mx[0][1] * mx[2][2] - mx[0][2] * mx[2][1]) * detInv;
        m_result[0][2] = (mx[0][1] * mx[1][2] - mx[0][2] * mx[1][1]) * detInv;
        m_result[1][0] = -(mx[1][0] * mx[2][2] - mx[1][2] * mx[2][0]) * detInv;
        m_result[1][1] = (mx[0][0] * mx[2][2] - mx[0][2] * mx[2][0]) * detInv;
        m_result[1][2] = -(mx[0][0] * mx[1][2] - mx[0][2] * mx[1][0]) * detInv;
        m_result[2][0] = (mx[1][0] * mx[2][1] - mx[1][1] * mx[2][0]) * detInv;
        m_result[2][1] = -(mx[0][0] * mx[2][1] - mx[0][1] * mx[2][0]) * detInv;
        m_result[2][2] = (mx[0][0] * mx[1][1] - mx[0][1] * mx[1][0]) * detInv;
    } else {
        BSK_PRINT(MSG_ERROR, "Error: singular 3x3 matrix inverse");
        m33Set(0.0, 0.0, 0.0,
               0.0, 0.0, 0.0,
               0.0, 0.0, 0.0,
               m_result);
        status = 1;
    }
    m33Copy(m_result, result);
    return status;
}

void m33SingularValues(float mx[3][3], float result[3])
{
    float sv[3];
    float a[3];
    float mxtmx[3][3];
    int    i;

    m33tMultM33(mx, mx, mxtmx);

    /* Compute characteristic polynomial */
    a[0] = -m33Determinant(mxtmx);
    a[1] = mxtmx[0][0] * mxtmx[1][1] - mxtmx[0][1] * mxtmx[1][0]
           + mxtmx[0][0] * mxtmx[2][2] - mxtmx[0][2] * mxtmx[2][0]
           + mxtmx[1][1] * mxtmx[2][2] - mxtmx[1][2] * mxtmx[2][1];
    a[2] = -mxtmx[0][0] - mxtmx[1][1] - mxtmx[2][2];

    /* Solve cubic equation */
    cubicRoots(a, sv);

    /* take square roots */
    for(i = 0; i < 3; i++) {
        sv[i] = sqrt(sv[i]);
    }

    /* order roots */
    v3Sort(sv, result);
}

void m33EigenValues(float mx[3][3], float result[3])
{
    float sv[3];
    float a[3];

    /* Compute characteristic polynomial */
    a[0] = -m33Determinant(mx);
    a[1] = mx[0][0] * mx[1][1] - mx[0][1] * mx[1][0]
           + mx[0][0] * mx[2][2] - mx[0][2] * mx[2][0]
           + mx[1][1] * mx[2][2] - mx[1][2] * mx[2][1];
    a[2] = -mx[0][0] - mx[1][1] - mx[2][2];

    /* Solve cubic equation */
    cubicRoots(a, sv);

    /* order roots */
    v3Sort(sv, result);
}

float m33ConditionNumber(float mx[3][3])
{
    float sv[3];
    m33SingularValues(mx, sv);
    return (sv[0] / sv[2]);
}

void    m33PrintScreen(const char *name, float mx[3][3])
{
    int i;
    printf("%s:\n", name);
    for (i=0;i<3;i++) {
        printf("%20.15g, %20.15g, %20.15g\n", mx[i][0], mx[i][1], mx[i][2]);
    }
}

void m44Set(float m00, float m01, float m02, float m03,
            float m10, float m11, float m12, float m13,
            float m20, float m21, float m22, float m23,
            float m30, float m31, float m32, float m33,
            float m[4][4])
{
    m[0][0] = m00;
    m[0][1] = m01;
    m[0][2] = m02;
    m[0][3] = m03;
    m[1][0] = m10;
    m[1][1] = m11;
    m[1][2] = m12;
    m[1][3] = m13;
    m[2][0] = m20;
    m[2][1] = m21;
    m[2][2] = m22;
    m[2][3] = m23;
    m[3][0] = m30;
    m[3][1] = m31;
    m[3][2] = m32;
    m[3][3] = m33;
}

void m44Copy(float mx[4][4],
             float result[4][4])
{
    size_t dim1 = 4;
    size_t dim2 = 4;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = mx[i][j];
        }
    }
}

void m44SetZero(float result[4][4])
{
    size_t dim1 = 4;
    size_t dim2 = 4;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = 0.0;
        }
    }
}

void m44MultV4(float mx[4][4],
               float v[4],
               float result[4])
{
    size_t dim11 = 4;
    size_t dim12 = 4;
    size_t dim22 = 1;
    size_t i;
    size_t j;
    size_t k;
    float m_result[4];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[i] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[i] += mx[i][k] * v[k];
            }
        }
    }
    v4Copy(m_result, result);
}

float m44Determinant(float mx[4][4])
{
    float value;
    value = mx[0][3] * mx[1][2] * mx[2][1] * mx[3][0]
            - mx[0][2] * mx[1][3] * mx[2][1] * mx[3][0]
            - mx[0][3] * mx[1][1] * mx[2][2] * mx[3][0]
            + mx[0][1] * mx[1][3] * mx[2][2] * mx[3][0]
            + mx[0][2] * mx[1][1] * mx[2][3] * mx[3][0]
            - mx[0][1] * mx[1][2] * mx[2][3] * mx[3][0]
            - mx[0][3] * mx[1][2] * mx[2][0] * mx[3][1]
            + mx[0][2] * mx[1][3] * mx[2][0] * mx[3][1]
            + mx[0][3] * mx[1][0] * mx[2][2] * mx[3][1]
            - mx[0][0] * mx[1][3] * mx[2][2] * mx[3][1]
            - mx[0][2] * mx[1][0] * mx[2][3] * mx[3][1]
            + mx[0][0] * mx[1][2] * mx[2][3] * mx[3][1]
            + mx[0][3] * mx[1][1] * mx[2][0] * mx[3][2]
            - mx[0][1] * mx[1][3] * mx[2][0] * mx[3][2]
            - mx[0][3] * mx[1][0] * mx[2][1] * mx[3][2]
            + mx[0][0] * mx[1][3] * mx[2][1] * mx[3][2]
            + mx[0][1] * mx[1][0] * mx[2][3] * mx[3][2]
            - mx[0][0] * mx[1][1] * mx[2][3] * mx[3][2]
            - mx[0][2] * mx[1][1] * mx[2][0] * mx[3][3]
            + mx[0][1] * mx[1][2] * mx[2][0] * mx[3][3]
            + mx[0][2] * mx[1][0] * mx[2][1] * mx[3][3]
            - mx[0][0] * mx[1][2] * mx[2][1] * mx[3][3]
            - mx[0][1] * mx[1][0] * mx[2][2] * mx[3][3]
            + mx[0][0] * mx[1][1] * mx[2][2] * mx[3][3];
    return value;
}

int m44IsEqual(float mx1[4][4],
               float mx2[4][4],
               float accuracy)
{
    size_t dim1 = 4;
    size_t dim2 = 4;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            if(fabs(mx1[i][j] - mx2[i][j]) > accuracy) {
                return 0;
            }
        }
    }
    return 1;
}

int m44Inverse(float mx[4][4], float result[4][4])
{
    float det = m44Determinant(mx);
    float detInv;
    float m_result[4][4];
    int    status = 0;

    if(fabs(det) > DB0_EPS) {
        detInv = 1.0 / det;
        m_result[0][0] = (mx[1][2] * mx[2][3] * mx[3][1] - mx[1][3] * mx[2][2] * mx[3][1] + mx[1][3] * mx[2][1] * mx[3][2] - mx[1][1] * mx[2][3] * mx[3][2] - mx[1][2] * mx[2][1] * mx[3][3] + mx[1][1] * mx[2][2] * mx[3][3]) * detInv;
        m_result[0][1] = (mx[0][3] * mx[2][2] * mx[3][1] - mx[0][2] * mx[2][3] * mx[3][1] - mx[0][3] * mx[2][1] * mx[3][2] + mx[0][1] * mx[2][3] * mx[3][2] + mx[0][2] * mx[2][1] * mx[3][3] - mx[0][1] * mx[2][2] * mx[3][3]) * detInv;
        m_result[0][2] = (mx[0][2] * mx[1][3] * mx[3][1] - mx[0][3] * mx[1][2] * mx[3][1] + mx[0][3] * mx[1][1] * mx[3][2] - mx[0][1] * mx[1][3] * mx[3][2] - mx[0][2] * mx[1][1] * mx[3][3] + mx[0][1] * mx[1][2] * mx[3][3]) * detInv;
        m_result[0][3] = (mx[0][3] * mx[1][2] * mx[2][1] - mx[0][2] * mx[1][3] * mx[2][1] - mx[0][3] * mx[1][1] * mx[2][2] + mx[0][1] * mx[1][3] * mx[2][2] + mx[0][2] * mx[1][1] * mx[2][3] - mx[0][1] * mx[1][2] * mx[2][3]) * detInv;
        m_result[1][0] = (mx[1][3] * mx[2][2] * mx[3][0] - mx[1][2] * mx[2][3] * mx[3][0] - mx[1][3] * mx[2][0] * mx[3][2] + mx[1][0] * mx[2][3] * mx[3][2] + mx[1][2] * mx[2][0] * mx[3][3] - mx[1][0] * mx[2][2] * mx[3][3]) * detInv;
        m_result[1][1] = (mx[0][2] * mx[2][3] * mx[3][0] - mx[0][3] * mx[2][2] * mx[3][0] + mx[0][3] * mx[2][0] * mx[3][2] - mx[0][0] * mx[2][3] * mx[3][2] - mx[0][2] * mx[2][0] * mx[3][3] + mx[0][0] * mx[2][2] * mx[3][3]) * detInv;
        m_result[1][2] = (mx[0][3] * mx[1][2] * mx[3][0] - mx[0][2] * mx[1][3] * mx[3][0] - mx[0][3] * mx[1][0] * mx[3][2] + mx[0][0] * mx[1][3] * mx[3][2] + mx[0][2] * mx[1][0] * mx[3][3] - mx[0][0] * mx[1][2] * mx[3][3]) * detInv;
        m_result[1][3] = (mx[0][2] * mx[1][3] * mx[2][0] - mx[0][3] * mx[1][2] * mx[2][0] + mx[0][3] * mx[1][0] * mx[2][2] - mx[0][0] * mx[1][3] * mx[2][2] - mx[0][2] * mx[1][0] * mx[2][3] + mx[0][0] * mx[1][2] * mx[2][3]) * detInv;
        m_result[2][0] = (mx[1][1] * mx[2][3] * mx[3][0] - mx[1][3] * mx[2][1] * mx[3][0] + mx[1][3] * mx[2][0] * mx[3][1] - mx[1][0] * mx[2][3] * mx[3][1] - mx[1][1] * mx[2][0] * mx[3][3] + mx[1][0] * mx[2][1] * mx[3][3]) * detInv;
        m_result[2][1] = (mx[0][3] * mx[2][1] * mx[3][0] - mx[0][1] * mx[2][3] * mx[3][0] - mx[0][3] * mx[2][0] * mx[3][1] + mx[0][0] * mx[2][3] * mx[3][1] + mx[0][1] * mx[2][0] * mx[3][3] - mx[0][0] * mx[2][1] * mx[3][3]) * detInv;
        m_result[2][2] = (mx[0][1] * mx[1][3] * mx[3][0] - mx[0][3] * mx[1][1] * mx[3][0] + mx[0][3] * mx[1][0] * mx[3][1] - mx[0][0] * mx[1][3] * mx[3][1] - mx[0][1] * mx[1][0] * mx[3][3] + mx[0][0] * mx[1][1] * mx[3][3]) * detInv;
        m_result[2][3] = (mx[0][3] * mx[1][1] * mx[2][0] - mx[0][1] * mx[1][3] * mx[2][0] - mx[0][3] * mx[1][0] * mx[2][1] + mx[0][0] * mx[1][3] * mx[2][1] + mx[0][1] * mx[1][0] * mx[2][3] - mx[0][0] * mx[1][1] * mx[2][3]) * detInv;
        m_result[3][0] = (mx[1][2] * mx[2][1] * mx[3][0] - mx[1][1] * mx[2][2] * mx[3][0] - mx[1][2] * mx[2][0] * mx[3][1] + mx[1][0] * mx[2][2] * mx[3][1] + mx[1][1] * mx[2][0] * mx[3][2] - mx[1][0] * mx[2][1] * mx[3][2]) * detInv;
        m_result[3][1] = (mx[0][1] * mx[2][2] * mx[3][0] - mx[0][2] * mx[2][1] * mx[3][0] + mx[0][2] * mx[2][0] * mx[3][1] - mx[0][0] * mx[2][2] * mx[3][1] - mx[0][1] * mx[2][0] * mx[3][2] + mx[0][0] * mx[2][1] * mx[3][2]) * detInv;
        m_result[3][2] = (mx[0][2] * mx[1][1] * mx[3][0] - mx[0][1] * mx[1][2] * mx[3][0] - mx[0][2] * mx[1][0] * mx[3][1] + mx[0][0] * mx[1][2] * mx[3][1] + mx[0][1] * mx[1][0] * mx[3][2] - mx[0][0] * mx[1][1] * mx[3][2]) * detInv;
        m_result[3][3] = (mx[0][1] * mx[1][2] * mx[2][0] - mx[0][2] * mx[1][1] * mx[2][0] + mx[0][2] * mx[1][0] * mx[2][1] - mx[0][0] * mx[1][2] * mx[2][1] - mx[0][1] * mx[1][0] * mx[2][2] + mx[0][0] * mx[1][1] * mx[2][2]) * detInv;
    } else {
        BSK_PRINT(MSG_ERROR, "Error: singular 4x4 matrix inverse");
        m44Set(0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0,
               m_result);
        status = 1;
    }
    m44Copy(m_result, result);
    return status;
}

void m66Set(float m00, float m01, float m02, float m03, float m04, float m05,
            float m10, float m11, float m12, float m13, float m14, float m15,
            float m20, float m21, float m22, float m23, float m24, float m25,
            float m30, float m31, float m32, float m33, float m34, float m35,
            float m40, float m41, float m42, float m43, float m44, float m45,
            float m50, float m51, float m52, float m53, float m54, float m55,
            float m[6][6])
{
    m[0][0] = m00;
    m[0][1] = m01;
    m[0][2] = m02;
    m[0][3] = m03;
    m[0][4] = m04;
    m[0][5] = m05;
    m[1][0] = m10;
    m[1][1] = m11;
    m[1][2] = m12;
    m[1][3] = m13;
    m[1][4] = m14;
    m[1][5] = m15;
    m[2][0] = m20;
    m[2][1] = m21;
    m[2][2] = m22;
    m[2][3] = m23;
    m[2][4] = m24;
    m[2][5] = m25;
    m[3][0] = m30;
    m[3][1] = m31;
    m[3][2] = m32;
    m[3][3] = m33;
    m[3][4] = m34;
    m[3][5] = m35;
    m[4][0] = m40;
    m[4][1] = m41;
    m[4][2] = m42;
    m[4][3] = m43;
    m[4][4] = m44;
    m[4][5] = m45;
    m[5][0] = m50;
    m[5][1] = m51;
    m[5][2] = m52;
    m[5][3] = m53;
    m[5][4] = m54;
    m[5][5] = m55;
}

void m66Copy(float mx[6][6],
             float result[6][6])
{
    size_t dim1 = 6;
    size_t dim2 = 6;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = mx[i][j];
        }
    }
}

void m66SetZero(float result[6][6])
{
    size_t dim1 = 6;
    size_t dim2 = 6;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = 0.0;
        }
    }
}

void m66SetIdentity(float result[6][6])
{
    size_t dim = 6;
    size_t i;
    size_t j;
    for(i = 0; i < dim; i++) {
        for(j = 0; j < dim; j++) {
            result[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
}

void m66Transpose(float mx[6][6],
                  float result[6][6])
{
    size_t dim1 = 6;
    size_t dim2 = 6;
    size_t i;
    size_t j;
    float m_result[6][6];
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            m_result[j][i] = mx[i][j];
        }
    }
    m66Copy(m_result, result);
}

void m66Get33Matrix(size_t row, size_t col,
                    float m[6][6],
                    float mij[3][3])
{
    size_t i;
    size_t j;
    /* WARNING! Doesn't error check that row = 0,1,N-1 and col = 0,1,N-1 */
    for(i = 0; i < 3; i++) {
        for(j = 0; j < 3; j++) {
            mij[i][j] = m[row * 3 + i][col * 3 + j];
        }
    }
}

void m66Set33Matrix(size_t row, size_t col,
                    float mij[3][3],
                    float m[6][6])
{
    size_t i;
    size_t j;
    /* WARNING! Doesn't error check that row = 0,1,N-1 and col = 0,1,N-1 */
    for(i = 0; i < 3; i++) {
        for(j = 0; j < 3; j++) {
            m[row * 3 + i][col * 3 + j] = mij[i][j];
        }
    }
}

void m66Scale(float scaleFactor,
              float mx[6][6],
              float result[6][6])
{
    size_t dim1 = 6;
    size_t dim2 = 6;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = scaleFactor * mx[i][j];
        }
    }
}

void m66Add(float mx1[6][6],
            float mx2[6][6],
            float result[6][6])
{
    size_t dim1 = 6;
    size_t dim2 = 6;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = mx1[i][j] + mx2[i][j];
        }
    }
}

void m66Subtract(float mx1[6][6],
                 float mx2[6][6],
                 float result[6][6])
{
    size_t dim1 = 6;
    size_t dim2 = 6;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = mx1[i][j] - mx2[i][j];
        }
    }
}

void m66MultM66(float mx1[6][6],
                float mx2[6][6],
                float result[6][6])
{
    size_t dim11 = 6;
    size_t dim12 = 6;
    size_t dim22 = 6;
    size_t i;
    size_t j;
    size_t k;
    float m_result[6][6];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[i][j] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[i][k] * mx2[k][j];
            }
        }
    }
    m66Copy(m_result, result);
}

void m66tMultM66(float mx1[6][6],
                 float mx2[6][6],
                 float result[6][6])
{
    size_t dim11 = 6;
    size_t dim12 = 6;
    size_t dim22 = 6;
    size_t i;
    size_t j;
    size_t k;
    float m_result[6][6];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[i][j] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[k][i] * mx2[k][j];
            }
        }
    }
    m66Copy(m_result, result);
}

void m66MultM66t(float mx1[6][6],
                 float mx2[6][6],
                 float result[6][6])
{
    size_t dim11 = 6;
    size_t dim12 = 6;
    size_t dim21 = 6;
    size_t i;
    size_t j;
    size_t k;
    float m_result[6][6];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim21; j++) {
            m_result[i][j] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[i][k] * mx2[j][k];
            }
        }
    }
    m66Copy(m_result, result);
}

void m66MultV6(float mx[6][6],
               float v[6],
               float result[6])
{
    size_t dim11 = 6;
    size_t dim12 = 6;
    size_t dim22 = 1;
    size_t i;
    size_t j;
    size_t k;
    float m_result[6];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[i] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[i] += mx[i][k] * v[k];
            }
        }
    }
    v6Copy(m_result, result);
}

void m66tMultV6(float mx[6][6],
                float v[6],
                float result[6])
{
    size_t dim11 = 6;
    size_t dim12 = 6;
    size_t dim22 = 1;
    size_t i;
    size_t j;
    size_t k;
    float m_result[6];
    for(i = 0; i < dim11; i++) {
        for(j = 0; j < dim22; j++) {
            m_result[i] = 0.0;
            for(k = 0; k < dim12; k++) {
                m_result[i] += mx[k][i] * v[k];
            }
        }
    }
    v6Copy(m_result, result);
}

int m66IsEqual(float mx1[6][6],
               float mx2[6][6],
               float accuracy)
{
    size_t dim1 = 6;
    size_t dim2 = 6;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            if(fabs(mx1[i][j] - mx2[i][j]) > accuracy) {
                return 0;
            }
        }
    }
    return 1;
}

int m66IsZero(float mx[6][6],
              float accuracy)
{
    size_t dim1 = 6;
    size_t dim2 = 6;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            if(fabs(mx[i][j]) > accuracy) {
                return 0;
            }
        }
    }
    return 1;
}

void m99SetZero(float result[9][9])
{
    size_t dim1 = 9;
    size_t dim2 = 9;
    size_t i;
    size_t j;
    for(i = 0; i < dim1; i++) {
        for(j = 0; j < dim2; j++) {
            result[i][j] = 0.0;
        }
    }
}

void cubicRoots(float a[3], float result[3])
{
    /* Solve cubic formula for x^3 + a[2]*x^2 + a[1]*x + a[0] = 0 */
    /* see http://mathworld.wolfram.com/CubicFormula.html */
    float a2sq = a[2] * a[2];
    float a2d3 = a[2] / 3.0;
    float Q = (3.0 * a[1] - a2sq) / 9.0;
    float R = (a[2] * (9 * a[1] - 2 * a2sq) - 27 * a[0]) / 54.0;
    float RdsqrtnQ3 = R / sqrt(-Q * Q * Q);

    if(Q < 0.0 && fabs(RdsqrtnQ3) < 1.0) {
        /* A = 2*sqrt(-Q)
         * B = a[2]/3
         * result[0] = Acos(t) - B
         * result[1] = Acos(t + 2pi/3) - B = A*(cos(t)*-0.5 - sin(t)*sqrt(3)*0.5) - B
         * result[1] = Acos(t + 4pi/3) - B = A*(cos(t)*-0.5 + sin(t)*sqrt(3)*0.5) - B */
        float A = 2.0 * sqrt(-Q);
        float td3 = safeAcos(RdsqrtnQ3) / 3.0;
        float costd3 = cos(td3);
        float sintd3 = sin(td3);
        float sqrt3d2 = sqrt(3) * 0.5;
        float temp1 = -0.5 * costd3;
        float temp2 = sqrt3d2 * sintd3;

        result[0] = A * costd3 - a2d3;
        result[1] = A * (temp1 - temp2) - a2d3;
        result[2] = A * (temp1 + temp2) - a2d3;
    } else {
        float D = Q * Q * Q + R * R;
        float sqrtD = sqrt(D);
        float S = cbrt(R + sqrtD);
        float T = cbrt(R - sqrtD);

        result[0] = -a2d3 + (S + T);
        result[1] = -a2d3 - 0.5 * (S + T);
        result[2] = result[1];
    }

}

float safeAcos (float x) {
    if (x < -1.0)
        return acos(-1);
    else if (x > 1.0)
        return acos(1) ;
    return acos (x) ;
}

float safeAsin (float x) {
    if (x < -1.0)
        return asin(-1);
    else if (x > 1.0)
        return asin(1) ;
    return asin (x) ;
}

float safeSqrt(float x) {
    if (x < 0.0)
        return 0.0;
    return sqrt(x);
}

