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

#ifndef _RIGID_BODY_KINEMATICS_0_H_
#define _RIGID_BODY_KINEMATICS_0_H_
#include <architecture/utilities/bskLogging.h>

#ifdef __cplusplus
extern "C" {
#endif
    void   addEP(float *b1, float *b2, float *result);
    void   addEuler121(float *e1, float *e2, float *result);
    void   addEuler123(float *e1, float *e2, float *result);
    void   addEuler131(float *e1, float *e2, float *result);
    void   addEuler132(float *e1, float *e2, float *result);
    void   addEuler212(float *e1, float *e2, float *result);
    void   addEuler213(float *e1, float *e2, float *result);
    void   addEuler231(float *e1, float *e2, float *result);
    void   addEuler232(float *e1, float *e2, float *result);
    void   addEuler312(float *e1, float *e2, float *result);
    void   addEuler313(float *e1, float *e2, float *result);
    void   addEuler321(float *e1, float *e2, float *result);
    void   addEuler323(float *e1, float *e2, float *result);
    void   addGibbs(float *q1, float *q2, float *result);
    void   addMRP(float *q1, float *q2, float *result);
    void   addPRV(float *q1, float *q2, float *result);
    void   BinvEP(float *q, float B[3][4]);
    void   BinvEuler121(float *q, float B[3][3]);
    void   BinvEuler123(float *q, float B[3][3]);
    void   BinvEuler131(float *q, float B[3][3]);
    void   BinvEuler132(float *q, float B[3][3]);
    void   BinvEuler212(float *q, float B[3][3]);
    void   BinvEuler213(float *q, float B[3][3]);
    void   BinvEuler231(float *q, float B[3][3]);
    void   BinvEuler232(float *q, float B[3][3]);
    void   BinvEuler312(float *q, float B[3][3]);
    void   BinvEuler313(float *q, float B[3][3]);
    void   BinvEuler321(float *q, float B[3][3]);
    void   BinvEuler323(float *q, float B[3][3]);
    void   BinvGibbs(float *q, float B[3][3]);
    void   BinvMRP(float *q, float B[3][3]);
    void   BinvPRV(float *q, float B[3][3]);
    void   BmatEP(float *q, float B[4][3]);
    void   BmatEuler121(float *q, float B[3][3]);
    void   BmatEuler131(float *q, float B[3][3]);
    void   BmatEuler123(float *q, float B[3][3]);
    void   BmatEuler132(float *q, float B[3][3]);
    void   BmatEuler212(float *q, float B[3][3]);
    void   BmatEuler213(float *q, float B[3][3]);
    void   BmatEuler231(float *q, float B[3][3]);
    void   BmatEuler232(float *q, float B[3][3]);
    void   BmatEuler312(float *q, float B[3][3]);
    void   BmatEuler313(float *q, float B[3][3]);
    void   BmatEuler321(float *q, float B[3][3]);
    void   BmatEuler323(float *q, float B[3][3]);
    void   BmatGibbs(float *q, float B[3][3]);
    void   BmatMRP(float *q, float B[3][3]);
    void   BdotmatMRP(float *q, float *dq, float B[3][3]);
    void   BmatPRV(float *q, float B[3][3]);
    void   C2EP(float C[3][3], float b[4]);
    void   C2Euler121(float C[3][3], float *q);
    void   C2Euler123(float C[3][3], float *q);
    void   C2Euler131(float C[3][3], float *q);
    void   C2Euler132(float C[3][3], float *q);
    void   C2Euler212(float C[3][3], float *q);
    void   C2Euler213(float C[3][3], float *q);
    void   C2Euler231(float C[3][3], float *q);
    void   C2Euler232(float C[3][3], float *q);
    void   C2Euler312(float C[3][3], float *q);
    void   C2Euler313(float C[3][3], float *q);
    void   C2Euler321(float C[3][3], float *q);
    void   C2Euler323(float C[3][3], float *q);
    void   C2Gibbs(float C[3][3], float *q);
    void   C2MRP(float C[3][3], float *q);
    void   C2PRV(float C[3][3], float *q);
    void   dEP(float *q, float *w, float *dq);
    void   dEuler121(float *q, float *w, float *dq);
    void   dEuler123(float *q, float *w, float *dq);
    void   dEuler131(float *q, float *w, float *dq);
    void   dEuler132(float *q, float *w, float *dq);
    void   dEuler212(float *q, float *w, float *dq);
    void   dEuler213(float *q, float *w, float *dq);
    void   dEuler231(float *q, float *w, float *dq);
    void   dEuler232(float *q, float *w, float *dq);
    void   dEuler312(float *q, float *w, float *dq);
    void   dEuler313(float *q, float *w, float *dq);
    void   dEuler321(float *q, float *w, float *dq);
    void   dEuler323(float *q, float *w, float *dq);
    void   dGibbs(float *q, float *w, float *dq);
    void   dMRP(float *q, float *w, float *dq);
    void   dMRP2Omega(float *q, float *dq, float *w);
    void   ddMRP(float *q, float *dq, float *w, float *dw, float *ddq);
    void   ddMRP2dOmega(float *q, float *dq, float *ddq, float *dw);
    void   dPRV(float *q, float *w, float *dq);
    void   elem2PRV(float *r, float *q);
    void   EP2C(float *q, float C[3][3]);
    void   EP2Euler121(float *q, float *e);
    void   EP2Euler123(float *q, float *e);
    void   EP2Euler131(float *q, float *e);
    void   EP2Euler132(float *q, float *e);
    void   EP2Euler212(float *q, float *e);
    void   EP2Euler213(float *q, float *e);
    void   EP2Euler231(float *q, float *e);
    void   EP2Euler232(float *q, float *e);
    void   EP2Euler312(float *q, float *e);
    void   EP2Euler313(float *q, float *e);
    void   EP2Euler321(float *q, float *e);
    void   EP2Euler323(float *q, float *e);
    void   EP2Gibbs(float *q1, float *q);
    void   EP2MRP(float *q1, float *q);
    void   EP2PRV(float *q1, float *q);
    void   Euler1(float x, float m[3][3]);
    void   Euler2(float x, float m[3][3]);
    void   Euler3(float x, float m[3][3]);
    void   Euler1212C(float *q, float C[3][3]);
    void   Euler1212EP(float *e, float *q);
    void   Euler1212Gibbs(float *e, float *q);
    void   Euler1212MRP(float *e, float *q);
    void   Euler1212PRV(float *e, float *q);
    void   Euler1232C(float *q, float C[3][3]);
    void   Euler1232EP(float *e, float *q);
    void   Euler1232Gibbs(float *e, float *q);
    void   Euler1232MRP(float *e, float *q);
    void   Euler1232PRV(float *e, float *q);
    void   Euler1312C(float *q, float C[3][3]);
    void   Euler1312EP(float *e, float *q);
    void   Euler1312Gibbs(float *e, float *q);
    void   Euler1312MRP(float *e, float *q);
    void   Euler1312PRV(float *e, float *q);
    void   Euler1322C(float *q, float C[3][3]);
    void   Euler1322EP(float *e, float *q);
    void   Euler1322Gibbs(float *e, float *q);
    void   Euler1322MRP(float *e, float *q);
    void   Euler1322PRV(float *e, float *q);
    void   Euler2122C(float *q, float C[3][3]);
    void   Euler2122EP(float *e, float *q);
    void   Euler2122Gibbs(float *e, float *q);
    void   Euler2122MRP(float *e, float *q);
    void   Euler2122PRV(float *e, float *q);
    void   Euler2132C(float *q, float C[3][3]);
    void   Euler2132EP(float *e, float *q);
    void   Euler2132Gibbs(float *e, float *q);
    void   Euler2132MRP(float *e, float *q);
    void   Euler2132PRV(float *e, float *q);
    void   Euler2312C(float *q, float C[3][3]);
    void   Euler2312EP(float *e, float *q);
    void   Euler2312Gibbs(float *e, float *q);
    void   Euler2312MRP(float *e, float *q);
    void   Euler2312PRV(float *e, float *q);
    void   Euler2322C(float *q, float C[3][3]);
    void   Euler2322EP(float *e, float *q);
    void   Euler2322Gibbs(float *e, float *q);
    void   Euler2322MRP(float *e, float *q);
    void   Euler2322PRV(float *e, float *q);
    void   Euler3122C(float *q, float C[3][3]);
    void   Euler3122EP(float *e, float *q);
    void   Euler3122Gibbs(float *e, float *q);
    void   Euler3122MRP(float *e, float *q);
    void   Euler3122PRV(float *e, float *q);
    void   Euler3132C(float *q, float C[3][3]);
    void   Euler3132EP(float *e, float *q);
    void   Euler3132Gibbs(float *e, float *q);
    void   Euler3132MRP(float *e, float *q);
    void   Euler3132PRV(float *e, float *q);
    void   Euler3212C(float *q, float C[3][3]);
    void   Euler3212EP(float *e, float *q);
    void   Euler3212Gibbs(float *e, float *q);
    void   Euler3212MRP(float *e, float *q);
    void   Euler3212PRV(float *e, float *q);
    void   Euler3232C(float *q, float C[3][3]);
    void   Euler3232EP(float *e, float *q);
    void   Euler3232Gibbs(float *e, float *q);
    void   Euler3232MRP(float *e, float *q);
    void   Euler3232PRV(float *e, float *q);
    void   Gibbs2C(float *q, float C[3][3]);
    void   Gibbs2EP(float *q1, float *q);
    void   Gibbs2Euler121(float *q, float *e);
    void   Gibbs2Euler123(float *q, float *e);
    void   Gibbs2Euler131(float *q, float *e);
    void   Gibbs2Euler132(float *q, float *e);
    void   Gibbs2Euler212(float *q, float *e);
    void   Gibbs2Euler213(float *q, float *e);
    void   Gibbs2Euler231(float *q, float *e);
    void   Gibbs2Euler232(float *q, float *e);
    void   Gibbs2Euler312(float *q, float *e);
    void   Gibbs2Euler313(float *q, float *e);
    void   Gibbs2Euler321(float *q, float *e);
    void   Gibbs2Euler323(float *q, float *e);
    void   Gibbs2MRP(float *q1, float *q);
    void   Gibbs2PRV(float *q1, float *q);
    void   MRP2C(float *q, float C[3][3]);
    void   MRP2EP(float *q1, float *q);
    void   MRP2Euler121(float *q, float *e);
    void   MRP2Euler123(float *q, float *e);
    void   MRP2Euler131(float *q, float *e);
    void   MRP2Euler132(float *q, float *e);
    void   MRP2Euler212(float *q, float *e);
    void   MRP2Euler213(float *q, float *e);
    void   MRP2Euler231(float *q, float *e);
    void   MRP2Euler232(float *q, float *e);
    void   MRP2Euler312(float *q, float *e);
    void   MRP2Euler313(float *q, float *e);
    void   MRP2Euler321(float *q, float *e);
    void   MRP2Euler323(float *q, float *e);
    void   MRP2Gibbs(float *q1, float *q);
    void   MRP2PRV(float *q1, float *q);
    void   MRPswitch(float *q, float s2, float *s);
    void   MRPshadow(float *qIn, float *qOut);
    float wrapToPi(float x);
    void   PRV2C(float *q, float C[3][3]);
    void   PRV2elem(float *r, float *q);
    void   PRV2EP(float *q0, float *q);
    void   PRV2Euler121(float *q, float *e);
    void   PRV2Euler123(float *q, float *e);
    void   PRV2Euler131(float *q, float *e);
    void   PRV2Euler132(float *q, float *e);
    void   PRV2Euler212(float *q, float *e);
    void   PRV2Euler213(float *q, float *e);
    void   PRV2Euler231(float *q, float *e);
    void   PRV2Euler232(float *q, float *e);
    void   PRV2Euler312(float *q, float *e);
    void   PRV2Euler313(float *q, float *e);
    void   PRV2Euler321(float *q, float *e);
    void   PRV2Euler323(float *q, float *e);
    void   PRV2Gibbs(float *q0, float *q);
    void   PRV2MRP(float *q0, float *q);
    void   subEP(float *b1, float *b2, float *q);
    void   subEuler121(float *e, float *e1, float *e2);
    void   subEuler123(float *e, float *e1, float *e2);
    void   subEuler131(float *e, float *e1, float *e2);
    void   subEuler132(float *e, float *e1, float *e2);
    void   subEuler212(float *e, float *e1, float *e2);
    void   subEuler213(float *e, float *e1, float *e2);
    void   subEuler231(float *e, float *e1, float *e2);
    void   subEuler232(float *e, float *e1, float *e2);
    void   subEuler312(float *e, float *e1, float *e2);
    void   subEuler313(float *e, float *e1, float *e2);
    void   subEuler321(float *e, float *e1, float *e2);
    void   subEuler323(float *e, float *e1, float *e2);
    void   subGibbs(float *q1, float *q2, float *q);
    void   subMRP(float *q1, float *q2, float *q);
    void   subPRV(float *q10, float *q20, float *q);
    void   Mi(float angle, int axis, float C[3][3]);
    void   tilde(float *v, float mat[3][3]);
    
#ifdef __cplusplus
}
#endif

#endif
