/*
 ISC License

 Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef prescribedRotationSimMsg_h
#define prescribedRotationSimMsg_h


 /*! @brief Structure used to define the prescribed motion state effector rotational state data message */
typedef struct {
    double omega_FM_F[3];                      //!< [rad/s] Angular velocity of the F frame wrt the M frame in F frame components
    double omegaPrime_FM_F[3];                 //!< [rad/s^2] B/M frame time derivative of omega_FM_F
    double sigma_FM[3];                        //!< MRP attitude parameters for the F frame relative to the M frame
}PrescribedRotationMsgPayload;


#endif /* prescribedRotationSimMsg_h */
