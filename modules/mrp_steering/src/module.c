#include "module.h"

#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"

#include <math.h>
#include <string.h>

void MRPSteeringLaw(MrpSteering* config, double const sigma_BR[3], double omega_ast[3], double omega_ast_p[3]) {
    /* Equation (18): Determine the desired steering rates  */
    for (int i = 0; i < 3; i++) {
        double sigma_i  = sigma_BR[i];
        double value = atan(M_PI_2/config->omega_max*(config->K1*sigma_i
                     + config->K3*sigma_i*sigma_i*sigma_i))/M_PI_2*config->omega_max;
        omega_ast[i] = -value;
    }

    v3SetZero(omega_ast_p);

    if (!config->ignoreOuterLoopFeedforward) {
        /* Equation (21): Determine the body frame derivative of the steering rates */
        double  B[3][3];        /* B-matrix of MRP differential kinematic equations */
        BmatMRP((double*)sigma_BR, B);

        double  sigma_p[3];     /* MRP rates */
        m33MultV3(B, omega_ast, sigma_p);
        v3Scale(0.25, sigma_p, sigma_p);

        for (int i = 0; i < 3; i++) {
            double sigma_i  = sigma_BR[i];
            double value = (3*config->K3*sigma_i*sigma_i + config->K1)
                         / (pow(M_PI_2/config->omega_max*(config->K1*sigma_i + config->K3*sigma_i*sigma_i*sigma_i),2) + 1);
            omega_ast_p[i] = - value*sigma_p[i];
        }
    }
}
