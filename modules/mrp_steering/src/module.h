#ifndef MRP_STEERING_MODULE_H
#define MRP_STEERING_MODULE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef struct {
    //! [rad/sec] Proportional gain applied to MRP errors
    double K1;
    //! [rad/sec] Cubic gain applied to MRP error in steering saturation function
    double K3;
    //! [rad/sec] Maximum rate command of steering control
    double omega_max;

    //! Boolean flag indicating if outer feedforward term should be included
    bool ignoreOuterLoopFeedforward;
} MrpSteering;

void MRPSteeringLaw(
    MrpSteering* config,
    double const sigma_BR[3],
    double omega_ast[3],
    double omega_ast_p[3]);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // MRP_STEERING_MODULE_H
