/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics,
 University of Colorado at Boulder

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

#ifndef FILTER_DYN_MODELS_H
#define FILTER_DYN_MODELS_H

#include <Eigen/Core>
#include "fswAlgorithms/_GeneralModuleFiles/stateModels.h"

/*! @brief Measurement models used to map a state vector to a measurement */
class DynamicsModel{
private:
    std::function<const FilterStateVector(const double, const FilterStateVector&)> propagator; //!< [-] state propagator using dynamics
    std::function<const Eigen::MatrixXd(const double, const FilterStateVector&)> dynamicsMatrix; //!< [-] partial of dynamics wrt state

    static FilterStateVector rk4(std::function<const FilterStateVector(const double, const FilterStateVector&)> ODEfunction,
            const FilterStateVector& X0,
            double t0,
            double dt) ;

public:
    DynamicsModel() = default;
    ~DynamicsModel() = default;

    FilterStateVector propagate(std::array<double, 2>, const FilterStateVector& state, double dt) const;
    void setDynamics(const std::function<const FilterStateVector(const double, const FilterStateVector&)>& dynamicsPropagator);

    Eigen::MatrixXd computeDynamicsMatrix(double time, const FilterStateVector& state) const;
    void setDynamicsMatrix(const std::function<const Eigen::MatrixXd(const double, const FilterStateVector&)>&
            dynamicsMatrixCalculator);
};

#endif
