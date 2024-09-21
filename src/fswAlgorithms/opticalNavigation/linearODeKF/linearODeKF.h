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


/*! @brief Top level structure for the flyby OD linear kalman filter.
 Used to estimate the spacecraft's inertial position relative to a body.
 */

#ifndef LINEAR_OD_EKF_H
#define LINEAR_OD_EKF_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/orbitalMotion.h"
#include "architecture/utilities/macroDefinitions.h"
#include "msgPayloadDef/NavTransMsgPayload.h"
#include "msgPayloadDef/OpNavUnitVecMsgPayload.h"
#include "msgPayloadDef/FilterMsgPayload.h"
#include "msgPayloadDef/FilterResidualsMsgPayload.h"

#include "fswAlgorithms/_GeneralModuleFiles/ekfInterface.h"
#include "fswAlgorithms/_GeneralModuleFiles/measurementModels.h"

class LinearODeKF: public EkfInterface {
public:
    LinearODeKF(FilterType type)
    : EkfInterface(type){};
    ~LinearODeKF() = default;

private:
    void customReset() final;
    void readFilterMeasurements() final;
    void writeOutputMessages(uint64_t CurrentSimNanos) final;
    static Eigen::MatrixXd measurementMatrix(const FilterStateVector &state);

public:
    ReadFunctor<OpNavUnitVecMsgPayload> opNavHeadingMsg;
    OpNavUnitVecMsgPayload opNavHeadingBuffer;
    Message<NavTransMsgPayload> navTransOutMsg;
    Message<FilterMsgPayload> opNavFilterMsg;
    Message<FilterResidualsMsgPayload> opNavResidualMsg;

    void setConstantVelocity(const Eigen::Vector3d &velocity);
    std::optional<Eigen::Vector3d> getConstantVelocity() const ;

private:

    std::optional<Eigen::Vector3d> constantVelocity; //!< Unestimated constant velocity
    std::optional<Eigen::Vector3d> constantVelocityInitial; //!< Initial value of constant velocity
};

#endif
