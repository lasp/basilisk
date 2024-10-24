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

#ifndef _COB_CONVERT_H_
#define _COB_CONVERT_H_

#include <stdint.h>
#include <Eigen/Dense>
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/messaging/messaging.h"

#include "architecture/msgPayloadDefC/CameraConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDefCpp/OpNavCOBMsgPayload.h"
#include "architecture/msgPayloadDefCpp/OpNavCOMMsgPayload.h"
#include "architecture/msgPayloadDefCpp/OpNavUnitVecMsgPayload.h"
#include "architecture/msgPayloadDefCpp/FilterMsgPayload.h"

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"

enum class PhaseAngleCorrectionMethod {NoCorrection, Lambertian, Binary};

/*! @brief visual limb finding module */
class CobConverter: public SysModel {
public:
    CobConverter(PhaseAngleCorrectionMethod method, double radiusObject);
    ~CobConverter();

    void UpdateState(uint64_t CurrentSimNanos);
    void Reset(uint64_t CurrentSimNanos);

    void setRadius(const double radius);
    double getRadius() const;
    void setAttitudeCovariance(const Eigen::Matrix3d covAtt_BN_B);
    Eigen::Matrix3d getAttitudeCovariance() const;
    void setNumStandardDeviations(const double num);
    double getNumStandardDeviations() const;
    void setStandardDeviation(const double num);
    double getStandardDeviation() const;
    bool isStandardDeviationSpecified() const;
    void enableOutlierDetection();
    void disableOutlierDetection();
    bool isOutlierDetectionEnabled() const;

private:
    bool cobOutlierDetection(Eigen::Vector3d& rhatCOB_C,
                             const Eigen::Vector3d& rhatNav_N,
                             const Eigen::Matrix3d& covarNav_N,
                             const Eigen::Matrix3d& covarCob_C,
                             const Eigen::Matrix3d& dcm_CN,
                             const Eigen::Matrix3d& dcm_CB,
                             const Eigen::Matrix3d& cameraCalibrationMatrix) const;

public:
    Message<OpNavUnitVecMsgPayload> opnavUnitVecCOBOutMsg;
    Message<OpNavUnitVecMsgPayload> opnavUnitVecCOMOutMsg;
    Message<OpNavCOMMsgPayload> opnavCOMOutMsg;
    ReadFunctor<OpNavCOBMsgPayload> opnavCOBInMsg;
    ReadFunctor<FilterMsgPayload> opnavFilterInMsg;
    ReadFunctor<CameraConfigMsgPayload> cameraConfigInMsg;
    ReadFunctor<NavAttMsgPayload> navAttInMsg;
    ReadFunctor<EphemerisMsgPayload> ephemInMsg;

    uint64_t sensorTimeTag;
    BSKLogger bskLogger;

private:
    PhaseAngleCorrectionMethod phaseAngleCorrectionMethod;
    double objectRadius{};
    Eigen::Matrix3d covarAtt_BN_B{};
    double numStandardDeviations = 3;
    double standardDeviation{};
    bool specifiedStandardDeviation{};
    bool performOutlierDetection{};
};

#endif
