// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _INITSICP_H_
#define _INITSICP_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CameraConfigMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/PointCloudMsgPayload.h>
#include <architecture/msgPayloadDef/SICPMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/rigidBodyKinematics.h>

#include <mission/parameters.h>
#include <stdint.h>

#include <Eigen/Dense>

/*! @brief Scaling iterative Closest Point Algorithm */
class InitializeICP : public SysModel {
public:
    InitializeICP();
    ~InitializeICP();

    void updateState(uint64_t currentSimNanos) override;
    void reset(uint64_t currentSimNanos) override;

    ReadFunctor<SICPMsgPayload> inputSICPData;                  //!< The output algorithm data
    ReadFunctor<EphemerisMsgPayload> ephemerisInMsg;            //!< ephemeris input message
    ReadFunctor<CameraConfigMsgPayload> cameraConfigInMsg;      //!< camera configuration input message
    ReadFunctor<PointCloudMsgPayload> inputMeasuredPointCloud;  //!< The input measured data
    Message<PointCloudMsgPayload> measuredPointCloud;           //!< The output fitted point cloud
    Message<SICPMsgPayload> initializeSICPMsg;                  //!< The output algorithm data

    BSKLogger bskLogger;  //!< -- BSK Logging

    double maxTimeBetweenMeasurements = 600;
    bool normalizeMeasuredCloud = false;

private:
    void normalizePointCloud();
    void setInitialConditions(uint64_t currentSimNanos);
    void writeOutputMessages(uint64_t currentSimNanos);

    PointCloudMsgPayload normalizedCloudBuffer;
    SICPMsgPayload outputIcpBuffer;
    double averageNorm = 0;
    Eigen::VectorXd averagePoint;
    Eigen::VectorXd referencePoint;
    bool initialPhase = true;

    //!< Logged results that will be used when this module is called again
    Eigen::MatrixXd R_logged = Eigen::MatrixXd::Identity(SICP_POINT_DIM, SICP_POINT_DIM);
    Eigen::MatrixXd t_logged = Eigen::VectorXd::Zero(SICP_POINT_DIM);
    double s_logged = 1;
    uint64_t previousTimeTag = 0;
};

#endif
