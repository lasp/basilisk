// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _SICP_H_
#define _SICP_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/PointCloudMsgPayload.h>
#include <architecture/msgPayloadDef/SICPMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenMRP.h>
#include <architecture/utilities/eigenSupport.h>

#include <mission/parameters.h>
#include <stdint.h>

#include <Eigen/Dense>

/*! @brief Scaling iterative Closest Point Algorithm */
class ScalingIterativeClosestPoint : public SysModel {
public:
    ScalingIterativeClosestPoint();
    ~ScalingIterativeClosestPoint();

    void updateState(uint64_t currentSimNanos) override;
    void reset(uint64_t currentSimNanos) override;

    Message<PointCloudMsgPayload> outputPointCloud;         //!< The output fitted point cloud
    Message<SICPMsgPayload> outputSICPData;                 //!< The output algorithm data
    ReadFunctor<SICPMsgPayload> initialCondition;           //!< The input measured data
    ReadFunctor<PointCloudMsgPayload> measuredPointCloud;   //!< The input measured data
    ReadFunctor<PointCloudMsgPayload> referencePointCloud;  //!< The input reference data
    BSKLogger bskLogger;                                    //!< -- BSK Logging

    double scalingMax = 1.1;                  //!< Scaling maximums
    double scalingMin = 0.9;                  //!< Scaling minimums
    double errorTolerance = 1e-10;            //!< Error tolerance for convergence
    int maxIterations = MAX_SICP_ITERATIONS;  //!< Max iterations
    int numberScalePoints = 100;              //!< Number of points in order to find the scale factor

    //!< Initial conditions that could be set by user to better start off the SICP apgorithm
    Eigen::MatrixXd R_init = Eigen::MatrixXd::Identity(SICP_POINT_DIM, SICP_POINT_DIM);
    Eigen::MatrixXd t_init = Eigen::VectorXd::Zero(SICP_POINT_DIM);
    double s_init = 1;

private:
    void computePointCorrespondance(
        Eigen::MatrixXd const &R_kmin1,
        Eigen::MatrixXd const &t_kmin1,
        double const s_kmin1,
        Eigen::MatrixXd const &measuredPoints,
        Eigen::MatrixXd const &referencePoints
    );
    void centerCloud(Eigen::MatrixXd const &measuredPoints);
    Eigen::MatrixXd computeRk(double const s_kmin1, Eigen::MatrixXd const &R_kmin1);
    double computeSk(Eigen::MatrixXd const &R_kmin1);
    Eigen::MatrixXd computeTk(double const s_k, Eigen::MatrixXd const &R_k, Eigen::MatrixXd const &measuredPoints);

    PointCloudMsgPayload outputCloudBuffer;
    PointCloudMsgPayload measuredCloudBuffer;
    PointCloudMsgPayload referenceCloudBuffer;
    SICPMsgPayload initialConditionBuffer;
    SICPMsgPayload sicpBuffer;

    Eigen::MatrixXd correspondingPoints;
    Eigen::MatrixXd q;
    Eigen::MatrixXd n;

    int Np = 0;                      //!< Number of detected points
    int maxInternalIterations = 10;  //!< Maximum iterations in the inner loop for scale factor and rotation
};

#endif
