/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef KINEMATICS_ENGINE_H
#define KINEMATICS_ENGINE_H

#include "simulation/dynamics/_GeneralModuleFiles/Frame.h"
#include "simulation/dynamics/_GeneralModuleFiles/Part.h"
#include "simulation/dynamics/_GeneralModuleFiles/Joint.h"
#include "simulation/dynamics/_GeneralModuleFiles/Point.h"
#include "simulation/dynamics/_GeneralModuleFiles/Assembly.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"
#include <iostream>

class KinematicsEngine {
public:
    KinematicsEngine();
    ~KinematicsEngine();

    std::shared_ptr<Frame> rootFrame = std::make_shared<Frame>();
    std::vector<std::shared_ptr<Frame>> frameList;
    std::vector<std::shared_ptr<Part>> partList;
    std::vector<std::shared_ptr<Joint>> jointList;
    std::vector<std::shared_ptr<Point>> pointList;
    std::vector<std::shared_ptr<PositionVector>> positionVectorList;
    std::vector<std::shared_ptr<AngularVelocityVector>> angularVelocityVectorList;

    std::shared_ptr<Point> createPoint();
    std::shared_ptr<Frame> createFrame();
    std::shared_ptr<Frame> createFrame(const std::shared_ptr<Frame>& parentFrame);
    std::shared_ptr<Part> createPart();
    std::shared_ptr<Part> createPart(const std::shared_ptr<Frame>& parentFrame);
    std::shared_ptr<RotaryOneDOF> createRotaryOneDOFJoint();
    std::shared_ptr<RotaryTwoDOF> createRotaryTwoDOFJoint();
    std::shared_ptr<PositionVector> createPositionVector(std::shared_ptr<Point> headPoint, std::shared_ptr<Point> tailPoint);
    std::shared_ptr<AngularVelocityVector> createAngularVelocityVector(std::shared_ptr<Frame> upperFrame, std::shared_ptr<Frame> lowerFrame);
    std::shared_ptr<InertiaTensor> createInertiaTensor(std::shared_ptr<Point> point);
    std::shared_ptr<Assembly> createAssembly();

    void connect(const std::shared_ptr<Part>& lowerPart, const std::shared_ptr<Joint>& joint, const std::shared_ptr<Part>& upperPart);
    static std::vector<std::shared_ptr<Frame>> findAbsolutePath(std::shared_ptr<Frame> frame);
    static std::pair<std::vector<std::shared_ptr<Frame>>, std::vector<std::shared_ptr<Frame>>> findPath2LCA(std::shared_ptr<Frame> upperFrame, std::shared_ptr<Frame> lowerFrame);
    static Eigen::MRPd findIntermediateAttitude(std::vector<std::shared_ptr<Frame>> path);
    static std::shared_ptr<AngularVelocityVector> findIntermediateAngularVelocity(std::vector<std::shared_ptr<Frame>> path, std::shared_ptr<Frame> lowerFrame);
    static Eigen::MRPd findRelativeAttitude(std::shared_ptr<Frame> upperFrame, std::shared_ptr<Frame> lowerFrame);
    std::shared_ptr<PositionVector> callFindRelativePosition(std::shared_ptr<Point> headPoint,
                                                             std::shared_ptr<Point> tailPoint);
    std::shared_ptr<AngularVelocityVector> callFindRelativeAngularVelocity(std::shared_ptr<Frame> upperFrame,
                                                                           std::shared_ptr<Frame> lowerFrame);
    std::shared_ptr<InertiaTensor> parallelAxisTheorem(std::shared_ptr<Part> part, std::shared_ptr<Point> point);

    double getAssemblyMass(std::shared_ptr<Assembly> assembly);
    std::shared_ptr<PositionVector> getAssemblyCOM(std::shared_ptr<Assembly> assembly, std::shared_ptr<Point> tailPoint);
    std::shared_ptr<InertiaTensor> getAssemblyInertia(std::shared_ptr<Assembly> assembly, std::shared_ptr<Point> point);

private:
    std::shared_ptr<PositionVector> findRelativePosition(std::shared_ptr<Point> headPoint,
                                                         std::shared_ptr<Point> tailPoint,
                                                         std::shared_ptr<Point> intermediateHeadPoint,
                                                         std::shared_ptr<PositionVector> intermediatePosVec,
                                                         std::vector<std::shared_ptr<PositionVector>> visitedVectors);
    std::shared_ptr<AngularVelocityVector> findRelativeAngularVelocity(std::shared_ptr<Frame> upperFrame,
                                                                       std::shared_ptr<Frame> lowerFrame,
                                                                       std::shared_ptr<Frame> intermediateUpperFrame,
                                                                       std::shared_ptr<AngularVelocityVector> intermediateAngVelVec,
                                                                       std::vector<std::shared_ptr<AngularVelocityVector>> visitedVectors);
};

#endif
