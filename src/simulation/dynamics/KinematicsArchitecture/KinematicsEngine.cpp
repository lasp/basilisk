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

#include "KinematicsEngine.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <utility>

KinematicsEngine::KinematicsEngine() {
    auto originPoint = this->createPoint();
    this->rootFrame = std::make_shared<Frame>();
    this->rootFrame->originPoint = originPoint;
}

KinematicsEngine::~KinematicsEngine() {
    this->frameList.clear();
    this->pointList.clear();
    this->translationList.clear();
    this->rotationList.clear();
}

std::shared_ptr<Point> KinematicsEngine::createPoint() {
    auto tempPoint = std::make_shared<Point>();
    this->pointList.push_back(tempPoint);

    return tempPoint;
}

std::shared_ptr<Frame> KinematicsEngine::createFrame() {
    return createFrame(this->rootFrame);
}

std::shared_ptr<Frame> KinematicsEngine::createFrame(const std::shared_ptr<Frame>& parentFrame) {
    auto originPoint = this->createPoint();
    auto tempFrame = std::make_shared<Frame>(parentFrame, originPoint);
    auto transProperties = this->createTranslationProperties(originPoint, parentFrame->originPoint);
    auto rotProperties = this->createRotationProperties(tempFrame, parentFrame);

    // Required before setting so that these containers are not nullptrs
    tempFrame->r_SP = transProperties;
    tempFrame->sigma_SP = rotProperties;

    auto zeroVector = Vector(Eigen::Vector3d::Zero(), parentFrame);
    transProperties->setPosition(zeroVector);
    transProperties->setVelocity(zeroVector, parentFrame);
    transProperties->setAcceleration(zeroVector, parentFrame, parentFrame);
    zeroVector.set(Eigen::Vector3d::Zero(), tempFrame);
    rotProperties->setAngularVelocity(zeroVector);
    rotProperties->setAngularAcceleration(zeroVector, tempFrame);

    this->frameList.push_back(tempFrame);

    return tempFrame;
}

// We might want to overload this to get a frame as the nominal written/derivative instead of the root frame
std::shared_ptr<Translation> KinematicsEngine::createTranslationProperties(const std::shared_ptr<Point>& headPoint,
                                                                           const std::shared_ptr<Point>& tailPoint) {
    auto translationProperties = std::make_shared<Translation>(headPoint, tailPoint);

    auto zeroVector = Vector(Eigen::Vector3d::Zero(), this->rootFrame);
    translationProperties->setPosition(zeroVector);
    translationProperties->setVelocity(zeroVector, this->rootFrame);
    translationProperties->setAcceleration(zeroVector, this->rootFrame, this->rootFrame);

    this->translationList.push_back(translationProperties);

    return translationProperties;
}

// We might want to overload this to get a frame as the nominal written/derivative instead of the root frame
std::shared_ptr<Rotation> KinematicsEngine::createRotationProperties(const std::shared_ptr<Frame>& upperFrame,
                                                                     const std::shared_ptr<Frame>& lowerFrame) {
    auto rotationalProperties = std::make_shared<Rotation>(upperFrame, lowerFrame);

    auto zeroVector = Vector(Eigen::Vector3d::Zero(), this->rootFrame);
    Eigen::MRPd zeroAttitude;
    zeroAttitude = Eigen::Vector3d::Zero();
    rotationalProperties->setAttitude(zeroAttitude);
    rotationalProperties->setAngularVelocity(zeroVector);
    rotationalProperties->setAngularAcceleration(zeroVector, this->rootFrame);

    this->rotationList.push_back(rotationalProperties);

    return rotationalProperties;
}

InertiaTensor KinematicsEngine::createInertiaTensor(const std::shared_ptr<Point>& point) {
    return InertiaTensor(point);
}

std::vector<std::shared_ptr<Frame> > KinematicsEngine::findAbsolutePath(const std::shared_ptr<Frame>& frame) {
    auto intermediateFrame = frame;
    std::vector<std::shared_ptr<Frame>> path {frame};

    while(intermediateFrame->parentFrame != nullptr) {
        intermediateFrame = intermediateFrame->parentFrame;
        path.push_back(intermediateFrame);
    }
    std::reverse(path.begin(), path.end());

    return path;
}

std::pair<std::vector<const std::shared_ptr<Frame>>, std::vector<const std::shared_ptr<Frame>>> KinematicsEngine::findPath2LCA(const std::shared_ptr<Frame>&  upperFrame,
                                                                                                                               const std::shared_ptr<Frame>&  lowerFrame) {
    std::vector<const std::shared_ptr<Frame>> path2LCA1 {};
    std::vector<const std::shared_ptr<Frame>> path2LCA2 {};

    auto absolutePath1 = findAbsolutePath(upperFrame);
    auto absolutePath2 = findAbsolutePath(lowerFrame);

    int index = 0;
    for (int i = 0; i < absolutePath1.size() && i < absolutePath2.size(); i++) {
        if(absolutePath1.at(i) != absolutePath2.at(i)) {
            break;
        }
        index++;
    }

    absolutePath1.erase(absolutePath1.begin());
    absolutePath2.erase(absolutePath2.begin());

    for (int j = index-1; j < absolutePath1.size(); j++) {
        path2LCA1.push_back(absolutePath1.at(j));
    }
    for (int k = index-1; k < absolutePath2.size(); k++) {
        path2LCA2.push_back(absolutePath2.at(k));
    }

    return std::make_pair(path2LCA1, path2LCA2);
}

Eigen::MRPd KinematicsEngine::findIntermediateAttitude(const std::vector<const std::shared_ptr<Frame>>& path) {
    Eigen::MRPd pulledMRP;
    Eigen::Matrix3d transformDCM = Eigen::Matrix3d::Identity();

    for (int i = 0; i < path.size(); i++) {
        pulledMRP = path.at(i)->sigma_SP->getAttitude();
        transformDCM = pulledMRP.toRotationMatrix().transpose() * transformDCM;
    }

    Eigen::MRPd relativeMRP = eigenC2MRP(transformDCM);

    return relativeMRP;
}

Eigen::MRPd KinematicsEngine::findRelativeAttitude(const std::shared_ptr<Frame>& upperFrame, const std::shared_ptr<Frame>&  lowerFrame) {
    if (upperFrame != lowerFrame) {
        auto [path2LCA1, path2LCA2] = findPath2LCA(upperFrame, lowerFrame);

        if((!path2LCA1.empty() && !path2LCA2.empty())) {
            auto relativeMRP1 = findIntermediateAttitude(path2LCA1);
            auto relativeMRP2 = findIntermediateAttitude(path2LCA2);

            Eigen::Matrix3d transformDCM = relativeMRP1.toRotationMatrix().transpose() * relativeMRP2.toRotationMatrix();
            Eigen::MRPd relativeMRP = eigenC2MRP(transformDCM);

            return relativeMRP;
        }
        else if (path2LCA1.empty()) {
            Eigen::MRPd relativeMRP = findIntermediateAttitude(path2LCA2); // this is ridiculous
            Eigen::Matrix3d dcm = relativeMRP.toRotationMatrix();
            return eigenC2MRP(dcm);
        }
        else {
            return findIntermediateAttitude(path2LCA1);
        }
    }
    else{
        Eigen::MRPd relativeMRP;
        return relativeMRP.setIdentity(); // this is dumb
    }
}

Vector KinematicsEngine::findIntermediateAngularVelocity(const std::vector<const std::shared_ptr<Frame>>& path) {
    auto relativeAngularVelocity = path.at(0)->sigma_SP->getAngularVelocity();

    for (int i = 1; i < path.size(); i++) {
        relativeAngularVelocity += path.at(i)->sigma_SP->getAngularVelocity();
    }

    return relativeAngularVelocity;
}

Vector KinematicsEngine::findRelativeAngularVelocity(const std::shared_ptr<Frame>& upperFrame, const std::shared_ptr<Frame>&  lowerFrame) {
    if (upperFrame != lowerFrame) {
        auto [path2LCA1, path2LCA2] = findPath2LCA(upperFrame, lowerFrame);

        if (!path2LCA1.empty() && !path2LCA2.empty()) {
            auto relativeAngVel1 = findIntermediateAngularVelocity(path2LCA1);
            auto relativeAngVel2 = findIntermediateAngularVelocity(path2LCA2);
            return relativeAngVel1 - relativeAngVel2;
        }
        else if (path2LCA1.empty()) {
            return - findIntermediateAngularVelocity(path2LCA2);
        }
        else {
            return findIntermediateAngularVelocity(path2LCA1);
        }
    }
    else{
        return Vector(Eigen::Vector3d::Zero(), upperFrame);
    }
}

Vector KinematicsEngine::findIntermediateAngularAcceleration(const std::vector<const std::shared_ptr<Frame>>& path,
                                                             const std::shared_ptr<Frame>& derivFrame) {
    auto relativeAngularAcceleration = Vector(Eigen::Vector3d::Zero(), path.at(0)->sigma_SP->getAngularVelocity().getWrittenFrame()); // we might need to change this, not happy with it

    for (auto const& Frame: path) {
        relativeAngularAcceleration += Frame->sigma_SP->getAngularAcceleration(derivFrame);
    }

    return relativeAngularAcceleration;
}

Vector KinematicsEngine::findRelativeAngularAcceleration(const std::shared_ptr<Frame>& upperFrame,
                                                         const std::shared_ptr<Frame>& lowerFrame,
                                                         const std::shared_ptr<Frame>& derivFrame) {
    if (upperFrame != lowerFrame) {
        auto [path2LCA1, path2LCA2] = findPath2LCA(upperFrame, lowerFrame);

        if (!path2LCA1.empty() && !path2LCA2.empty()) {
            auto relativeAngAccel1 = findIntermediateAngularAcceleration(path2LCA1, derivFrame);
            auto relativeAngAccel2 = findIntermediateAngularAcceleration(path2LCA2, derivFrame);
            return relativeAngAccel1 - relativeAngAccel2;
        }
        else if (path2LCA1.empty()) {
            return - findIntermediateAngularAcceleration(path2LCA2, derivFrame);
        }
        else {
            return findIntermediateAngularAcceleration(path2LCA1, derivFrame);
        }
    }
    else{
        return Vector(Eigen::Vector3d::Zero(), upperFrame);
    }
}

Vector KinematicsEngine::findRelativePosition(std::shared_ptr<Point> headPoint,
                                              const std::shared_ptr<Point>& tailPoint) {
    auto returnVector = Vector(Eigen::Vector3d::Zero(), this->rootFrame);

    if (headPoint != tailPoint) {
        std::vector<const std::shared_ptr<const Translation>> visitedVectors = {};
        auto intermediateHeadPoint = headPoint;

        auto [path, val] = searchPointPath(headPoint, tailPoint, visitedVectors, visitedVectors, false);
        std::vector<const std::shared_ptr<const Translation>> translationPath(path);

        for (const auto& container: translationPath) {
            if (container->getHeadPoint() == intermediateHeadPoint) {
                returnVector += container->getPosition();
                intermediateHeadPoint = container->getTailPoint();
            } else {
                returnVector -= container->getPosition();
                intermediateHeadPoint = container->getHeadPoint();
            }
        }
    }
    return returnVector;
}

Vector KinematicsEngine::findRelativeVelocity(std::shared_ptr<Point> headPoint,
                                              const std::shared_ptr<Point>& tailPoint,
                                              const std::shared_ptr<Frame> derivFrame) {
    auto returnVector = Vector(Eigen::Vector3d::Zero(), this->rootFrame);

    if (headPoint != tailPoint) {
        std::vector<const std::shared_ptr<const Translation>> visitedVectors = {};
        auto intermediateHeadPoint = headPoint;

        auto [path, val] = searchPointPath(headPoint, tailPoint, visitedVectors, visitedVectors, false);
        std::vector<const std::shared_ptr<const Translation>> translationPath(path);

        for (const auto& container: translationPath) {
            if (container->getHeadPoint() == intermediateHeadPoint) {
                returnVector += container->getVelocity(derivFrame);
                intermediateHeadPoint = container->getTailPoint();
            } else {
                returnVector -= container->getVelocity(derivFrame);
                intermediateHeadPoint = container->getHeadPoint();
            }
        }
    }
    return returnVector;
}

std::pair<std::vector<const std::shared_ptr<const Translation>>, bool> KinematicsEngine::searchPointPath(std::shared_ptr<const Point> headPoint,
                                                                                                         const std::shared_ptr<const Point>& tailPoint, std::vector<const std::shared_ptr<const Translation>> translationPath,
                                                                                                         std::vector<const std::shared_ptr<const Translation>> visitedContainers, bool leafReached) {
    std::vector<const std::shared_ptr<const Translation>> transPath(translationPath);
    std::vector<const std::shared_ptr<const Translation>> commonContainers = {};

    for (const auto& transContainer: translationList) {
        if ((transContainer->getHeadPoint() == headPoint ||
             transContainer->getTailPoint() == headPoint) &&
            !(std::find(visitedContainers.begin(), visitedContainers.end(), transContainer) !=
              visitedContainers.end())) {
            commonContainers.push_back(transContainer);
        }
    }

    if (commonContainers.empty()) {
        leafReached = true;
    }

    for (const auto& container: commonContainers) {
        if (container->getHeadPoint() == headPoint) {
            headPoint = container->getTailPoint();
        } else {
            headPoint = container->getHeadPoint();
        }

        transPath.push_back(container);
        visitedContainers.push_back(container);

        if (headPoint == tailPoint) {
            return std::make_pair(transPath, false);
        }

        auto [path, leaf] = searchPointPath(headPoint, tailPoint, transPath, visitedContainers, leafReached);
        leafReached = leaf;

        transPath.clear();
        for (const auto& element: path) {
            transPath.push_back(element);
        }

        if (leafReached) {
            transPath.pop_back();
        }
    }
    return std::make_pair(transPath, leafReached);
}
