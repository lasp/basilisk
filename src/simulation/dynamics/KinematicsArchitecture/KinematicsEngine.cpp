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

KinematicsEngine::KinematicsEngine() {
    auto originPoint = this->createPoint();
    this->rootFrame = std::make_shared<Frame>();
    this->rootFrame->originPoint = originPoint;
}

KinematicsEngine::~KinematicsEngine() {
    this->frameList.clear();
    this->partList.clear();
    this->jointList.clear();
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

std::shared_ptr<Part> KinematicsEngine::createPart() {
    return this->createPart(this->rootFrame);
}

std::shared_ptr<Part> KinematicsEngine::createPart(const std::shared_ptr<Frame>& parentFrame) {
    auto CoMPoint = this->createPoint();
    auto tempFrame = this->createFrame(parentFrame);
    auto transProperties = this->createTranslationProperties(CoMPoint, tempFrame->originPoint);
    auto tempPart = std::make_shared<Part>(std::move(tempFrame)); // maybe add constructor for part with pos vec
    auto tempInertia = this->createInertiaTensor(CoMPoint);

    tempPart->r_ScS = transProperties;

    auto zeroVector = Vector(Eigen::Vector3d::Zero(), parentFrame);
    transProperties->setPosition(zeroVector);
    transProperties->setVelocity(zeroVector, parentFrame);
    transProperties->setAcceleration(zeroVector, parentFrame, parentFrame);
    tempInertia.set(Eigen::Matrix3d::Zero(), tempPart->frame);

    tempPart->CoMPoint = CoMPoint;
    tempPart->IPntSc_S = tempInertia;

    this->partList.push_back(tempPart);

    auto tempNode = std::make_shared<Node>(tempPart);
    this->nodeList.push_back(tempNode);

    return tempPart;
}

std::shared_ptr<RotaryOneDOF> KinematicsEngine::createRotaryOneDOFJoint() {
    auto equilibriumFrame = this->createFrame();
    auto currentFrame = this->createFrame(equilibriumFrame);
    auto tempHinge = std::make_shared<Hinge>(equilibriumFrame, currentFrame);

    auto tempJoint = std::make_shared<RotaryOneDOF>(tempHinge);
    this->jointList.push_back(tempJoint);

    return tempJoint;
}

std::shared_ptr<RotaryTwoDOF> KinematicsEngine::createRotaryTwoDOFJoint() {
    auto firstEquilibriumFrame = this->createFrame();
    auto firstCurrentFrame = this->createFrame(firstEquilibriumFrame);
    auto tempFirstHinge = std::make_shared<Hinge>(std::move(firstEquilibriumFrame), std::move(firstCurrentFrame));

    auto secondEquilibriumFrame = this->createFrame();
    auto secondCurrentFrame = this->createFrame(secondEquilibriumFrame);
    auto tempSecondHinge = std::make_shared<Hinge>(std::move(secondEquilibriumFrame), std::move(secondCurrentFrame));

    auto tempJoint = std::make_shared<RotaryTwoDOF>(tempFirstHinge, tempSecondHinge);
    this->jointList.push_back(tempJoint);

    return tempJoint;
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

std::shared_ptr<Assembly> KinematicsEngine::createAssembly() {
    return std::make_shared<Assembly>();
}

std::shared_ptr<Node> KinematicsEngine::findNode(const std::shared_ptr<Part>& part) {
    for (const auto& node: this->nodeList) {
        if (node->part == part)
            return node;
    }
    // Add an error message here, this cannot happen
    return nullptr;
}

void KinematicsEngine::connect(const std::shared_ptr<Part>& lowerPart,
                                const std::shared_ptr<Joint>& joint,
                                const std::shared_ptr<Part>& upperPart) {
    joint->lowerFrame->setParentFrame(lowerPart->frame);
    upperPart->frame->setParentFrame(joint->upperFrame);

    auto lowerNode = findNode(lowerPart);
    auto upperNode = findNode(upperPart);
    lowerNode->addEdge(upperNode, joint);
    upperNode->addEdge(lowerNode, joint);
}

void KinematicsEngine::writeOutputMessages(uint64_t callTime, std::shared_ptr<Frame> inertialFrame) {
    for (const auto& part: this->partList) {
        // - Populate state output message
        SCStatesMsgPayload stateOut;
        stateOut = part->bodyStateOutMsg.zeroMsgPayload;

        // Get attitude and angular velocity
        auto omega_BN = part->frame->sigma_SP->getAngularVelocity();
        auto sigma_BN = part->frame->sigma_SP->getAttitude();  // this conversion is so dumb

        // Get position and velocity from the center of mass
        auto r_CN = part->frame->r_SP->getPosition() + part->r_ScS->getPosition();
        auto rDot_CN = part->frame->r_SP->getVelocity(inertialFrame)
                + part->r_ScS->getVelocity(inertialFrame);

        eigenMatrixXd2CArray(r_CN.getMatrix(inertialFrame), stateOut.r_CN_N);
        eigenMatrixXd2CArray(rDot_CN.getMatrix(inertialFrame), stateOut.v_CN_N);
        eigenMatrixXd2CArray(eigenMRPd2Vector3d(sigma_BN), stateOut.sigma_BN);
        eigenMatrixXd2CArray(omega_BN.getMatrix(part->frame), stateOut.omega_BN_B);
        part->bodyStateOutMsg.write(&stateOut, 0, callTime);  // need to figure out moduleID
    }
}


void Node::addEdge(const std::shared_ptr<Node>& adjNode, const std::shared_ptr<Joint>& joint) {
    this->edgeList.emplace_back(adjNode, joint);
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

InertiaTensor KinematicsEngine::parallelAxisTheorem(const std::shared_ptr<Part>& part, const std::shared_ptr<Point>& point) {
    Vector relativePosition = findRelativePosition(part->IPntSc_S.getPoint() , point);

    Eigen::Matrix3d returnInertiaMatrix = part->IPntSc_S.getMatrix(relativePosition.getWrittenFrame()) +
                                          (part->mass * eigenTilde(relativePosition.getMatrix(relativePosition.getWrittenFrame())) * eigenTilde(relativePosition.getMatrix(relativePosition.getWrittenFrame())).transpose());

    InertiaTensor returnInertia = this->createInertiaTensor(point);
    returnInertia.set(returnInertiaMatrix, relativePosition.getWrittenFrame());

    return returnInertia;
}

double KinematicsEngine::getAssemblyMass(const std::shared_ptr<Assembly>& assembly) {
    double assemblyMass = 0.0;
    for (auto const& part: assembly->partList) {
        assemblyMass += part->mass;
    }

    return assemblyMass;
}

Vector KinematicsEngine::getAssemblyCOM(const std::shared_ptr<Assembly>& assembly, const std::shared_ptr<Point>& tailPoint) {
    double assemblyMass = getAssemblyMass(assembly);
    auto assemblyCOMPositionVector = Vector(Eigen::Vector3d::Zero(), nullptr);
    auto intermediatePosVec1 = Vector(Eigen::Vector3d::Zero(), this->rootFrame);
    Eigen::Vector3d intermediateVec;

    for (const auto& part: assembly->partList) {
        intermediatePosVec1 = findRelativePosition(part->r_ScS->getHeadPoint(), tailPoint);
        intermediatePosVec1 *= (part->mass / assemblyMass);

        if (assemblyCOMPositionVector.getWrittenFrame()) {
            assemblyCOMPositionVector += intermediatePosVec1;
        }
        else{
            assemblyCOMPositionVector = intermediatePosVec1;
        }
    }

    return assemblyCOMPositionVector;
}

InertiaTensor KinematicsEngine::getAssemblyInertia(const std::shared_ptr<Assembly>& assembly, const std::shared_ptr<Point>& point) {
    auto intermediateInertia = Tensor();
    for (int i = 0; i<assembly->partList.size(); i++) {
        auto partInertia = parallelAxisTheorem(assembly->partList.at(i), point);

        if (!(i == 0)) {
            intermediateInertia += partInertia;
        }
        else{
            intermediateInertia.set(partInertia.getMatrix(partInertia.getWrittenFrame()), partInertia.getWrittenFrame());
        }
    }

    auto assemblyInertia = InertiaTensor(point);
    assemblyInertia.set(intermediateInertia.getMatrix(intermediateInertia.getWrittenFrame()), intermediateInertia.getWrittenFrame());

    return assemblyInertia;
}
