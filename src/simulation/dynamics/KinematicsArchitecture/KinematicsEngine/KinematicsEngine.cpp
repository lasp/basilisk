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
#include <utility>

KinematicsEngine::KinematicsEngine() {
    auto originPoint = this->createPoint();
    this->rootFrame->originPoint = originPoint;
}

KinematicsEngine::~KinematicsEngine() {
    this->frameList.clear();
    this->partList.clear();
    this->jointList.clear();
    this->pointList.clear();
    this->positionVectorList.clear();
    this->angularVelocityVectorList.clear();
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
    auto posVec = this->createPositionVector(originPoint, parentFrame->originPoint);
    auto tempFrame = std::make_shared<Frame>(parentFrame);
    auto angVelVec = this->createAngularVelocityVector(tempFrame, parentFrame);

    tempFrame->originPoint = originPoint;
    tempFrame->omega_SP = angVelVec;
    tempFrame->omega_SP->writtenFrame = tempFrame;
    tempFrame->omega_SP->firstOrder.writtenFrame = tempFrame;
    tempFrame->omega_SP->firstOrder.derivFrame = tempFrame;
    tempFrame->r_SP = posVec;
    tempFrame->r_SP->writtenFrame = parentFrame;
    tempFrame->r_SP->firstOrder.writtenFrame = parentFrame;
    tempFrame->r_SP->firstOrder.derivFrame = parentFrame;
    tempFrame->r_SP->secondOrder.writtenFrame = parentFrame;
    tempFrame->r_SP->secondOrder.derivFrame = parentFrame;

    this->frameList.push_back(tempFrame);

    return tempFrame;
}


std::shared_ptr<Part> KinematicsEngine::createPart() {
    return this->createPart(this->rootFrame);
}

std::shared_ptr<Part> KinematicsEngine::createPart(const std::shared_ptr<Frame>& parentFrame) {
    auto CoMPoint = this->createPoint();
    auto tempFrame = this->createFrame(parentFrame);
    auto tempPosVec = this->createPositionVector(CoMPoint, tempFrame->originPoint);
    auto tempPart = std::make_shared<Part>(std::move(tempFrame)); // maybe add constructor for part with pos vec
    auto tempInertia = this->createInertiaTensor(CoMPoint);

    tempPart->r_ScS = tempPosVec;
    tempPart->r_ScS->writtenFrame = tempPart->frame;
    tempPart->IPntSc_S = tempInertia;
    tempPart->IPntSc_S->writtenFrame = tempPart->frame;

    this->partList.push_back(tempPart);

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

std::shared_ptr<PositionVector> KinematicsEngine::createPositionVector(std::shared_ptr<Point> headPoint, std::shared_ptr<Point> tailPoint) {
    auto posVec = std::make_shared<PositionVector>(headPoint, tailPoint);

    posVec->writtenFrame = this->rootFrame;

    this->positionVectorList.push_back(posVec);

    return posVec;
}

std::shared_ptr<AngularVelocityVector> KinematicsEngine::createAngularVelocityVector(std::shared_ptr<Frame> upperFrame, std::shared_ptr<Frame> lowerFrame) {
    auto angVelVec = std::make_shared<AngularVelocityVector>(upperFrame, lowerFrame);
    angVelVec->writtenFrame = upperFrame;

    this->angularVelocityVectorList.push_back(angVelVec);

    return angVelVec;
}

std::shared_ptr<InertiaTensor> KinematicsEngine::createInertiaTensor(std::shared_ptr<Point> point) {
    auto inertiaTensor = std::make_shared<InertiaTensor>(point);

    return inertiaTensor;
}

std::shared_ptr<Assembly> KinematicsEngine::createAssembly() {
    return std::make_shared<Assembly>();
}

void KinematicsEngine::connect(const std::shared_ptr<Part>& lowerPart, const std::shared_ptr<Joint>& joint,
                               const std::shared_ptr<Part>& upperPart) {
    joint->lowerFrame->setParentFrame(lowerPart->frame);
    upperPart->frame->setParentFrame(joint->upperFrame);
}

std::vector<std::shared_ptr<Frame>> KinematicsEngine::findAbsolutePath(std::shared_ptr<Frame> frame) {
    std::vector<std::shared_ptr<Frame>> path;
    std::shared_ptr<Frame> intermediateFrame = frame;

    while(intermediateFrame->parentFrame != nullptr) {
        path.push_back(intermediateFrame);
        intermediateFrame = intermediateFrame->parentFrame;
    }
    std::reverse(path.begin(), path.end());

    return path;
}

std::pair<std::vector<std::shared_ptr<Frame>>, std::vector<std::shared_ptr<Frame>>> KinematicsEngine::findPath2LCA(std::shared_ptr<Frame> upperFrame, std::shared_ptr<Frame> lowerFrame) {
    std::vector<std::shared_ptr<Frame>> absolutePath1;
    std::vector<std::shared_ptr<Frame>> absolutePath2;
    std::vector<std::shared_ptr<Frame>> path2LCA1;
    std::vector<std::shared_ptr<Frame>> path2LCA2;

    absolutePath1 = findAbsolutePath(upperFrame);
    absolutePath2 = findAbsolutePath(lowerFrame);

    int index = 0;
    while(index < absolutePath1.size() && index < absolutePath2.size()) {
        if(&absolutePath1.at(index) != &absolutePath2.at(index)) {
            index++;
            break;
        }
        index++;
    }

    for (int i = index-1; i<absolutePath1.size(); i++) {
        path2LCA1.push_back(absolutePath1.at(i));
    }
    for (int i = index-1; i<absolutePath2.size(); i++) {
        path2LCA2.push_back(absolutePath2.at(i));
    }

    std::pair<std::vector<std::shared_ptr<Frame>>, std::vector<std::shared_ptr<Frame>>> pair;
    pair.first = path2LCA1;
    pair.second = path2LCA2;

    return pair;
}

Eigen::MRPd KinematicsEngine::findIntermediateAttitude(std::vector<std::shared_ptr<Frame>> path) {
    Eigen::MRPd pulledMRP;
    Eigen::MRPd intermediateMRP;
    Eigen::MRPd relativeMRP;
    Eigen::Matrix3d pulledMRPDCM;
    Eigen::Matrix3d intermediateMRPDCM;
    Eigen::Matrix3d transformDCM;

    pulledMRP.setIdentity();
    intermediateMRP.setIdentity();
    relativeMRP.setIdentity();
    pulledMRPDCM.setIdentity(3,3);
    intermediateMRPDCM.setIdentity(3,3);
    transformDCM.setIdentity(3,3);

    for (int i = 1; i<path.size(); i++) {
        pulledMRP = path.at(i)->sigma_SP;
        pulledMRPDCM = (pulledMRP.toRotationMatrix()).transpose();
        intermediateMRPDCM = transformDCM;
        transformDCM = pulledMRPDCM * intermediateMRPDCM;
    }

    relativeMRP = eigenC2MRP(transformDCM);

    return relativeMRP;
}

std::shared_ptr<AngularVelocityVector> KinematicsEngine::findIntermediateAngularVelocity(std::vector<std::shared_ptr<Frame>> path, std::shared_ptr<Frame> lowerFrame) {
    Eigen::Vector3d relativeAngularVelocityVec = {0.0, 0.0, 0.0};

    for (int i = 1; i<path.size(); i++) {
        Eigen::MRPd relativeAttitude;
        relativeAttitude = findRelativeAttitude(path.at(i), lowerFrame);
        Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();
        relativeAngularVelocityVec = relativeAngularVelocityVec + dcm * path.at(i)->omega_SP->matrix;
    }

    auto relativeAngularVelocity = std::make_shared<AngularVelocityVector>();
    relativeAngularVelocity->matrix = relativeAngularVelocityVec;
    relativeAngularVelocity->writtenFrame = lowerFrame;

    return relativeAngularVelocity;
}

Eigen::MRPd KinematicsEngine::findRelativeAttitude(std::shared_ptr<Frame> upperFrame, std::shared_ptr<Frame> lowerFrame) {
    Eigen::MRPd relativeMRP1;
    Eigen::MRPd relativeMRP2;
    Eigen::MRPd relativeMRP;
    Eigen::Matrix3d relativeMRPDCM1;
    Eigen::Matrix3d relativeMRPDCM2;
    Eigen::Matrix3d transformDCM;

    relativeMRP1.setIdentity();
    relativeMRP2.setIdentity();
    relativeMRP.setIdentity();
    relativeMRPDCM1.setIdentity(3,3);
    relativeMRPDCM2.setIdentity(3,3);
    transformDCM.setIdentity(3,3);

    std::vector<std::shared_ptr<Frame>> path2LCA1;
    std::vector<std::shared_ptr<Frame>> path2LCA2;
    auto paths = findPath2LCA(upperFrame, lowerFrame);
    path2LCA1 = paths.first;
    path2LCA2 = paths.second;

    relativeMRP1 = findIntermediateAttitude(path2LCA1);
    relativeMRP2 = findIntermediateAttitude(path2LCA2);

    relativeMRPDCM1 = relativeMRP1.toRotationMatrix().transpose();
    relativeMRPDCM2 = relativeMRP2.toRotationMatrix();

    transformDCM = relativeMRPDCM1 * relativeMRPDCM2;
    relativeMRP = eigenC2MRP(transformDCM);

    return relativeMRP;
}

std::shared_ptr<AngularVelocityVector> KinematicsEngine::findRelativeAngularVelocity(std::shared_ptr<Frame> upperFrame, std::shared_ptr<Frame> lowerFrame) {
    std::vector<std::shared_ptr<Frame>> path2LCA1;
    std::vector<std::shared_ptr<Frame>> path2LCA2;
    auto paths = findPath2LCA(upperFrame, lowerFrame);
    path2LCA1 = paths.first;
    path2LCA2 = paths.second;

    auto relAngularVelocityVector1 = findIntermediateAngularVelocity(path2LCA1, lowerFrame);
    auto relAngularVelocityVector2 = findIntermediateAngularVelocity(path2LCA2, lowerFrame);

    Eigen::Vector3d relativeAngularVelocityVec = relAngularVelocityVector1->matrix + relAngularVelocityVector2->matrix;
    auto relativeAngularVelocity = std::make_shared<AngularVelocityVector>();
    relativeAngularVelocity->matrix = relativeAngularVelocityVec;
    relativeAngularVelocity->writtenFrame = relAngularVelocityVector2->writtenFrame;

    return relativeAngularVelocity;
}

std::shared_ptr<PositionVector> KinematicsEngine::addPositionVectors(std::shared_ptr<PositionVector> positionVector1, std::shared_ptr<PositionVector> positionVector2) {
    if (positionVector1->tailPoint == positionVector2->headPoint) {
        Eigen::MRPd relativeAttitude;
        relativeAttitude = findRelativeAttitude(positionVector1->writtenFrame.lock(), positionVector2->writtenFrame.lock());

        Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix();
        Eigen::Vector3d relPosition = dcm * positionVector1->matrix + positionVector2->matrix;

        auto relativePosition = std::make_shared<PositionVector>(positionVector1->headPoint, positionVector2->headPoint);
        relativePosition->matrix = relPosition;
        relativePosition->writtenFrame = positionVector2->writtenFrame;

        return relativePosition;
    }
    else{
        auto relativePosition = std::make_shared<PositionVector>();

        return relativePosition;
    }
}

std::shared_ptr<AngularVelocityVector> KinematicsEngine::addAngularVelocityVectors(std::shared_ptr<AngularVelocityVector> angularVelocityVector1, std::shared_ptr<AngularVelocityVector> angularVelocityVector2) {
    if (angularVelocityVector1->lowerFrame.lock() == angularVelocityVector2->upperFrame.lock()) {
        Eigen::MRPd relativeAttitude;
        relativeAttitude = findRelativeAttitude(angularVelocityVector1->writtenFrame.lock(), angularVelocityVector2->writtenFrame.lock());

        Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix();
        Eigen::Vector3d relAngularVelocity = dcm * angularVelocityVector1->matrix + angularVelocityVector2->matrix;

        auto relativeAngularVelocity = std::make_shared<AngularVelocityVector>(angularVelocityVector1->upperFrame,
                                                                               angularVelocityVector2->lowerFrame);
        relativeAngularVelocity->matrix = relAngularVelocity;
        relativeAngularVelocity->writtenFrame = angularVelocityVector2->writtenFrame;

        return relativeAngularVelocity;
    }
    else{
        auto relativeAngularVelocity = std::make_shared<AngularVelocityVector>();

        return relativeAngularVelocity;
    }
}

std::shared_ptr<PositionVector> KinematicsEngine::callFindRelativePosition(std::shared_ptr<Point> headPoint,
                                                                           std::shared_ptr<Point> tailPoint,
                                                                           std::shared_ptr<Frame> writtenFrame) {
    Eigen::Vector3d intermediatePosVec =  {0.0, 0.0, 0.0};
    std::vector<std::shared_ptr<PositionVector>> visitedVectors = {};

    return findRelativePosition(headPoint, tailPoint, headPoint, writtenFrame, intermediatePosVec,
                                visitedVectors);
}

std::shared_ptr<AngularVelocityVector> KinematicsEngine::callFindRelativeAngularVelocity(std::shared_ptr<Frame> upperFrame,
                                                                                 std::shared_ptr<Frame> lowerFrame,
                                                                                 std::shared_ptr<Frame> writtenFrame) {
    Eigen::Vector3d intermediatePosVec =  {0.0, 0.0, 0.0};
    std::vector<std::shared_ptr<AngularVelocityVector>> visitedVectors = {};

    return findRelativeAngularVelocity(upperFrame, lowerFrame, upperFrame, writtenFrame, intermediatePosVec,
                                       visitedVectors);
}

std::shared_ptr<PositionVector> KinematicsEngine::findRelativePosition(std::shared_ptr<Point> headPoint,
                                                           std::shared_ptr<Point> tailPoint,
                                                           std::shared_ptr<Point> intermediateHeadPoint,
                                                           std::shared_ptr<Frame> writtenFrame,
                                                           Eigen::Vector3d intermediatePosVec,
                                                           std::vector<std::shared_ptr<PositionVector>> visitedVectors) {
    auto returnVector = std::make_shared<PositionVector>();
    std::vector<std::shared_ptr<PositionVector>> commonVecs;

    for (int j = 0; j < this->positionVectorList.size(); j++) {
        if ((this->positionVectorList.at(j)->headPoint == intermediateHeadPoint ||
             this->positionVectorList.at(j)->tailPoint == intermediateHeadPoint) &&
            !(std::find(visitedVectors.begin(), visitedVectors.end(), this->positionVectorList.at(j)) !=
              visitedVectors.end())) {
            commonVecs.push_back(this->positionVectorList.at(j));
        }
    }

    Eigen::Vector3d relativePosition;
    for (int i = 0; i < commonVecs.size(); i++) {
        if (commonVecs.at(i)->headPoint == intermediateHeadPoint) {
            Eigen::MRPd relativeAttitude;
            relativeAttitude = findRelativeAttitude(writtenFrame, commonVecs.at(i)->writtenFrame.lock());

            Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();

            relativePosition = intermediatePosVec + dcm * commonVecs.at(i)->matrix;

            if (commonVecs.at(i)->tailPoint == tailPoint) {
                returnVector->matrix = relativePosition;
                returnVector->writtenFrame = writtenFrame;
                returnVector->headPoint = headPoint;
                returnVector->tailPoint = tailPoint;

                return returnVector;
            } else {
                std::shared_ptr<Point> desHead = commonVecs.at(i)->tailPoint;

                visitedVectors.push_back(commonVecs.at(i));

                returnVector = findRelativePosition(headPoint, tailPoint, desHead, writtenFrame,
                                                    relativePosition, visitedVectors);
            }
        }

        if (commonVecs.at(i)->tailPoint == intermediateHeadPoint) {
            Eigen::MRPd relativeAttitude;
            relativeAttitude = findRelativeAttitude(writtenFrame, commonVecs.at(i)->writtenFrame.lock());

            Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();

            relativePosition = intermediatePosVec - dcm * commonVecs.at(i)->matrix;

            if (commonVecs.at(i)->headPoint == tailPoint) {
                returnVector->matrix = relativePosition;
                returnVector->writtenFrame = writtenFrame;
                returnVector->headPoint = headPoint;
                returnVector->tailPoint = tailPoint;

                return returnVector;
            } else {
                std::shared_ptr<Point> desHead = commonVecs.at(i)->headPoint;

                visitedVectors.push_back(commonVecs.at(i));

                returnVector = findRelativePosition(headPoint, tailPoint, desHead, writtenFrame,
                                                    relativePosition, visitedVectors);
            }
        }

        if (returnVector->headPoint == headPoint && returnVector->tailPoint == tailPoint) {
            return returnVector;
        }
    }
    return returnVector;
}

std::shared_ptr<AngularVelocityVector> KinematicsEngine::findRelativeAngularVelocity(std::shared_ptr<Frame> upperFrame,
                                                 std::shared_ptr<Frame> lowerFrame,
                                                 std::shared_ptr<Frame> intermediateUpperFrame,
                                                 std::shared_ptr<Frame> writtenFrame,
                                                 Eigen::Vector3d intermediateAngVelVec,
                                                 std::vector<std::shared_ptr<AngularVelocityVector>> visitedVectors) {
    auto returnVector = std::make_shared<AngularVelocityVector>();
    std::vector<std::shared_ptr<AngularVelocityVector>> commonVecs;

    for (int j = 0; j < this->angularVelocityVectorList.size(); j++) {
        if ((this->angularVelocityVectorList.at(j)->upperFrame.lock() == intermediateUpperFrame ||
             this->angularVelocityVectorList.at(j)->lowerFrame.lock() == intermediateUpperFrame) &&
            !(std::find(visitedVectors.begin(), visitedVectors.end(), this->angularVelocityVectorList.at(j)) !=
              visitedVectors.end())) {
            commonVecs.push_back(this->angularVelocityVectorList.at(j));
        }
    }

    Eigen::Vector3d relativeAngularVelocity;
    for (int i = 0; i < commonVecs.size(); i++) {
        if (commonVecs.at(i)->upperFrame.lock() == intermediateUpperFrame) {
            Eigen::MRPd relativeAttitude;
            relativeAttitude = findRelativeAttitude(writtenFrame, commonVecs.at(i)->writtenFrame.lock());

            Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();

            relativeAngularVelocity = intermediateAngVelVec + dcm * commonVecs.at(i)->matrix;

            if (commonVecs.at(i)->lowerFrame.lock() == lowerFrame) {
                returnVector = std::make_shared<AngularVelocityVector>(upperFrame, lowerFrame);
                returnVector->matrix = relativeAngularVelocity;
                returnVector->writtenFrame = writtenFrame;

                return returnVector;
            } else {
                std::shared_ptr<Frame> desUpperFrame = commonVecs.at(i)->lowerFrame.lock();
                visitedVectors.push_back(commonVecs.at(i));

                returnVector = findRelativeAngularVelocity(upperFrame, lowerFrame, desUpperFrame, writtenFrame,relativeAngularVelocity, visitedVectors);
            }
        }

        if (commonVecs.at(i)->lowerFrame.lock() == intermediateUpperFrame) {
            Eigen::MRPd relativeAttitude;
            relativeAttitude = findRelativeAttitude(writtenFrame, commonVecs.at(i)->writtenFrame.lock());

            Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();

            relativeAngularVelocity = intermediateAngVelVec - dcm * commonVecs.at(i)->matrix;

            if (commonVecs.at(i)->upperFrame.lock() == lowerFrame) {
                returnVector = std::make_shared<AngularVelocityVector>(upperFrame, lowerFrame);
                returnVector->matrix = relativeAngularVelocity;
                returnVector->writtenFrame = writtenFrame;

                return returnVector;
            } else {
                std::shared_ptr<Frame> desUpperFrame = commonVecs.at(i)->upperFrame.lock();

                visitedVectors.push_back(commonVecs.at(i));

                returnVector = findRelativeAngularVelocity(upperFrame, lowerFrame, desUpperFrame, writtenFrame,
                                                           relativeAngularVelocity, visitedVectors);
            }
        }

        if (returnVector->upperFrame.lock() == upperFrame && returnVector->lowerFrame.lock() == lowerFrame) {
            return returnVector;
        }
    }
    return returnVector;
}

std::shared_ptr<InertiaTensor> KinematicsEngine::parallelAxisTheorem(std::shared_ptr<Part> part, std::shared_ptr<Point> point, std::shared_ptr<Frame> writtenFrame) {
    Eigen::Vector3d inputVec = {0.0, 0.0, 0.0};
    std::vector<std::shared_ptr<PositionVector>> emptyVector;

    std::shared_ptr<PositionVector> relativePosition = findRelativePosition(part->IPntSc_S->point, point,
                                                                            part->IPntSc_S->point, writtenFrame,
                                                                            inputVec, emptyVector);

    Eigen::MRPd relativeAttitude;
    relativeAttitude = findRelativeAttitude(writtenFrame, part->IPntSc_S->writtenFrame);

    Eigen::Matrix3d dcm = relativeAttitude.toRotationMatrix().transpose();

    Eigen::Matrix3d returnInertiaMatrix = dcm * part->IPntSc_S->matrix * dcm.transpose() +
                                          (part->mass * eigenTilde(relativePosition->matrix) * eigenTilde(relativePosition->matrix).transpose());

    auto returnInertia = std::make_shared<InertiaTensor>(point);
    returnInertia->matrix = returnInertiaMatrix;
    returnInertia->writtenFrame = writtenFrame;

    return returnInertia;
}

double KinematicsEngine::getAssemblyMass(std::shared_ptr<Assembly> assembly) {
    double assemblyMass = 0.0;

    for (int i = 0; i < assembly->partList.size(); i++) {
        assemblyMass = assemblyMass + assembly->partList.at(i)->mass;
    }

    return assemblyMass;
}

std::shared_ptr<PositionVector> KinematicsEngine::getAssemblyCOM(std::shared_ptr<Assembly> assembly, std::shared_ptr<Point> tailPoint, std::shared_ptr<Frame> writtenFrame) {
    Eigen::Vector3d assemblyCOMVector = {0.0, 0.0, 0.0};
    double assemblyMass = getAssemblyMass(assembly);

    for (int i = 0; i<assembly->partList.size(); i++) {
        Eigen::Vector3d inputVec = {0.0, 0.0, 0.0};
        std::vector<std::shared_ptr<PositionVector>> emptyVector;
        auto addedPositionVector = findRelativePosition(assembly->partList.at(i)->r_ScS->headPoint, tailPoint, assembly->partList.at(i)->r_ScS->headPoint, writtenFrame, inputVec, emptyVector);

        Eigen::Vector3d addVector = {(assembly->partList.at(i)->mass / assemblyMass) * addedPositionVector->matrix(0),
                                     (assembly->partList.at(i)->mass / assemblyMass) * addedPositionVector->matrix(1),
                                     (assembly->partList.at(i)->mass / assemblyMass) * addedPositionVector->matrix(2)};
        assemblyCOMVector = assemblyCOMVector + addVector;
    }

    auto assemblyCOMPositionVector = std::make_shared<PositionVector>();
    assemblyCOMPositionVector->matrix = assemblyCOMVector;
    assemblyCOMPositionVector->writtenFrame = writtenFrame;

    return assemblyCOMPositionVector;
}

std::shared_ptr<InertiaTensor> KinematicsEngine::getAssemblyInertia(std::shared_ptr<Assembly> assembly, std::shared_ptr<Point> point, std::shared_ptr<Frame> writtenFrame) {
    auto assemblyInertia = std::make_shared<InertiaTensor>();
    Eigen::Matrix3d intermediateInertia;
    intermediateInertia.setZero();

    for (int i = 0; i<assembly->partList.size(); i++) {
        auto partInertia = parallelAxisTheorem(assembly->partList.at(i), point, writtenFrame);

        intermediateInertia = intermediateInertia + partInertia->matrix;
    }

    assemblyInertia->matrix = intermediateInertia;
    assemblyInertia->point = point;
    assemblyInertia->writtenFrame = writtenFrame;

    return assemblyInertia;
}

