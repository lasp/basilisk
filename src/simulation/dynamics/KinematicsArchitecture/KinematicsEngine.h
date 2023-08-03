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

#include "simulation/dynamics/KinematicsArchitecture/Frame.h"
#include "simulation/dynamics/KinematicsArchitecture/Part.h"
#include "simulation/dynamics/KinematicsArchitecture/Point.h"
#include "simulation/dynamics/KinematicsArchitecture/Tensor.h"
#include "simulation/dynamics/KinematicsArchitecture/Vector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"
#include <iostream>

class KinematicsEngine {
public:
    KinematicsEngine();
    ~KinematicsEngine();

    std::shared_ptr<Frame> rootFrame;
    std::vector<std::shared_ptr<Frame>> frameList;
    std::vector<std::shared_ptr<Part>> partList;
    std::vector<std::shared_ptr<Point>> pointList;
    std::vector<std::shared_ptr<Translation>> translationList;
    std::vector<std::shared_ptr<Rotation>> rotationList;

    std::shared_ptr<Point> createPoint();
    std::shared_ptr<Frame> createFrame();
    std::shared_ptr<Frame> createFrame(const std::shared_ptr<Frame>& parentFrame);
    std::shared_ptr<Part> createPart();
    std::shared_ptr<Part> createPart(const std::shared_ptr<Frame>& parentFrame);
    std::shared_ptr<Translation> createTranslationProperties(const std::shared_ptr<Point>& headPoint, const std::shared_ptr<Point>& tailPoint);
    std::shared_ptr<Rotation> createRotationProperties(const std::shared_ptr<Frame>& upperFrame, const std::shared_ptr<Frame>& lowerFrame);
    InertiaTensor createInertiaTensor(const std::shared_ptr<Point>& point);

    static std::vector<std::shared_ptr<Frame>> findAbsolutePath(const std::shared_ptr<Frame>& frame);
    static std::pair<std::vector<const std::shared_ptr<Frame>>, std::vector<const std::shared_ptr<Frame>>> findPath2LCA(const std::shared_ptr<Frame>& upperFrame, const std::shared_ptr<Frame>& lowerFrame);
    static Eigen::MRPd findRelativeAttitude(const std::shared_ptr<Frame>& upperFrame, const std::shared_ptr<Frame>& lowerFrame);
    static Vector findRelativeAngularVelocity(const std::shared_ptr<Frame>& upperFrame, const std::shared_ptr<Frame>& lowerFrame);
    static Vector findRelativeAngularAcceleration(const std::shared_ptr<Frame>& upperFrame, const std::shared_ptr<Frame>& lowerFrame, const std::shared_ptr<Frame>& derivFrame);
    Vector findRelativePosition(std::shared_ptr<Point> headPoint, const std::shared_ptr<Point>& tailPoint);
    Vector findRelativeVelocity(std::shared_ptr<Point> headPoint, const std::shared_ptr<Point>& tailPoint, const std::shared_ptr<Frame> derivFrame);
    InertiaTensor parallelAxisTheorem(const std::shared_ptr<Part>& part, const std::shared_ptr<Point>& point);

private:
    static Eigen::MRPd findIntermediateAttitude(const std::vector<const std::shared_ptr<Frame>>& path);
    static Vector findIntermediateAngularVelocity(const std::vector<const std::shared_ptr<Frame>>& path);
    static Vector findIntermediateAngularAcceleration(const std::vector<const std::shared_ptr<Frame>>& path, const std::shared_ptr<Frame>& derivFrame);
    std::pair<std::vector<const std::shared_ptr<const Translation>>, bool> searchPointPath(std::shared_ptr<const Point> headPoint,
                                                                                           const std::shared_ptr<const Point>& tailPoint,
                                                                                           std::vector<const std::shared_ptr<const Translation>> translationPath,
                                                                                           std::vector<const std::shared_ptr<const Translation>> visitedContainers, bool leafReached);
};

#endif
