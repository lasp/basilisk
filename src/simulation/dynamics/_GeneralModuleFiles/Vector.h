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

#ifndef VECTOR_H
#define VECTOR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include <Eigen/Core>
#include <utility>

class Frame;
class Point;

/*! @brief basic Basilisk C++ module class */
class Vector{
public:
    Vector() = default;
    Vector(Eigen::Vector3d zerothMatrix, Frame* zerothWrittenFrame);
    ~Vector() = default;

    BSKLogger bskLogger;              //!< -- BSK Logging

    Eigen::Vector3d matrix = Eigen::Vector3d::Zero();
    Frame* writtenFrame = nullptr;
};


/*! @brief Vector derivative properties data structure */
struct VectorDerivativeProperties{
    Eigen::Vector3d matrix = Eigen::Vector3d::Zero();
    Frame* writtenFrame = nullptr;
    Frame* derivFrame = nullptr;
};


/*! @brief basic Basilisk C++ module class */
class PositionVector : public Vector{
public:
    PositionVector() = default;
    PositionVector(Eigen::Vector3d zerothMatrix, Frame* zerothWrittenFrame);
    PositionVector(Eigen::Vector3d zerothMatrix, Frame* zerothWrittenFrame,
                   Eigen::Vector3d firstMatrix, Frame* firstWrittenFrame,
                   Frame* firstDerivFrame, Eigen::Vector3d secondMatrix,
                   Frame* secondWrittenFrame, Frame* secondDerivFrame);
    ~PositionVector() = default;

    Point* tailPoint = nullptr;
    Point* headPoint = nullptr;

    VectorDerivativeProperties firstOrder;
    VectorDerivativeProperties secondOrder;
};


/*! @brief basic Basilisk C++ module class */
class ForceVector : public Vector{
public:
    ForceVector() = default;
    ~ForceVector() = default;

    Point* applicationPoint = nullptr;
};


/*! @brief basic Basilisk C++ module class */
class AngularVelocityVector : public Vector{
public:
    AngularVelocityVector() = default;
    AngularVelocityVector(Eigen::Vector3d zerothMatrix, Frame* zerothWrittenFrame);
    AngularVelocityVector(Eigen::Vector3d zerothMatrix, Frame* zerothWrittenFrame,
                          Eigen::Vector3d firstMatrix, Frame* firstWrittenFrame, Frame* firstDerivFrame);
    ~AngularVelocityVector() = default;

    VectorDerivativeProperties firstOrder;
    Frame* upperFrame = nullptr;
    Frame* lowerFrame = nullptr;
};


/*! @brief basic Basilisk C++ module class */
class UnitVector : public Vector{
public:
    UnitVector(Eigen::Vector3d zerothMatrix, Frame* zerothWrittenFrame);
    ~UnitVector() = default;
};


#endif
