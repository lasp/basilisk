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
#include "Vector.h"
#include "Frame.h"
#include <utility>


/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the module */
Vector::Vector(){
}

Vector::Vector(Eigen::Vector3d zerothMatrix, Frame* zerothWrittenFrame){
    this->matrix = std::move(zerothMatrix);
    this->writtenFrame = zerothWrittenFrame;
}

/*! Module Destructor. */
Vector::~Vector(){
}



/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the module */
PositionVector::PositionVector(Eigen::Vector3d zerothMatrix,
                               Frame* zerothWrittenFrame):Vector(zerothMatrix, zerothWrittenFrame){
}

PositionVector::PositionVector(Eigen::Vector3d zerothMatrix, Frame* zerothWrittenFrame,
               Eigen::Vector3d firstMatrix, Frame* firstWrittenFrame, Frame* firstDerivFrame,
               Eigen::Vector3d secondMatrix, Frame* secondWrittenFrame,
               Frame* secondDerivFrame):Vector(zerothMatrix, zerothWrittenFrame){
this->firstOrder.matrix = std::move(firstMatrix);
this->firstOrder.writtenFrame = firstWrittenFrame;
this->firstOrder.derivFrame = firstDerivFrame;
this->secondOrder.matrix = std::move(secondMatrix);
this->secondOrder.writtenFrame = secondWrittenFrame;
this->secondOrder.derivFrame = secondDerivFrame;
}



/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the module */
AngularVelocityVector::AngularVelocityVector(Eigen::Vector3d zerothMatrix,
                               Frame* zerothWrittenFrame):Vector(zerothMatrix, zerothWrittenFrame){
}

AngularVelocityVector::AngularVelocityVector(Eigen::Vector3d zerothMatrix, Frame* zerothWrittenFrame,
               Eigen::Vector3d firstMatrix, Frame* firstWrittenFrame,
               Frame* firstDerivFrame):Vector(zerothMatrix, zerothWrittenFrame){
this->firstOrder.matrix = std::move(firstMatrix);
this->firstOrder.writtenFrame = firstWrittenFrame;
this->firstOrder.derivFrame = firstDerivFrame;
}



/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the module */
UnitVector::UnitVector(Eigen::Vector3d zerothMatrix,
                       Frame* zerothWrittenFrame):Vector(zerothMatrix.normalized(), zerothWrittenFrame){
}

/*! Module Destructor. */
UnitVector::~UnitVector(){
}