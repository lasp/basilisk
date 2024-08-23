/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics,
 University of Colorado at Boulder

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

#include "linearODeKF.h"

/*! Reset the flyby OD filter to an initial state and initializes the internal estimation matrices.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void LinearODeKF::customReset() {
    /*! - Check if the required message has not been connected */
    assert(this->opNavHeadingMsg.isLinked());
    if (this->constantVelocityInitial) {
        this->constantVelocity = this->constantVelocityInitial.value()*this->unitConversion;
    }
    /*! - Set the dynamics matrix calculator*/
    std::function<Eigen::MatrixXd(double, const StateVector)> dynamicsMatrixCalculator= [this]
            (double time, const StateVector &state){
        Eigen::VectorXd position = state.getPositionStates();
        Eigen::MatrixXd dynamicsMatrix = Eigen::MatrixXd::Zero(state.size(), state.size());
        if (!this->constantVelocity) {
            dynamicsMatrix.block(0, position.size(), position.size(), position.size()) =
                    Eigen::MatrixXd::Identity(position.size(), position.size());
        }
        return dynamicsMatrix;
    };

    this->dynamics.setDynamicsMatrix(dynamicsMatrixCalculator);

    /*! - Set the filter dynamics (linear) */
    std::function<StateVector(double, const StateVector)> twoBodyDynamics = [this](double t, const StateVector &state){
        StateVector XDot;
        PositionState stateDerivative;

        if (this->constantVelocity) {
            stateDerivative.setValues(this->constantVelocity.value());
        }
        else {
            stateDerivative.setValues(state.getVelocityStates());
            VelocityState flybyVelocity;
            flybyVelocity.setValues(Eigen::Vector3d::Zero());
            XDot.setVelocity(flybyVelocity);
        }

        XDot.setPosition(stateDerivative);
        Eigen::MatrixXd stm = state.detachStm();

        Eigen::MatrixXd dynMatrix = this->dynamics.computeDynamicsMatrix(t, state);
        XDot.attachStm(dynMatrix*stm);

        return XDot;
    };
    this->dynamics.setDynamics(twoBodyDynamics);
}

/*! Write the output data to appropriate messages given the state components
 @return void
 @param uint64_t CurrentSimNanos
 */
void LinearODeKF::writeOutputMessages(uint64_t CurrentSimNanos) {
    NavTransMsgPayload navTransOutMsgBuffer = this->navTransOutMsg.zeroMsgPayload;
    FilterMsgPayload opNavFilterMsgBuffer = this->opNavFilterMsg.zeroMsgPayload;
    FilterResidualsMsgPayload residualsBuffer = this->opNavResidualMsg.zeroMsgPayload;

    /*! - Write the flyby OD estimate into the copy of the navigation message structure*/
    eigenMatrixXd2CArray(this->stateLogged.scale(1/this->unitConversion).getPositionStates(), navTransOutMsgBuffer.r_BN_N);
    if (this->constantVelocity){
        eigenMatrixXd2CArray(1/this->unitConversion*this->constantVelocity.value(), navTransOutMsgBuffer.v_BN_N);
    }
    else{
        eigenMatrixXd2CArray(this->stateLogged.scale(1/this->unitConversion).getVelocityStates(), navTransOutMsgBuffer.v_BN_N);
    }

    /*! - Populate the filter states output buffer and write the output message*/
    opNavFilterMsgBuffer.timeTag = this->previousFilterTimeTag;
    eigenMatrixXd2CArray(this->stateLogged.scale(1/this->unitConversion).returnValues(), opNavFilterMsgBuffer.state);
    eigenMatrixXd2CArray(1/this->unitConversion*this->stateError, opNavFilterMsgBuffer.stateError);
    eigenMatrixXd2CArray(1/this->unitConversion/this->unitConversion*this->covar, opNavFilterMsgBuffer.covar);
    opNavFilterMsgBuffer.numberOfStates = this->state.size();

    auto optionalMeasurement = this->measurements[0];
    if (optionalMeasurement.has_value()) {
        auto measurement = MeasurementModel();
        measurement = optionalMeasurement.value();
        residualsBuffer.valid = true;
        residualsBuffer.numberOfObservations = 1;
        residualsBuffer.sizeOfObservations = measurement.size();
        eigenMatrixXd2CArray(measurement.getObservation(), &residualsBuffer.observation[0]);
        eigenMatrixXd2CArray(measurement.getPostFitResiduals(), &residualsBuffer.postFits[0]);
        eigenMatrixXd2CArray(measurement.getPreFitResiduals(), &residualsBuffer.preFits[0]);
        this->measurements[0].reset();
    }
    this->opNavResidualMsg.write(&residualsBuffer, this->moduleID, CurrentSimNanos);
    this->navTransOutMsg.write(&navTransOutMsgBuffer, this->moduleID, CurrentSimNanos);
    this->opNavFilterMsg.write(&opNavFilterMsgBuffer, this->moduleID, CurrentSimNanos);
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void LinearODeKF::readFilterMeasurements() {
    this->opNavHeadingBuffer = this->opNavHeadingMsg();
    auto headingMeasurement = MeasurementModel();

    headingMeasurement.setTimeTag(this->opNavHeadingBuffer.timeTag);
    headingMeasurement.setValidity(this->opNavHeadingBuffer.valid);

    if (headingMeasurement.getValidity() && headingMeasurement.getTimeTag() >= this->previousFilterTimeTag){
        /*! - Read measurement and cholesky decomposition its noise*/
        headingMeasurement.setObservation(cArray2EigenVector3d(this->opNavHeadingBuffer.rhat_BN_N).normalized());
        headingMeasurement.setMeasurementNoise(
                this->measNoiseScaling * cArray2EigenMatrixXd(this->opNavHeadingBuffer.covar_N,
                                                                   (int) headingMeasurement.size(),
                                                                   (int) headingMeasurement.size()));
        headingMeasurement.setMeasurementModel(MeasurementModel::normalizedPositionStates);
        headingMeasurement.setMeasurementMatrix(LinearODeKF::measurementMatrix);
        this->measurements[0] = headingMeasurement;
    }
}

/*! Compute the measurement matrix to linearize the measurement model
    @param StateVector state
    @return Eigen::MatrixXd
*/
Eigen::MatrixXd LinearODeKF::measurementMatrix(const StateVector &state){

    Eigen::Vector3d position = state.getPositionStates();
    Eigen::MatrixXd measurementMatrix = Eigen::MatrixXd::Zero(position.size(), state.size());
    measurementMatrix.block(0, 0, position.size(), position.size()) =
            1/position.norm()*(Eigen::MatrixXd::Identity(position.size(), position.size())
            - 1/pow(position.norm(), 2)*position*position.transpose());

    return measurementMatrix;
}

/*! Set a constant velocity vector that will not be estimated. Assert that velocity is not part of the state
    @param Eigen::Vector3d velocity
    */
void LinearODeKF::setConstantVelocity(const Eigen::Vector3d &velocity){
    assert(this->state.hasVelocity() == false);
    this->constantVelocityInitial = velocity;
}

/*! Get the constant velocity vector
    @return std::optional<Eigen::Vector3d> velocity
    */
std::optional<Eigen::Vector3d> LinearODeKF::getConstantVelocity() const {
    return this->constantVelocityInitial;
}
