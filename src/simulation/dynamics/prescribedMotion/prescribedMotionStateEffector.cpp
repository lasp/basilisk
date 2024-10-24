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

#include "prescribedMotionStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include <string>

/*! The constructor sets the module variables to default values. */
PrescribedMotionStateEffector::PrescribedMotionStateEffector()
{
    // Zero the effector's mass properties and mass property rate contributions
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);

    // Initialize the effector's hub-relative states
    this->r_FM_M.setZero();
    this->rPrime_FM_M.setZero();
    this->rPrimePrime_FM_M.setZero();
    this->omega_FM_F.setZero();
    this->omegaPrime_FM_F.setZero();
    this->sigma_FM.setIdentity();

    // Initialize the other module variables
    this->currentSimTimeSec = 0.0;
    this->mass = 0.0;
    this->IPntFc_F.setIdentity();
    this->r_FcF_F.setZero();
    this->r_MB_B.setZero();
    this->omega_MB_M.setZero();
    this->omegaPrime_MB_B.setZero();
    this->sigma_MB.setIdentity();

    // Initialize prescribed states at epoch
    this->rEpoch_FM_M.setZero();
    this->rPrimeEpoch_FM_M.setZero();
    this->omegaEpoch_FM_F.setZero();

    // Set the effector's hub-relative sigma_FM MRP attitude state name
    this->nameOfsigma_FMState = "prescribedMotionsigma_FM" + std::to_string(this->effectorID);

    PrescribedMotionStateEffector::effectorID++;
}

uint64_t PrescribedMotionStateEffector::effectorID = 1;

/*! This is the destructor. */
PrescribedMotionStateEffector::~PrescribedMotionStateEffector()
{
    PrescribedMotionStateEffector::effectorID = 1;
}

/*! This method is used to reset the module.
 @return void
 @param callTime [ns] Time the method is called
*/
void PrescribedMotionStateEffector::Reset(uint64_t callTime)
{
}

/*! This method writes the module output messages to the messaging system.
 @return void
 @param callTime [ns] Time the method is called
*/
void PrescribedMotionStateEffector::writeOutputStateMessages(uint64_t callTime)
{
    // Write the prescribed translational motion output message if it is linked
    if (this->prescribedTranslationOutMsg.isLinked()) {
        PrescribedTranslationMsgPayload prescribedTranslationBuffer = this->prescribedTranslationOutMsg.zeroMsgPayload;
        eigenVector3d2CArray(this->r_FM_M, prescribedTranslationBuffer.r_FM_M);
        eigenVector3d2CArray(this->rPrime_FM_M, prescribedTranslationBuffer.rPrime_FM_M);
        eigenVector3d2CArray(this->rPrimePrime_FM_M, prescribedTranslationBuffer.rPrimePrime_FM_M);
        this->prescribedTranslationOutMsg.write(&prescribedTranslationBuffer, this->moduleID, callTime);
    }

    // Write the prescribed rotational motion output message if it is linked
    if (this->prescribedRotationOutMsg.isLinked()) {
        PrescribedRotationMsgPayload prescribedRotationBuffer = this->prescribedRotationOutMsg.zeroMsgPayload;
        eigenVector3d2CArray(this->omega_FM_F, prescribedRotationBuffer.omega_FM_F);
        eigenVector3d2CArray(this->omegaPrime_FM_F, prescribedRotationBuffer.omegaPrime_FM_F);
        Eigen::Vector3d sigma_FM_loc = eigenMRPd2Vector3d(this->sigma_FM);
        eigenVector3d2CArray(sigma_FM_loc, prescribedRotationBuffer.sigma_FM);
        this->prescribedRotationOutMsg.write(&prescribedRotationBuffer, this->moduleID, callTime);
    }

    // Write the effector config log message if it is linked
    if (this->prescribedMotionConfigLogOutMsg.isLinked()) {
        SCStatesMsgPayload configLogMsg = this->prescribedMotionConfigLogOutMsg.zeroMsgPayload;

        // Note that the configLogMsg B frame represents the prescribed motion effector body frame (F frame)
        eigenVector3d2CArray(this->r_FcN_N, configLogMsg.r_BN_N);
        eigenVector3d2CArray(this->v_FcN_N, configLogMsg.v_BN_N);
        eigenVector3d2CArray(this->sigma_FN, configLogMsg.sigma_BN);
        eigenVector3d2CArray(this->omega_FN_F, configLogMsg.omega_BN_B);
        this->prescribedMotionConfigLogOutMsg.write(&configLogMsg, this->moduleID, callTime);
    }
}

/*! This method gives the prescribed motion effector access the hub inertial states.
 @return void
 @param statesIn Pointer to give the state effector access the hub states
*/
void PrescribedMotionStateEffector::linkInStates(DynParamManager& statesIn)
{
    this->hubOmega = statesIn.getStateObject("hubOmega");
    this->hubSigma = statesIn.getStateObject("hubSigma");
    this->inertialPositionProperty = statesIn.getPropertyReference("r_BN_N");
    this->inertialVelocityProperty = statesIn.getPropertyReference("v_BN_N");
}

/*! This method registers the prescribed motion effector hub-relative attitude with the dynamic parameter manager.
 @return void
 @param states Pointer to give the state effector access the hub states
*/
void PrescribedMotionStateEffector::registerStates(DynParamManager& states)
{
    this->sigma_FMState = states.registerState(3, 1, this->nameOfsigma_FMState);
    Eigen::Vector3d sigma_FM_loc = eigenMRPd2Vector3d(this->sigma_FM);
    Eigen::Vector3d sigma_FMInitMatrix;
    sigma_FMInitMatrix(0) = sigma_FM_loc[0];
    sigma_FMInitMatrix(1) = sigma_FM_loc[1];
    sigma_FMInitMatrix(2) = sigma_FM_loc[2];
    this->sigma_FMState->setState(sigma_FMInitMatrix);
}

/*! This method provides the effector contributions to the mass props and mass prop rates of
 the spacecraft.
 @return void
 @param callTime [s] Time the method is called
*/
void PrescribedMotionStateEffector::updateEffectorMassProps(double callTime)
{
    // Update the prescribed states
    double dt = callTime - this->currentSimTimeSec;
    this->r_FM_M = this->rEpoch_FM_M + (this->rPrimeEpoch_FM_M * dt) + (0.5 * this->rPrimePrime_FM_M * dt * dt);
    this->rPrime_FM_M = this->rPrimeEpoch_FM_M + (this->rPrimePrime_FM_M * dt);
    this->omega_FM_F = this->omegaEpoch_FM_F + (this->omegaPrime_FM_F * dt);
    this->sigma_FM = (Eigen::Vector3d)this->sigma_FMState->getState();

    // Give the mass of the prescribed body to the effProps mass
    this->effProps.mEff = this->mass;

    // Compute dcm_BM
    Eigen::Matrix3d dcm_BM = this->sigma_MB.toRotationMatrix();

    // Compute dcm_FM
    Eigen::Matrix3d dcm_FM = (this->sigma_FM.toRotationMatrix()).transpose();

    // Compute dcm_BF
    this->dcm_BF = dcm_BM * dcm_FM.transpose();

    // Compute omega_FB_B
    Eigen::Vector3d omega_FM_B = this->dcm_BF * this->omega_FM_F;
    Eigen::Vector3d omega_MB_B = dcm_BM * this->omega_MB_M;
    this->omega_FB_B = omega_FM_B + omega_MB_B;

    // Compute omegaPrime_FB_B
    this->omegaTilde_FB_B = eigenTilde(this->omega_FB_B);
    Eigen::Vector3d omegaPrime_FM_B = this->dcm_BF * this->omegaPrime_FM_F;
    this->omegaPrime_FB_B = omegaPrime_FM_B + this->omegaTilde_FB_B * omega_FM_B;

    // Convert the prescribed translational states to the B frame
    Eigen::Vector3d r_FM_B = dcm_BM * this->r_FM_M;
    Eigen::Vector3d rPrime_FM_B = dcm_BM * this->rPrime_FM_M;
    this->rPrimePrime_FM_B = dcm_BM * this->rPrimePrime_FM_M;

    // Compute the effector's center of mass with respect to point B
    Eigen::Vector3d r_FB_B = r_FM_B + this->r_MB_B;
    this->r_FcF_B = this->dcm_BF * this->r_FcF_F;
    this->r_FcB_B = this->r_FcF_B + r_FB_B;
    this->effProps.rEff_CB_B = this->r_FcB_B;

    // Find the effector's inertia about point B
    this->rTilde_FcB_B = eigenTilde(this->r_FcB_B);
    this->IPntFc_B = this->dcm_BF * this->IPntFc_F * this->dcm_BF.transpose();
    this->effProps.IEffPntB_B = this->IPntFc_B - this->mass * this->rTilde_FcB_B * this->rTilde_FcB_B;

    // Find the B frame time derivative of r_FcB_B
    this->omegaTilde_FB_B = eigenTilde(this->omega_FB_B);
    this->rPrime_FcB_B = this->omegaTilde_FB_B * this->r_FcF_B + rPrime_FM_B;
    this->effProps.rEffPrime_CB_B = this->rPrime_FcB_B;

    // Find the B frame time derivative of IPntFc_B
    Eigen::Matrix3d rPrimeTilde_FcB_B = eigenTilde(this->rPrime_FcB_B);
    this->effProps.IEffPrimePntB_B = this->omegaTilde_FB_B * this->IPntFc_B
                                     - this->IPntFc_B * this->omegaTilde_FB_B
                                     + this->mass * (rPrimeTilde_FcB_B * this->rTilde_FcB_B.transpose()
                                     + this->rTilde_FcB_B * rPrimeTilde_FcB_B.transpose());
}

/*! This method provides the effector's backsubstitution contributions.
 method
 @return void
 @param callTime [s] Time the method is called
 @param backSubContr State effector contribution matrices for backsubstitution
 @param sigma_BN Current B frame attitude with respect to the inertial frame
 @param omega_BN_B [rad/s] Angular velocity of the B frame with respect to the inertial frame, expressed in B frame
 components
 @param g_N [m/s^2] Gravitational acceleration in N frame components
*/
void PrescribedMotionStateEffector::updateContributions(double callTime,
                                                        BackSubMatrices & backSubContr,
                                                        Eigen::Vector3d sigma_BN,
                                                        Eigen::Vector3d omega_BN_B,
                                                        Eigen::Vector3d g_N)
{
    // Compute dcm_BN
    this->sigma_BN = sigma_BN;
    this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();

    // Define omega_BN_B
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);

    // Define omegaPrimeTilde_FB_B
    Eigen::Matrix3d omegaPrimeTilde_FB_B = eigenTilde(this->omegaPrime_FB_B);

    // Compute rPrimePrime_FcB_B
    Eigen::Vector3d rPrimePrime_FcB_B = (omegaPrimeTilde_FB_B + this->omegaTilde_FB_B * this->omegaTilde_FB_B) * this->r_FcF_B
                              + this->rPrimePrime_FM_B;

    // Backsubstitution RHS translational EOM contribution
    backSubContr.vecTrans = -this->mass * rPrimePrime_FcB_B;

    // Backsubstitution RHS rotational EOM contribution
    Eigen::Matrix3d IPrimePntFc_B = this->omegaTilde_FB_B * this->IPntFc_B - this->IPntFc_B * this->omegaTilde_FB_B;
    backSubContr.vecRot = -(this->mass * this->rTilde_FcB_B * rPrimePrime_FcB_B)
                          - (IPrimePntFc_B + this->omegaTilde_BN_B * this->IPntFc_B) * this->omega_FB_B
                          - this->IPntFc_B * this->omegaPrime_FB_B
                          - this->mass * this->omegaTilde_BN_B * rTilde_FcB_B * this->rPrime_FcB_B;
}

/*! This method defines the state effector's MRP attitude state derivative
 @return void
 @param callTime [s] Time the method is called
 @param rDDot_BN_N [m/s^2] Acceleration of the vector pointing from the inertial frame origin to the B frame origin,
 expressed in inertial frame components
 @param omegaDot_BN_B [rad/s^2] Inertial time derivative of the angular velocity of the B frame with respect to the
 inertial frame, expressed in B frame components
 @param sigma_BN Current B frame attitude with respect to the inertial frame
*/
void PrescribedMotionStateEffector::computeDerivatives(double callTime,
                                                       Eigen::Vector3d rDDot_BN_N,
                                                       Eigen::Vector3d omegaDot_BN_B,
                                                       Eigen::Vector3d sigma_BN)
{
    Eigen::MRPd sigma_FM_loc;
    sigma_FM_loc = (Eigen::Vector3d)this->sigma_FMState->getState();
    this->sigma_FMState->setDerivative(0.25 * sigma_FM_loc.Bmat() * this->omega_FM_F);
}

/*! This method calculates the effector's contributions to the energy and momentum of the
 spacecraft.
 @return void
 @param callTime [s] Time the method is called
 @param rotAngMomPntCContr_B [kg m^2/s] Contribution of stateEffector to total rotational angular mom
 @param rotEnergyContr [J] Contribution of stateEffector to total rotational energy
 @param omega_BN_B [rad/s] Angular velocity of the B frame with respect to the inertial frame, expressed in B frame
 components
*/
void PrescribedMotionStateEffector::updateEnergyMomContributions(double callTime,
                                                                 Eigen::Vector3d & rotAngMomPntCContr_B,
                                                                 double & rotEnergyContr,
                                                                 Eigen::Vector3d omega_BN_B)
{
    // Update omega_BN_B and omega_FN_B
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    this->omega_FN_B = this->omega_FB_B + this->omega_BN_B;

    // Compute rDot_FcB_B
    this->rDot_FcB_B = this->rPrime_FcB_B + this->omegaTilde_BN_B * this->r_FcB_B;

    // Find the rotational angular momentum contribution
    rotAngMomPntCContr_B = this->IPntFc_B * this->omega_FN_B + this->mass * this->rTilde_FcB_B * this->rDot_FcB_B;

    // Find the rotational energy contribution
    rotEnergyContr = 0.5 * this->omega_FN_B.dot(this->IPntFc_B * this->omega_FN_B)
                     + 0.5 * this->mass * this->rDot_FcB_B.dot(this->rDot_FcB_B);
}

/*! This method computes the prescribed motion state effector states relative to the inertial frame.
 @return void
*/
void PrescribedMotionStateEffector::computePrescribedMotionInertialStates()
{
    // Compute the effector's attitude with respect to the inertial frame
    Eigen::Matrix3d dcm_FN = (this->dcm_BF).transpose() * this->dcm_BN;
    this->sigma_FN = eigenMRPd2Vector3d(eigenC2MRP(dcm_FN));

    // Compute the effector's inertial angular velocity
    this->omega_FN_F = (this->dcm_BF).transpose() * this->omega_FN_B;

    // Compute the effector's inertial position vector
    this->r_FcN_N = (Eigen::Vector3d)*this->inertialPositionProperty + this->dcm_BN.transpose() * this->r_FcB_B;

    // Compute the effector's inertial velocity vector
    this->v_FcN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * this->rDot_FcB_B;
}

/*! This method updates the effector's states at the dynamics frequency.
 @return void
 @param callTime [ns] Time the method is called
*/
void PrescribedMotionStateEffector::UpdateState(uint64_t callTime)
{
    // Store the current simulation time
    this->currentSimTimeSec = callTime * NANO2SEC;

    // Read the translational input message if it is linked and written
    if (this->prescribedTranslationInMsg.isLinked() && this->prescribedTranslationInMsg.isWritten()) {
        PrescribedTranslationMsgPayload incomingPrescribedTransStates = this->prescribedTranslationInMsg();
        this->r_FM_M = cArray2EigenVector3d(incomingPrescribedTransStates.r_FM_M);
        this->rPrime_FM_M = cArray2EigenVector3d(incomingPrescribedTransStates.rPrime_FM_M);
        this->rPrimePrime_FM_M = cArray2EigenVector3d(incomingPrescribedTransStates.rPrimePrime_FM_M);

        // Save off the prescribed translational states at each dynamics time step
        this->rEpoch_FM_M = cArray2EigenVector3d(incomingPrescribedTransStates.r_FM_M);
        this->rPrimeEpoch_FM_M = cArray2EigenVector3d(incomingPrescribedTransStates.rPrime_FM_M);
    }

    // Read the rotational input message if it is linked and written
    if (this->prescribedRotationInMsg.isLinked() && this->prescribedRotationInMsg.isWritten()) {
        PrescribedRotationMsgPayload incomingPrescribedRotStates = this->prescribedRotationInMsg();
        this->omega_FM_F = cArray2EigenVector3d(incomingPrescribedRotStates.omega_FM_F);
        this->omegaPrime_FM_F = cArray2EigenVector3d(incomingPrescribedRotStates.omegaPrime_FM_F);
        this->sigma_FM = cArray2EigenVector3d(incomingPrescribedRotStates.sigma_FM);

        // Save off the prescribed rotational states at each dynamics time step
        this->omegaEpoch_FM_F = cArray2EigenVector3d(incomingPrescribedRotStates.omega_FM_F);
        Eigen::Vector3d sigma_FM_loc = cArray2EigenVector3d(incomingPrescribedRotStates.sigma_FM);
        this->sigma_FMState->setState(sigma_FM_loc);
    }

    // Call the method to compute the effector's inertial states
    this->computePrescribedMotionInertialStates();

    // Call the method to write the output messages
    this->writeOutputStateMessages(callTime);
}

/*! Setter method for the effector mass.
 @return void
 @param mass [kg] Effector mass
*/
void PrescribedMotionStateEffector::setMass(const double mass) {
    this->mass = mass;
}

/*! Setter method for IPntFc_F.
 @return void
 @param IPntFc_F [kg-m^2] Effector's inertia matrix about its center of mass point Fc expressed in F frame components
*/
void PrescribedMotionStateEffector::setIPntFc_F(const Eigen::Matrix3d IPntFc_F) {
    this->IPntFc_F = IPntFc_F;
}

/*! Setter method for r_FcF_F.
 @return void
 @param r_FcF_F [m] Position vector of the effector's center of mass point Fc relative to the effector's body frame origin point F expressed in F frame components
*/
void PrescribedMotionStateEffector::setR_FcF_F(const Eigen::Vector3d r_FcF_F) {
    this->r_FcF_F = r_FcF_F;
}

/*! Setter method for r_FM_M.
 @return void
 @param r_FM_M [m] Position vector of the effector's body frame origin point F relative to the hub-fixed mount frame origin point M expressed in M frame components
*/
void PrescribedMotionStateEffector::setR_FM_M(const Eigen::Vector3d r_FM_M) {
    this->r_FM_M = r_FM_M;
}

/*! Setter method for rPrime_FM_M.
 @return void
 @param rPrime_FM_M [m/s] B frame time derivative of r_FM_M expressed in M frame components
*/
void PrescribedMotionStateEffector::setRPrime_FM_M(const Eigen::Vector3d rPrime_FM_M) {
    this->rPrime_FM_M = rPrime_FM_M;
}

/*! Setter method for rPrimePrime_FM_M.
 @return void
 @param rPrimePrime_FM_M [m/s^2] B frame time derivative of rPrime_FM_M expressed in M frame components
*/
void PrescribedMotionStateEffector::setRPrimePrime_FM_M(const Eigen::Vector3d rPrimePrime_FM_M) {
    this->rPrimePrime_FM_M = rPrimePrime_FM_M;
}

/*! Setter method for omega_FM_F.
 @return void
 @param omega_FM_F [rad/s] Angular velocity of the effector body frame F relative to the hub-fixed mount frame M expressed in F frame components
*/
void PrescribedMotionStateEffector::setOmega_FM_F(const Eigen::Vector3d omega_FM_F) {
    this->omega_FM_F = omega_FM_F;
}

/*! Setter method for omegaPrime_FM_F.
 @return void
 @param omegaPrime_FM_F [rad/s^2] Angular acceleration of the effector body frame F relative to the hub-fixed mount frame M expressed in F frame components
*/
void PrescribedMotionStateEffector::setOmegaPrime_FM_F(const Eigen::Vector3d omegaPrime_FM_F) {
    this->omegaPrime_FM_F = omegaPrime_FM_F;
}

/*! Setter method for sigma_FM.
 @return void
 @param sigma_FM MRP attitude of the effector's body frame F relative to the hub-fixed mount frame M
*/
void PrescribedMotionStateEffector::setSigma_FM(const Eigen::MRPd sigma_FM) {
    this->sigma_FM = sigma_FM;
}

/*! Setter method for r_MB_B.
 @return void
 @param r_MB_B [m] Position vector describing the hub-fixed mount frame origin point M location relative to the hub frame origin point B expressed in B frame components
*/
void PrescribedMotionStateEffector::setR_MB_B(const Eigen::Vector3d r_MB_B) {
    this->r_MB_B = r_MB_B;
}

/*! Setter method for omega_MB_M.
 @return void
 @param omega_MB_M [rad/s] Angular velocity of the hub-fixed mount frame M relative to the hub frame B expressed in M frame components
*/
void PrescribedMotionStateEffector::setOmega_MB_M(const Eigen::Vector3d omega_MB_M) {
    this->omega_MB_M = omega_MB_M;
}

/*! Setter method for omegaPrime_MB_B.
 @return void
 @param omegaPrime_MB_B [rad/s^2] Angular acceleration of the hub-fixed mount frame M relative to the hub frame B expressed in B frame components
*/
void PrescribedMotionStateEffector::setOmegaPrime_MB_B(const Eigen::Vector3d omegaPrime_MB_B) {
    this->omegaPrime_MB_B = omegaPrime_MB_B;
}
/*! Setter method for sigma_MB.
 @return void
 @param sigma_MB MRP attitude of the hub-fixed frame M relative to the hub body frame B
*/
void PrescribedMotionStateEffector::setSigma_MB(const Eigen::MRPd sigma_MB) {
    this->sigma_MB = sigma_MB;
}

/*! Getter method for the effector mass.
 @return double
*/
double PrescribedMotionStateEffector::getMass() const {
    return this->mass;
}

/*! Getter method for IPntFc_F.
 @return const Eigen::Matrix3d
*/
const Eigen::Matrix3d PrescribedMotionStateEffector::getIPntFc_F() const {
    return this->IPntFc_F;
}

/*! Getter method for r_FcF_F.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d PrescribedMotionStateEffector::getR_FcF_F() const {
    return this->r_FcF_F;
}

/*! Getter method for r_FM_M.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d PrescribedMotionStateEffector::getR_FM_M() const {
    return this->r_FM_M;
}

/*! Getter method for rPrime_FM_M.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d PrescribedMotionStateEffector::getRPrime_FM_M() const {
    return this->rPrime_FM_M;
}

/*! Getter method for rPrimePrime_FM_M.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d PrescribedMotionStateEffector::getRPrimePrime_FM_M() const {
    return this->rPrimePrime_FM_M;
}

/*! Getter method for omega_FM_F.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d PrescribedMotionStateEffector::getOmega_FM_F() const {
    return this->omega_FM_F;
}

/*! Getter method for omegaPrime_FM_F.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d PrescribedMotionStateEffector::getOmegaPrime_FM_F() const {
    return this->omegaPrime_FM_F;
}

/*! Getter method for sigma_FM.
 @return const Eigen::MRPd
*/
const Eigen::MRPd PrescribedMotionStateEffector::getSigma_FM() const {
    return this->sigma_FM;
}

/*! Getter method for r_MB_B.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d PrescribedMotionStateEffector::getR_MB_B() const {
    return this->r_MB_B;
}

/*! Getter method for omega_MB_M.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d PrescribedMotionStateEffector::getOmega_MB_M() const {
    return this->omega_MB_M;
}

/*! Getter method for omegaPrime_MB_B.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d PrescribedMotionStateEffector::getOmegaPrime_MB_B() const {
    return this->omegaPrime_MB_B;
}

/*! Getter method for sigma_MB.
 @return const Eigen::MRPd
*/
const Eigen::MRPd PrescribedMotionStateEffector::getSigma_MB() const {
    return this->sigma_MB;
}
