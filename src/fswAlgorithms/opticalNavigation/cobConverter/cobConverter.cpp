/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "cobConverter.h"

static Eigen::Matrix3d computeTotalCobCovariance(const Eigen::Matrix3d& covarNav_N,
                                                 const Eigen::Matrix3d& covarAtt_B,
                                                 const Eigen::Matrix3d& covarCob_C,
                                                 const Eigen::Matrix3d& dcm_CN,
                                                 const Eigen::Matrix3d& dcm_CB,
                                                 const Eigen::Matrix3d& cameraCalibrationMatrix);

CobConverter::CobConverter(PhaseAngleCorrectionMethod method, double radiusObject)
{
    phaseAngleCorrectionMethod = method;
    assert(radiusObject > 0);
    objectRadius = radiusObject;
}

CobConverter::~CobConverter() = default;

/*! This method performs a complete reset of the module.  Local module variables that retain time varying states
 * between function calls are reset to their default values.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CobConverter::Reset(uint64_t CurrentSimNanos)
{
    // check that the required message has not been connected
    if (!this->opnavCOBInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "CobConverter.opnavCOBInMsg wasn't connected.");
    }
    if (this->performOutlierDetection && !this->opnavFilterInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "CobConverter.opnavFilterInMsg wasn't connected.");
    }
    if (!this->cameraConfigInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "CobConverter.cameraConfigInMsg wasn't connected.");
    }
    if (!this->navAttInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "CobConverter.navAttInMsg wasn't connected.");
    }
    if (!this->ephemInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "CobConverter.ephemInMsg wasn't connected.");
    }
}

/*! During an update, this module transforms pixel values for the center of brightness into a unit vector
 * direction in several frames (inertial, Camera, and Body).
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CobConverter::UpdateState(uint64_t CurrentSimNanos)
{
    CameraModelMsgPayload cameraSpecs = this->cameraConfigInMsg();
    OpNavCOBMsgPayload cobMsgBuffer = this->opnavCOBInMsg();
    NavAttMsgPayload navAttBuffer = this->navAttInMsg();
    EphemerisMsgPayload ephemBuffer = this->ephemInMsg();

    OpNavUnitVecMsgPayload uVecCOBMsgBuffer;
    uVecCOBMsgBuffer = this->opnavUnitVecCOBOutMsg.zeroMsgPayload;
    OpNavUnitVecMsgPayload uVecCOMMsgBuffer;
    uVecCOMMsgBuffer = this->opnavUnitVecCOMOutMsg.zeroMsgPayload;
    OpNavCOMMsgPayload comMsgBuffer;
    comMsgBuffer = this->opnavCOMOutMsg.zeroMsgPayload;

    if (cobMsgBuffer.valid && cobMsgBuffer.pixelsFound != 0){
        /*! - Extract rotations from relevant messages */
        double CB[3][3];
        double BN[3][3];
        MRP2C(cameraSpecs.bodyToCameraMrp, CB);
        Eigen::Matrix3d dcm_CB = c2DArray2EigenMatrix3d(CB);
        MRP2C(navAttBuffer.sigma_BN, BN);
        Eigen::Matrix3d dcm_BN = c2DArray2EigenMatrix3d(BN);

        Eigen::Matrix3d dcm_NC = dcm_BN.transpose() * dcm_CB.transpose();

        /*! - camera parameters */
        double alpha = 0;
        double fieldOfView = cameraSpecs.fieldOfView[0];
        double resolutionX = cameraSpecs.resolution[0];
        double resolutionY = cameraSpecs.resolution[1];
        double pX = 2.*tan(fieldOfView/2.0);
        double pY = 2.*tan(fieldOfView*resolutionY/resolutionX/2.0);
        double dX = resolutionX/pX;
        double dY = resolutionY/pY;
        double up = resolutionX/2;
        double vp = resolutionY/2;
        double X = 1/dX;
        double Y = 1/dY;
        /*! - build camera calibration matrix K */
        Eigen::Matrix3d cameraCalibrationMatrix;
        cameraCalibrationMatrix << dX, alpha, up,
                                   0., dY, vp,
                                   0., 0., 1.;
        /*! - build inverse K^-1 of camera calibration matrix K */
        Eigen::Matrix3d cameraCalibrationMatrixInverse;
        cameraCalibrationMatrixInverse << 1./dX, -alpha/(dX*dY), (alpha*vp - dY*up)/(dX*dY),
                                          0., 1./dY, -vp/dY,
                                          0., 0., 1.;

        /*! - phase angle correction */
        Eigen::Vector3d rhat_N = cArray2EigenVector3d(ephemBuffer.r_BdyZero_N).normalized();
        double rho = cArray2EigenVector3d(ephemBuffer.r_BdyZero_N).norm();
        Eigen::Vector3d shat_B = cArray2EigenVector3d(navAttBuffer.vehSunPntBdy).normalized();
        Eigen::Vector3d shat_N = dcm_BN.transpose() * shat_B;
        double alphaPA = acos(rhat_N.transpose() * shat_N); // phase angle

        Eigen::Vector3d shat_C = dcm_CB * shat_B;
        double phi = atan2(shat_C[1], shat_C[0]); // sun direction in image plane

        double Rc = this->objectRadius * dX / rho; // object radius in pixels

        double gamma = 0; // offset factor between Center of Mass and Center of Brightness
        bool validCOM = false; // valid COM estimation is false if PhaseAngleCorrectionMethod == NoCorrection
        if(phaseAngleCorrectionMethod == PhaseAngleCorrectionMethod::Lambertian){
            /*! - using phase angle correction assuming Lambertian reflectance sphere according to Shyam Bhaskaran:
             * https://doi.org/10.1109/AERO.1998.687921 */
            gamma = 3.0*M_PI/16.0 * ((cos(alphaPA) + 1.0)*sin(alphaPA))/(sin(alphaPA) + (M_PI - alphaPA)*cos(alphaPA));
            validCOM = true;
        }
        else if(phaseAngleCorrectionMethod == PhaseAngleCorrectionMethod::Binary){
            /*! using phase angle correction assuming a binarized image (brightness either 0 or 1) */
            gamma = 4.0/(3.0*M_PI) * (1.0 - cos(alphaPA));
            validCOM = true;
        }

        /*! - Center of Brightness in pixel space */
        Eigen::Vector3d centerOfBrightness;
        centerOfBrightness[0] = cobMsgBuffer.centerOfBrightness[0];
        centerOfBrightness[1] = cobMsgBuffer.centerOfBrightness[1];
        centerOfBrightness[2] = 1.0;

        /*! - Center of Mass in pixel space */
        Eigen::Vector3d centerOfMass;
        centerOfMass[0] = centerOfBrightness[0] - gamma * Rc * cos(phi);
        centerOfMass[1] = centerOfBrightness[1] - gamma * Rc * sin(phi);
        centerOfMass[2] = 1.0;

        /*! - Get the heading in the image plane */
        Eigen::Vector3d rhatCOB_C = cameraCalibrationMatrixInverse * centerOfBrightness;
        Eigen::Vector3d rhatCOM_C = cameraCalibrationMatrixInverse * centerOfMass;

        /*! - Retrieve the vector from target to camera and normalize */
        rhatCOB_C *= - 1;
        double rhatCOBNorm = rhatCOB_C.norm();
        rhatCOB_C.normalize();
        rhatCOM_C *= - 1;
        rhatCOM_C.normalize();

        /*! - Rotate the vector into frames of interest */
        Eigen::Vector3d rhatCOB_N = dcm_NC * rhatCOB_C;
        Eigen::Vector3d rhatCOB_B = dcm_CB.transpose() * rhatCOB_C;
        Eigen::Vector3d rhatCOM_N = dcm_NC * rhatCOM_C;
        Eigen::Vector3d rhatCOM_B = dcm_CB.transpose() * rhatCOM_C;

        /*! - define diagonal terms of the COB covariance */
        Eigen::Matrix3d covarCob_C;
        covarCob_C.setZero();
        covarCob_C(0,0) = pow(X,2);
        covarCob_C(1,1) = pow(Y,2);
        covarCob_C(2,2) = 1;
        /*! - scale covariance using number of pixels found and rotate into B frame */
        double scaleFactor = sqrt(cobMsgBuffer.pixelsFound / (4 * M_PI)) / pow(rhatCOBNorm, 2);
        covarCob_C *= scaleFactor;
        Eigen::Matrix3d covarCob_B = dcm_CB.transpose() * covarCob_C * dcm_CB;
        /*! - add attitude error covariance in B frame to get total covariance of unit vector measurements */
        Eigen::Matrix3d covar_B = covarCob_B + this->covarAtt_BN_B;
        /*! - rotate total covariance into all remaining frames */
        Eigen::Matrix3d covar_N = dcm_BN.transpose() * covar_B * dcm_BN;
        Eigen::Matrix3d covar_C = dcm_CB.transpose() * covar_B * dcm_CB;

        Eigen::Matrix3d dcm_CN = dcm_NC.transpose();

        bool goodOutlierCheck = true;
        if (this->performOutlierDetection){
            FilterMsgPayload filterMsgBuffer = this->opnavFilterInMsg();

            int numberOfStates = filterMsgBuffer.numberOfStates;
            Eigen::VectorXd filterState = cArray2EigenMatrixXd(filterMsgBuffer.state, numberOfStates, 1);
            Eigen::Vector3d rNav_BN_N = filterState.segment(0, 3);
            Eigen::Vector3d rhatNav_N = rNav_BN_N.normalized();
            Eigen::MatrixXd filterCovariance = cArray2EigenMatrixXd(filterMsgBuffer.covar,
                                                                    numberOfStates,
                                                                    numberOfStates);
            Eigen::Matrix3d covarNav_N = filterCovariance.block(0, 0, 3, 3) / pow(rNav_BN_N.norm(), 2);

            goodOutlierCheck = this->cobOutlierDetection(rhatCOB_C,
                                                         rhatNav_N,
                                                         covarNav_N,
                                                         covarCob_C,
                                                         dcm_CN,
                                                         dcm_CB,
                                                         cameraCalibrationMatrix);
        }

        /*! - output messages */
        eigenMatrix3d2CArray(covar_N, uVecCOBMsgBuffer.covar_N);
        eigenMatrix3d2CArray(covar_C, uVecCOBMsgBuffer.covar_C);
        eigenMatrix3d2CArray(covar_B, uVecCOBMsgBuffer.covar_B);
        eigenVector3d2CArray(rhatCOB_N, uVecCOBMsgBuffer.rhat_BN_N);
        eigenVector3d2CArray(rhatCOB_C, uVecCOBMsgBuffer.rhat_BN_C);
        eigenVector3d2CArray(rhatCOB_B, uVecCOBMsgBuffer.rhat_BN_B);
        uVecCOBMsgBuffer.timeTag = (double) cobMsgBuffer.timeTag * NANO2SEC;
        uVecCOBMsgBuffer.valid = goodOutlierCheck;

        eigenMatrix3d2CArray(covar_N, uVecCOMMsgBuffer.covar_N);
        eigenMatrix3d2CArray(covar_C, uVecCOMMsgBuffer.covar_C);
        eigenMatrix3d2CArray(covar_B, uVecCOMMsgBuffer.covar_B);
        eigenVector3d2CArray(rhatCOM_N, uVecCOMMsgBuffer.rhat_BN_N);
        eigenVector3d2CArray(rhatCOM_C, uVecCOMMsgBuffer.rhat_BN_C);
        eigenVector3d2CArray(rhatCOM_B, uVecCOMMsgBuffer.rhat_BN_B);
        uVecCOMMsgBuffer.timeTag = (double) cobMsgBuffer.timeTag * NANO2SEC;
        uVecCOMMsgBuffer.valid = (validCOM && goodOutlierCheck);

        comMsgBuffer.centerOfMass[0] = centerOfMass[0];
        comMsgBuffer.centerOfMass[1] = centerOfMass[1];
        comMsgBuffer.offsetFactor = gamma;
        comMsgBuffer.objectPixelRadius = int(Rc);
        comMsgBuffer.phaseAngle = alphaPA;
        comMsgBuffer.sunDirection = phi;
        comMsgBuffer.cameraID = cameraSpecs.cameraId;
        comMsgBuffer.timeTag = cobMsgBuffer.timeTag;
        comMsgBuffer.valid = validCOM;
    }

    this->opnavUnitVecCOBOutMsg.write(&uVecCOBMsgBuffer, this->moduleID, CurrentSimNanos);
    this->opnavUnitVecCOMOutMsg.write(&uVecCOMMsgBuffer, this->moduleID, CurrentSimNanos);
    this->opnavCOMOutMsg.write(&comMsgBuffer, this->moduleID, CurrentSimNanos);
}

/*! Compute the total COB covariance matrix in pixel units (given unit vector covariances)
    @param Eigen::Matrix3d& covarNav_N
    @param Eigen::Matrix3d& covarAtt_B
    @param Eigen::Matrix3d& covarCob_C
    @param Eigen::Matrix3d& dcm_CN
    @param Eigen::Matrix3d& dcm_CB
    @param Eigen::Matrix3d& cameraCalibrationMatrix
    @return Eigen::Matrix3d covarImage
    */
static Eigen::Matrix3d computeTotalCobCovariance(const Eigen::Matrix3d& covarNav_N,
                                                 const Eigen::Matrix3d& covarAtt_B,
                                                 const Eigen::Matrix3d& covarCob_C,
                                                 const Eigen::Matrix3d& dcm_CN,
                                                 const Eigen::Matrix3d& dcm_CB,
                                                 const Eigen::Matrix3d& cameraCalibrationMatrix)
{
    Eigen::Matrix3d covarAtt_C = dcm_CB * covarAtt_B * dcm_CB.transpose();
    Eigen::Matrix3d covarNav_C = dcm_CN * covarNav_N * dcm_CN.transpose();
    Eigen::Matrix3d covarTotal_C = covarCob_C + covarAtt_C + covarNav_C;
    Eigen::Matrix3d covarImage = cameraCalibrationMatrix * covarTotal_C * cameraCalibrationMatrix.transpose();

    return covarImage;
}

/*! Outlier detection on the COB
    @param Eigen::Vector3d& rhatCOB_C
    @param Eigen::Vector3d& rhatNav_N [m]
    @param Eigen::Matrix3d& covarNav_N [px]
    @param Eigen::Matrix3d& covarCob_C [px]
    @param Eigen::Matrix3d& dcm_CN
    @param Eigen::Matrix3d& dcm_CB
    @param Eigen::Matrix3d& cameraCalibrationMatrix
    @return bool goodOutlierCheck
    */
bool CobConverter::cobOutlierDetection(Eigen::Vector3d& rhatCOB_C,
                                       const Eigen::Vector3d& rhatNav_N,
                                       const Eigen::Matrix3d& covarNav_N,
                                       const Eigen::Matrix3d& covarCob_C,
                                       const Eigen::Matrix3d& dcm_CN,
                                       const Eigen::Matrix3d& dcm_CB,
                                       const Eigen::Matrix3d& cameraCalibrationMatrix) const
{
    rhatCOB_C *= - 1;  // turn unit vector from asteroid to camera into unit vector from camera to asteroid
    rhatCOB_C /= rhatCOB_C[2];  // make z-component 1 for image plane
    Eigen::Vector3d cob = cameraCalibrationMatrix * rhatCOB_C;

    // assume that the time of the last filter update corresponds to the current timestep (so no propagation required)
    Eigen::Vector3d rhatNav_C = (dcm_CN * rhatNav_N);
    rhatNav_C *= - 1;
    rhatNav_C /= rhatNav_C[2];
    Eigen::Vector3d cobNav = cameraCalibrationMatrix * rhatNav_C;

    double cobErrorPrediction = (cob - cobNav).norm();
    double sigma;
    if (this->specifiedStandardDeviation) {
        sigma = this->standardDeviation;
    } else {
        Eigen::Matrix3d covarImage = computeTotalCobCovariance(covarNav_N,
                                                               this->covarAtt_BN_B,
                                                               covarCob_C,
                                                               dcm_CN,
                                                               dcm_CB,
                                                               cameraCalibrationMatrix);
        sigma = sqrt(std::max(covarImage(0, 0), covarImage(1, 1)));
    }

    bool goodOutlierCheck = cobErrorPrediction < this->numStandardDeviations * sigma;

    return goodOutlierCheck;
}

/*! Set the object radius
    @param double radiusInput [m]
    @return void
    */
void CobConverter::setRadius(const double radius){
    assert(radius > 0);
    this->objectRadius = radius;
}

/*! Get the object radius
    @return double radius [m]
    */
double CobConverter::getRadius() const {
    return this->objectRadius;
}

/*! Set the attitude error covariance matrix in body frame B, for unit vector measurements
    @param cov_att_BN_B
    @return void
    */
void CobConverter::setAttitudeCovariance(const Eigen::Matrix3d covAtt_BN_B)
{
    this->covarAtt_BN_B = covAtt_BN_B;
}

/*! Get the attitude error covariance matrix in body frame B, for unit vector measurements
    @return Eigen::Matrix3d cov_att_BN_B
    */
Eigen::Matrix3d CobConverter::getAttitudeCovariance() const
{
    return this->covarAtt_BN_B;
}

/*! Set the number of standard deviations that are acceptable for the expected COB error
    @param double numStandardDeviations
    @return void
    */
void CobConverter::setNumStandardDeviations(const double num){
    assert(num > 0.0);
    this->numStandardDeviations = num;
}

/*! Get the number of standard deviations that are acceptable for the expected COB error
    @return double numStandardDeviations
    */
double CobConverter::getNumStandardDeviations() const {
    return this->numStandardDeviations;
}

/*! Set the accepted standard deviation for the expected COB error
    @return void
    */
void CobConverter::setStandardDeviation(const double num){
    assert(num > 0.0);
    this->standardDeviation = num;
    this->specifiedStandardDeviation = true;
}

/*! Get the accepted standard deviation for the expected COB error
    @return double numStandardDeviations
    */
double CobConverter::getStandardDeviation() const {
    return this->standardDeviation;
}

/*! Get whether or not a standard deviation is set
    @return bool specifiedStandardDeviation
    */
bool CobConverter::isStandardDeviationSpecified() const {
    return this->specifiedStandardDeviation;
}

/*! Enable the COB outlier detection
    @return void
    */
void CobConverter::enableOutlierDetection(){
    this->performOutlierDetection = true;
}

/*! Disable the COB outlier detection
    @return void
    */
void CobConverter::disableOutlierDetection(){
    this->performOutlierDetection = false;
}

/*! Get whether or not the COB outlier detection is performed
    @return bool performOutlierDetection
    */
bool CobConverter::isOutlierDetectionEnabled() const {
    return this->performOutlierDetection;
}
