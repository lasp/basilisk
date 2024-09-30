#include "miruLowPassFilterConverter.h"
#include "architecture/utilities/avsEigenSupport.h"

/*! This method checks the input message to ensure it is linked.
 @return void
 @param callTime [ns] Time the method is called
*/
void MiruLowPassFilterConverter::Reset(uint64_t callTime) {
    if (!this->imuAccelDataInMsg.isLinked()) {
        this->bskLogger->bskLog(BSK_ERROR, "miruLowPassFilterConverter.imuAccelDataInMsg wasn't connected.");
    }
}

/*! This method writes the module output message using the given input message.
 @return void
 @param callTime [ns] Time the method is called
*/
void MiruLowPassFilterConverter::UpdateState(uint64_t callTime) {
    auto lowPass = LowPassFilter();
    int smallestFutureIndex = 0;
    int numberOfValidGyroMeasurements = 0;
    double firstFutureTime = -1;
    double meanMeasurementTime = 0;
    AccDataMsgPayload gyrBuffer = this->imuAccelDataInMsg();
    for (int index = 0; index < MAX_ACC_BUF_PKT; index++) {
        double gyroMeasuredTime = gyrBuffer.accPkts[index].measTime*NANO2SEC;
        if (gyroMeasuredTime < firstFutureTime || firstFutureTime<0){
            smallestFutureIndex = index;
            firstFutureTime = gyroMeasuredTime;
        }
        meanMeasurementTime += gyroMeasuredTime;
        numberOfValidGyroMeasurements += 1;
    }
    lowPass.setFilterCutoff(this->cutOffFrequency);
    lowPass.setFilterStep(this->hStep);
    if (numberOfValidGyroMeasurements > 0){
        meanMeasurementTime /= numberOfValidGyroMeasurements;
        /*! - Loop through buffer for all future measurements since the previous time to filter omega_BN_B*/
        for (int index = 0; index < MAX_ACC_BUF_PKT; index++) {
            int shiftedIndex = (index + smallestFutureIndex) % MAX_ACC_BUF_PKT;
            auto omega_BN_B = Eigen::Map<Eigen::Vector3d>(gyrBuffer.accPkts[shiftedIndex].gyro_B);
            /*! - Apply low-pass filter to gyro measurements to get smoothed body rate*/
            lowPass.processMeasurement(omega_BN_B);
        }
    }

    // Write the output message
    IMUSensorMsgPayload imuSensorOut = IMUSensorMsgPayload();
    imuSensorOut.timeTag = meanMeasurementTime;
    imuSensorOut.numberOfValidGyroMeasurements = numberOfValidGyroMeasurements;
    Eigen::Vector3d omega_BN_B = lowPass.getCurrentState();
    eigenVector3d2CArray(omega_BN_B, imuSensorOut.AngVelPlatform);
    this->imuSensorOutMsg.write(&imuSensorOut, moduleID, callTime);
}

/*! Set the low pass filter parameters
 @param double step
 @param double frequencyCutOff
*/
void MiruLowPassFilterConverter::setLowPassFilter(double step, double frequencyCutOff) {
    this->hStep = step;
    this->cutOffFrequency = frequencyCutOff;
}
