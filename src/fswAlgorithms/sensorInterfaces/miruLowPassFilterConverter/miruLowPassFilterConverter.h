#ifndef _MIRULOWPASSFILTERCONVERTER_
#define _MIRULOWPASSFILTERCONVERTER_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/msgPayloadDefC/AccDataMsgPayload.h"
#include "architecture/msgPayloadDefC/IMUSensorMsgPayload.h"
#include "architecture/utilities/signalProcessing.h"
#include <Eigen/Core>

/*! @brief Convert AccDataMsgPayload to IMUSensorMsgPayload Class */
class MiruLowPassFilterConverter: public SysModel {
public:
    MiruLowPassFilterConverter() = default;                         //!< Constructor
    ~MiruLowPassFilterConverter() = default;                        //!< Destructor

    void Reset(uint64_t CurrentSimNanos);                           //!< Reset member function
    void UpdateState(uint64_t CurrentSimNanos);                     //!< Update member function
    void setLowPassFilter(double step, double frequencyCutOff);     //!< Setter method for the low pass filter

    ReadFunctor<AccDataMsgPayload> imuAccelDataInMsg;               //!< Input msg for the imu data
    Message<IMUSensorMsgPayload> imuSensorOutMsg;                   //!< Output msg for the imu data

    BSKLogger *bskLogger;                                           //!< BSK Logging

private:
    double cutOffFrequency = 15.0/(2*M_PI);                         //!< Low pass filter parameter
    double hStep = 0.5;                                             //!< Low pass filter parameter
};

#endif
