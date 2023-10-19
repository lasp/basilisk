#ifndef BASILISK_MESSAGEPROVIDER_H
#define BASILISK_MESSAGEPROVIDER_H

#include <vector>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/gauss_markov.h"
#include "architecture/utilities/saturate.h"

#include "architecture/msgPayloadDefC/AlbedoMsgPayload.h"
#include "architecture/msgPayloadDefC/CSSArraySensorMsgPayload.h"
#include "architecture/msgPayloadDefCpp/CSSConfigLogMsgPayload.h"
#include "architecture/msgPayloadDefC/CSSConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/CSSRawDataMsgPayload.h"
#include "architecture/msgPayloadDefC/EclipseMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/RWArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "cMsgCInterface/VehicleConfigMsg_C.h"
#include "cMsgCInterface/RateCmdMsg_C.h"

#include <Eigen/Dense>

class MessageProvider : public SysModel {
public:
    MessageProvider();

    ~MessageProvider();

    void Reset(uint64_t CurrentClock) override;

    void UpdateState(uint64_t CurrentSimNanos) override;

    void SelfInit() override;

public:
    Message<CSSArraySensorMsgPayload> cssArraySensorOutMsg;
    Message<CSSConfigMsgPayload> cssConfigLogOutMsg;
    Message<NavAttMsgPayload> sunDirectionOutMsg;
    Message<VehicleConfigMsgPayload> vehConfigOutMsg;
    Message<RWArrayConfigMsgPayload> rwConfigOutMsg;
    Message<NavAttMsgPayload> imuOutMsg;
    RateCmdMsg_C rateSteeringOutMsg;

private:
    void writeCSSConfigurationMessage();

    void writeVehicleConfigurationMessage();

    void writeRWConfigurationMessage();

    void updateCSSArraySensorOutMsg();

    void updateSunDirectionOutMsg();

    void updateImuMessage();

    void setArrayDouble3WithVecDouble3(std::vector<double> vec, double destination[3]);

    void setArrayDouble9WithVecDouble9(std::vector<double> vec, double destination[9]);

    std::vector<std::vector<double>> nHat_B_vec;
};


#endif //BASILISK_MESSAGEPROVIDER_H
