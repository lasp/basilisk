#include "messageProvider.h"
#include "architecture/msgPayloadDefC/CSSConfigMsgPayload.h"


MessageProvider::MessageProvider() {
    this->nHat_B_vec = {
            {0.,      0.,      1.},
            {0.6330,  0.7544,  0.1736},
            {-0.6403, 0.7631,  0.0872},
            {0.5868,  -0.4924, 0.6428},
            {-0.4132, -0.4924, 0.7660},
            {0.6794,  -0.7286, -0.0872},
            {-0.6794, -0.7286, 0.0872},
            {0.5678,  0.7815,  -0.2588},
            {0.5868,  -0.4924, -0.6428},
            {-0.525,  0.7790,  -0.3420},
            {-0.4924, -0.4132, -0.7660},
            {0,       0.6428,  -0.7660}
    };
}

MessageProvider::~MessageProvider() {}

void MessageProvider::Reset(uint64_t CurrentClock) {
    this->writeCSSConfigurationMessage();
    this->writeVehicleConfigurationMessage();
    this->writeRWConfigurationMessage();
}

void MessageProvider::UpdateState(uint64_t CurrentSimNanos) {
    this->updateImuMessage();
    this->updateCSSArraySensorOutMsg();
    this->updateSunDirectionOutMsg();
}

void MessageProvider::SelfInit() {}

void MessageProvider::updateImuMessage() {
    auto imuOutMsgPayload = NavAttMsgPayload();
    setArrayDouble3WithVecDouble3(std::vector<double>{1, 0, 0}, imuOutMsgPayload.omega_BN_B);
    this->imuOutMsg.write(&imuOutMsgPayload, 1, 0);
}

void MessageProvider::writeRWConfigurationMessage() {
    auto payload = RWArrayConfigMsgPayload();
    payload.numRW = 4;
    auto max_rw_momentum = 100.0;
    auto wheel_Js = max_rw_momentum / (6000.0 * M_PI * 2.0 / 60);

    for (int i = 0; i < payload.numRW; ++i) {
        payload.JsList[i] = wheel_Js;
        payload.uMax[i] = max_rw_momentum;
    }

    payload.GsMatrix_B[0] = 1.0 / 2;
    payload.GsMatrix_B[1] = 1. / 2;
    payload.GsMatrix_B[2] = -sqrt(2) / 2;
    payload.GsMatrix_B[3] = 1. / 2;
    payload.GsMatrix_B[4] = 1. / 2;
    payload.GsMatrix_B[5] = sqrt(2) / 2;
    payload.GsMatrix_B[6] = -1. / 2;
    payload.GsMatrix_B[7] = 1. / 2;
    payload.GsMatrix_B[8] = sqrt(2) / 2;
    payload.GsMatrix_B[9] = -1. / 2;
    payload.GsMatrix_B[10] = 1. / 2;
    payload.GsMatrix_B[11] = -sqrt(2) / 2;
    // Define orthogonal RW pyramid
    // -- Pointing directions
    payload.JsList[0] = -20.901 * 0.0254;
    payload.JsList[1] = 29.514 * 0.0254;
    payload.JsList[2] = 52.870 * 0.0254;
    payload.JsList[3] = -20.901 * 0.0254;
    payload.JsList[4] = 29.514 * 0.0254;
    payload.JsList[5] = 32.240 * 0.0254;
    payload.JsList[6] = 20.901 * 0.0254;
    payload.JsList[7] = 29.514 * 0.0254;
    payload.JsList[8] = 32.240 * 0.0254;
    payload.JsList[9] = 20.901 * 0.0254;
    payload.JsList[10] = 29.514 * 0.0254;
    payload.JsList[11] = 52.870 * 0.0254;
    rwConfigOutMsg.write(&payload, 1, 0);
}

void MessageProvider::writeVehicleConfigurationMessage() {
    auto vehicleConfigMsgPayloadGlobal = VehicleConfigMsgPayload();
    setArrayDouble9WithVecDouble9(std::vector{900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0},
                                  vehicleConfigMsgPayloadGlobal.ISCPntB_B);
    vehicleConfigMsgPayloadGlobal.massSC = 2210.0;
    vehConfigOutMsg.write(&vehicleConfigMsgPayloadGlobal, 1, 0);
}

void MessageProvider::writeCSSConfigurationMessage() {
    auto cssConfigMsgPayload = CSSConfigMsgPayload();
    std::vector<CSSUnitConfigMsgPayload> cssConfigMsgs;

    for (auto const &css_hat: this->nHat_B_vec) {
        CSSUnitConfigMsgPayload msgPayload;
        msgPayload.CBias = 1.0;
        setArrayDouble3WithVecDouble3(css_hat, msgPayload.nHat_B);
        cssConfigMsgs.push_back(msgPayload);
    }

    for (int i = 0; i < this->nHat_B_vec.size(); ++i) {
        cssConfigMsgPayload.cssVals[i] = cssConfigMsgs[i];
    }
    cssConfigMsgPayload.nCSS = this->nHat_B_vec.size();
    cssConfigLogOutMsg.write(&cssConfigMsgPayload, 1, 0);
}

void MessageProvider::updateCSSArraySensorOutMsg() {
    auto tempCssArraySensorMsgPayloadGlobal = CSSArraySensorMsgPayload();

    for (int i = 0; i < this->nHat_B_vec.size(); ++i) {
        tempCssArraySensorMsgPayloadGlobal.CosValue[i] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    }
    cssArraySensorOutMsg.write(&tempCssArraySensorMsgPayloadGlobal, 1, 0);
}

void MessageProvider::updateSunDirectionOutMsg() {
    auto sunDirectionOutMsgPayload = NavAttMsgPayload();
    setArrayDouble3WithVecDouble3(std::vector<double>{0, 1, 0}, sunDirectionOutMsgPayload.vehSunPntBdy);
    sunDirectionOutMsg.write(&sunDirectionOutMsgPayload, 1, 0);
}

void MessageProvider::setArrayDouble3WithVecDouble3(std::vector<double> vec, double destination[3]) {
    assert(vec.size() == 3);
    destination[0] = vec[0];
    destination[1] = vec[1];
    destination[2] = vec[2];
}

void MessageProvider::setArrayDouble9WithVecDouble9(std::vector<double> vec, double destination[9]) {
    assert(vec.size() == 9);
    destination[0] = vec[0];
    destination[1] = vec[1];
    destination[2] = vec[2];
    destination[3] = vec[3];
    destination[4] = vec[4];
    destination[5] = vec[5];
    destination[6] = vec[6];
    destination[7] = vec[7];
    destination[8] = vec[8];
}
