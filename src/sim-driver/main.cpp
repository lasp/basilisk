#include "iostream"
#include "simulationDriver.h"
// #include "fswAlgorithms/opticalNavigation/cobConverter/cobConverter.h"
// #include "fswAlgorithms/imageProcessing/centerOfBrightness/centerOfBrightness.h"
#include "fswAlgorithms/attGuidance/flybyPoint/flybyPoint.h"
#include "fswAlgorithms/attGuidance/attTrackingError/attTrackingError.h"
#include "fswAlgorithms/opticalNavigation/flybyODuKF/flybyODuKF.h"
#include "architecture/alg_contain/alg_contain.h"
//#include "../dist3/autoSource/cMsgCInterface/CameraConfigMsg_C.h"
//#include "/Users/pake0095/Documents/Repositories/lasp-basilisk-redux/dist3/autoSource/cMsgCInterface/OpNavCOBMsgPayload_C.h"
#include "architecture/messaging/messaging.h"

// void setCenterOfBrightness(CenterOfBrightness* model);
// void setCobConverter(CobConverter* model);
void setFlybyODuKF(FlybyODuKF* model);
void setFlybyPoint(FlybyPoint* model);

int main (int argc, char* argv[] ) {
    auto simDriver = SimulationDriver::SimulationDriver();
    simDriver.setStopTime(10000000000);
    auto proc = simDriver.createProcess("proc1", 1);
    auto taskTalonsFlyby = simDriver.createTask("talonsFlyby", 10000000);
    proc->addNewTask(taskTalonsFlyby.get());

    // auto center_of_brightness = new CenterOfBrightness;
    // setCenterOfBrightness(center_of_brightness);
    // auto cob_converter = new CobConverter();
    // setCobConverter(cob_converter);
    auto flyby_od = new FlybyODuKF();
    setFlybyODuKF(flyby_od);
    auto flyby_guid = new FlybyPoint();
    setFlybyPoint(flyby_guid);

    auto tracking_error_cam_config = new attTrackingErrorConfig();
    AlgPtr selfInitFunc = reinterpret_cast<AlgPtr>(SelfInit_attTrackingError);
    AlgUpdatePtr updateFunc = reinterpret_cast<AlgUpdatePtr>(Update_attTrackingError);
    AlgUpdatePtr resetFunc = reinterpret_cast<AlgUpdatePtr>(Reset_attTrackingError);
    auto tracking_error_cam_container = new AlgContain();
    tracking_error_cam_container->UseData(tracking_error_cam_config);
    tracking_error_cam_container->UseSelfInit(selfInitFunc);
    tracking_error_cam_container->UseUpdate(updateFunc);
    tracking_error_cam_container->UseReset(resetFunc);

    // taskTalonsFlyby->AddNewObject((SysModel *)center_of_brightness, 15);
    // taskTalonsFlyby->AddNewObject((SysModel *)cob_converter, 12);
    taskTalonsFlyby->AddNewObject((SysModel *)flyby_od, 9);
    taskTalonsFlyby->AddNewObject((SysModel *)flyby_guid, 8);
    taskTalonsFlyby->AddNewObject((SysModel *)tracking_error_cam_container, 7);

    // flyby_od->opNavHeadingMsg.subscribeTo(&cob_converter->opnavUnitVecOutMsg);


    // double cameraResolution[2] = {2048, 2048};
    // double sigma_CB[3] = {-1., -0.3, -0.1};
    double sigma_BN[3] = {-0.6, -1., -0.1};
    // double centerOfBrightness[2] = {1021, 1891};
    // int numberOfPixels = 1000;

    // // Set camera parameters
    // auto inputCamera = CameraConfigMsgPayload();
    // auto camInMsg = Message<CameraConfigMsgPayload>();
    // // inputCamera.fieldOfView = 2.0 * math.arctan(10*1e-3 / 2.0 / (1.*1e-3) )  # 2*arctan(size/2 / focal)
    // memcpy(inputCamera.resolution, cameraResolution, sizeof(inputCamera.resolution));
    // memcpy(inputCamera.sigma_CB, sigma_CB, sizeof(inputCamera.sigma_CB));
    // camInMsg.write(&inputCamera, 1, 0);
    // cob_converter->cameraConfigInMsg.subscribeTo(&camInMsg);

    // // Set center of brightness
    // auto inputCob = OpNavCOBMsgPayload();
    // auto cobInMsg = Message<OpNavCOBMsgPayload>();
    // memcpy(inputCob.centerOfBrightness, centerOfBrightness, sizeof(inputCob.centerOfBrightness));
    // inputCob.pixelsFound = numberOfPixels;
    // inputCob.timeTag = 12345;
    // cobInMsg.write(&inputCob, 1, 0);
    // cob_converter->opnavCOBInMsg.subscribeTo(&cobInMsg);

    // Set body attitude relative to inertial
    auto inputAtt = NavAttMsgPayload();
    auto inputAttInMsg = Message<NavAttMsgPayload>();
    memcpy(inputAtt.sigma_BN, sigma_BN, sizeof(inputAtt.sigma_BN));
    inputAttInMsg.write(&inputAtt, 1, 0);
    // cob_converter->navAttInMsg.subscribeTo(&inputAttInMsg);

    //    taskTalonsFlyby->AddNewObject(tracking_error_cam, 6);

    flyby_guid->filterInMsg.subscribeTo(&flyby_od->navTransOutMsg);

    simDriver.run();
}

// void setCobConverter(CobConverter* model) {
//     model->ModelTag = "CobConverter";
// }

void setFlybyODuKF(FlybyODuKF* model) {
    model->ModelTag = "FlybyODuKF";
}

void setFlybyPoint(FlybyPoint* model) {
    model->ModelTag = "FlybyPoint";
}

// void setCenterOfBrightness(CenterOfBrightness* model) {
//     model->filename = "imagePath";
//     model->blurSize = 5;
//     model->threshold = 50;
// }
