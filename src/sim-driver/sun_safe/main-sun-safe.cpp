#include "simulationDriver.h"
#include "messageProvider.h"

#include "architecture/alg_contain/alg_contain.h"
#include "fswAlgorithms/attControl/mrpPD/mrpPD.h"
#include "fswAlgorithms/attDetermination/CSSEst/cssWlsEst.h"
#include "fswAlgorithms/attGuidance/sunSafePoint/sunSafePoint.h"
#include "fswAlgorithms/effectorInterfaces/thrForceMapping/thrForceMapping.h"
#include "fswAlgorithms/effectorInterfaces/thrFiringSchmitt/thrFiringSchmitt.h"

void setArrayDouble3WithVecDouble3(std::vector<double> vec, double destination[3]);
void setArrayDouble9WithVecDouble9(std::vector<double> vec, double destination[9]);

int main (int argc, char* argv[] ) {
    auto simDriver = SimulationDriver::SimulationDriver();
    simDriver.setStopTime(10000000000);
    auto proc = simDriver.createProcess("proc1", 1);
    auto taskSunSafe = simDriver.createTask("sunSafe", 10000000);
    proc->addNewTask(taskSunSafe.get());
    auto logger = BSKLogger();
    auto message_provider = new MessageProvider();

     // CSS Weighted Least Squares Estimator
    auto css_wls_config = new CSSWLSConfig();
    css_wls_config->bskLogger = &logger;

    CSSArraySensorMsg_cpp_subscribe(&css_wls_config->cssDataInMsg,
                                    &message_provider->cssArraySensorOutMsg);

    CSSConfigMsg_cpp_subscribe(&css_wls_config->cssConfigInMsg,
                               &message_provider->cssConfigLogOutMsg);

    AlgPtr cssWlsEstSelfInitFunc = reinterpret_cast<AlgPtr>(SelfInit_cssWlsEst);
    AlgUpdatePtr cssWlsEstUpdateFunc = reinterpret_cast<AlgUpdatePtr>(Update_cssWlsEst);
    AlgUpdatePtr cssWlsEstResetFunc = reinterpret_cast<AlgUpdatePtr>(Reset_cssWlsEst);
    auto css_wls_container = new AlgContain();
    css_wls_container->ModelTag = "css_wls";
    css_wls_container->UseData(css_wls_config);
    css_wls_container->UseSelfInit(cssWlsEstSelfInitFunc);
    css_wls_container->UseUpdate(cssWlsEstUpdateFunc);
    css_wls_container->UseReset(cssWlsEstResetFunc);

    // Sun Safe Point
    auto sun_safe_point_config = new sunSafePointConfig ();
    sun_safe_point_config->bskLogger = &logger;
    setArrayDouble3WithVecDouble3(std::vector<double>{0.0, 0.0, 1.0}, sun_safe_point_config->sHatBdyCmd);
    AlgPtr sunSafePointSelfInitFunc = reinterpret_cast<AlgPtr>(SelfInit_sunSafePoint);
    AlgUpdatePtr sunSafePointUpdateFunc = reinterpret_cast<AlgUpdatePtr>(Update_sunSafePoint);
    AlgUpdatePtr sunSafePointResetFunc = reinterpret_cast<AlgUpdatePtr>(Reset_sunSafePoint);
    auto sun_safe_point_container = new AlgContain();
    sun_safe_point_container->ModelTag = "sun_safe_point";
    sun_safe_point_container->UseData(sun_safe_point_config);
    sun_safe_point_container->UseSelfInit(sunSafePointSelfInitFunc);
    sun_safe_point_container->UseUpdate(sunSafePointUpdateFunc);
    sun_safe_point_container->UseReset(sunSafePointResetFunc);

    NavAttMsg_cpp_subscribe(&sun_safe_point_config->sunDirectionInMsg,
                            &message_provider->sunDirectionOutMsg);
    NavAttMsg_cpp_subscribe(&sun_safe_point_config->imuInMsg,
                            &message_provider->imuOutMsg);

    // MRP PD Controller
    auto mrp_pd_control_config = new MrpPDConfig ();
    mrp_pd_control_config->K = 4.652242213692045;
    mrp_pd_control_config->P = 97.15034930362553;
    mrp_pd_control_config->bskLogger = &logger;
    AlgPtr mrpPDSelfInitFunc = reinterpret_cast<AlgPtr>(SelfInit_mrpPD);
    AlgUpdatePtr mrpPDUpdateFunc = reinterpret_cast<AlgUpdatePtr>(Update_mrpPD);
    AlgUpdatePtr mrpPDResetFunc = reinterpret_cast<AlgUpdatePtr>(Reset_mrpPD);
    auto mrp_pd_container = new AlgContain();
    mrp_pd_container->ModelTag = "mrp_pd";
    mrp_pd_container->UseData(mrp_pd_control_config);
    mrp_pd_container->UseSelfInit(mrpPDSelfInitFunc);
    mrp_pd_container->UseUpdate(mrpPDUpdateFunc);
    mrp_pd_container->UseReset(mrpPDResetFunc);

    AttGuidMsg_C_subscribe(&mrp_pd_control_config->guidInMsg,
                           &sun_safe_point_config->attGuidanceOutMsg);
    VehicleConfigMsg_cpp_subscribe(&mrp_pd_control_config->vehConfigInMsg,
                                   &message_provider->vehConfigOutMsg);

    // Thruster Force Mapping
    auto thruster_force_mapping_config = new thrForceMappingConfig();
    thruster_force_mapping_config->thrForceSign = +1;
    setArrayDouble9WithVecDouble9(std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0},
                                  thruster_force_mapping_config->controlAxes_B);
    thruster_force_mapping_config->numThrusters = 8;
    thruster_force_mapping_config->bskLogger = &logger;
    AlgPtr thrForceMappingSelfInitFunc = reinterpret_cast<AlgPtr>(SelfInit_thrForceMapping);
    AlgUpdatePtr thrForceMappingUpdateFunc = reinterpret_cast<AlgUpdatePtr>(Update_thrForceMapping);
    AlgUpdatePtr thrForceMappingResetFunc = reinterpret_cast<AlgUpdatePtr>(Reset_thrForceMapping);
    auto thruster_force_mapping_container = new AlgContain();
    thruster_force_mapping_container->ModelTag = "thruster_force_mapping";
    thruster_force_mapping_container->UseData(thruster_force_mapping_config);
    thruster_force_mapping_container->UseSelfInit(thrForceMappingSelfInitFunc);
    thruster_force_mapping_container->UseUpdate(thrForceMappingUpdateFunc);
    thruster_force_mapping_container->UseReset(thrForceMappingResetFunc);

    CmdTorqueBodyMsg_C_subscribe(&thruster_force_mapping_config->cmdTorqueInMsg,
                           &mrp_pd_control_config->cmdTorqueOutMsg);
    THRArrayConfigMsg_cpp_subscribe(&thruster_force_mapping_config->thrConfigInMsg,
                           &message_provider->thrusterArrayConfigMsg);
    VehicleConfigMsg_cpp_subscribe(&thruster_force_mapping_config->vehConfigInMsg,
                                   &message_provider->vehConfigOutMsg);

    // Thruster Firing Schmitt
    auto thruster_firing_schmitt_config = new thrFiringSchmittConfig();
    thruster_firing_schmitt_config->thrMinFireTime = 0.0001;
    thruster_firing_schmitt_config->level_on = .75;
    thruster_firing_schmitt_config->level_off = .25;
    AlgPtr thrFiringSchmittSelfInitFunc = reinterpret_cast<AlgPtr>(SelfInit_thrFiringSchmitt);
    AlgUpdatePtr thrFiringSchmittUpdateFunc = reinterpret_cast<AlgUpdatePtr>(Update_thrFiringSchmitt);
    AlgUpdatePtr thrFiringSchmittResetFunc = reinterpret_cast<AlgUpdatePtr>(Reset_thrFiringSchmitt);
    auto thruster_firing_schmitt_container = new AlgContain();
    thruster_firing_schmitt_container->ModelTag = "thruster_firing_schmitt";
    thruster_firing_schmitt_container->UseData(thruster_firing_schmitt_config);
    thruster_firing_schmitt_container->UseSelfInit(thrFiringSchmittSelfInitFunc);
    thruster_firing_schmitt_container->UseUpdate(thrFiringSchmittUpdateFunc);
    thruster_firing_schmitt_container->UseReset(thrFiringSchmittResetFunc);

    THRArrayCmdForceMsg_C_subscribe(&thruster_firing_schmitt_config->thrForceInMsg,
                                    &thruster_force_mapping_config->thrForceCmdOutMsg);
    THRArrayConfigMsg_cpp_subscribe(&thruster_firing_schmitt_config->thrConfInMsg,
                                    &message_provider->thrusterArrayConfigMsg);
    THRArrayOnTimeCmdMsg_C_addAuthor(&thruster_firing_schmitt_config->onTimeOutMsg,
                                     &message_provider->thrOnTimeCmdMsg);

    taskSunSafe->AddNewObject(message_provider, 11);
    taskSunSafe->AddNewObject(css_wls_container, 10);
    taskSunSafe->AddNewObject(sun_safe_point_container, 9);
    taskSunSafe->AddNewObject(mrp_pd_container, 8);
    taskSunSafe->AddNewObject(thruster_force_mapping_container, 7);
    taskSunSafe->AddNewObject(thruster_firing_schmitt_container, 6);

    simDriver.run();
}

void setArrayDouble3WithVecDouble3(std::vector<double> vec, double destination[3]) {
    assert(vec.size() == 3);
    destination[0] = vec[0];
    destination[1] = vec[1];
    destination[2] = vec[2];
}

void setArrayDouble9WithVecDouble9(std::vector<double> vec, double destination[9]) {
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
