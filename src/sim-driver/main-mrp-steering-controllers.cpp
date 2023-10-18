#include "simulationDriver.h"
#include "messageProvider.h"

#include "architecture/alg_contain/alg_contain.h"
#include "fswAlgorithms/attDetermination/CSSEst/cssWlsEst.h"
#include "fswAlgorithms/attControl/mrpSteering/mrpSteering.h"
#include "fswAlgorithms/attControl/rateServoFullNonlinear/rateServoFullNonlinear.h"
#include "fswAlgorithms/effectorInterfaces/rwMotorTorque/rwMotorTorque.h"
#include "fswAlgorithms/attGuidance/sunSafePoint/sunSafePoint.h"
#include "fswAlgorithms/opticalNavigation/flybyODuKF/flybyODuKF.h"

void setArrayDouble3WithVecDouble3(std::vector<double> vec, double destination[3]);
void setArrayDouble9WithVecDouble9(std::vector<double> vec, double destination[9]);

int main (int argc, char* argv[] ) {
    auto simDriver = SimulationDriver::SimulationDriver();
    simDriver.setStopTime(100000000000);
    auto proc = simDriver.createProcess("proc1", 1);
    auto attitudeControlTask = simDriver.createTask("talonsFlyby", 10000000);
    proc->addNewTask(attitudeControlTask.get());
    auto logger = BSKLogger();
    auto message_provider = new MessageProvider();

    // CSS Weighted Least Squares Estimator
    auto css_wls_config = new CSSWLSConfig ();
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

    // MRP Steering Controller
    auto mrp_steering_config = new mrpSteeringConfig();
    mrp_steering_config->K1 = 0.05;
    mrp_steering_config->ignoreOuterLoopFeedforward = false;
    mrp_steering_config->K3 = 0.75;
    mrp_steering_config->omega_max = 1.0 * M_PI/180.0;
    mrp_steering_config->bskLogger = &logger;
    AlgPtr mrpSteeringSelfInitFunc = reinterpret_cast<AlgPtr>(SelfInit_mrpSteering);
    AlgUpdatePtr mrpSteeringUpdateFunc = reinterpret_cast<AlgUpdatePtr>(Update_mrpSteering);
    AlgUpdatePtr mrpSteeringResetFunc = reinterpret_cast<AlgUpdatePtr>(Reset_mrpSteering);
    auto mrp_steering_container = new AlgContain();
    mrp_steering_container->ModelTag = "mrp_steering";
    mrp_steering_container->UseData(mrp_steering_config);
    mrp_steering_container->UseSelfInit(mrpSteeringSelfInitFunc);
    mrp_steering_container->UseUpdate(mrpSteeringUpdateFunc);
    mrp_steering_container->UseReset(mrpSteeringResetFunc);

    AttGuidMsg_C_subscribe(&mrp_steering_config->guidInMsg, &sun_safe_point_config->attGuidanceOutMsg);

    // Rate Servo Controller
    auto rate_servo_config = new rateServoFullNonlinearConfig();
    rate_servo_config->Ki = 5.0;
    rate_servo_config->P = 150.0;
    rate_servo_config->integralLimit = 2. / rate_servo_config->Ki * 0.1;
    rate_servo_config->knownTorquePntB_B[0] = 0.0;
    rate_servo_config->knownTorquePntB_B[1] = 0.0;
    rate_servo_config->knownTorquePntB_B[2] = 0.0;
    rate_servo_config->bskLogger = &logger;
    AlgPtr rateServoFullNonlinearSelfInitFunc = reinterpret_cast<AlgPtr>(SelfInit_rateServoFullNonlinear);
    AlgUpdatePtr rateServoFullNonlinearUpdateFunc = reinterpret_cast<AlgUpdatePtr>(Update_rateServoFullNonlinear);
    AlgUpdatePtr rateServoFullNonlinearResetFunc = reinterpret_cast<AlgUpdatePtr>(Reset_rateServoFullNonlinear);
    auto rate_servo_container = new AlgContain();
    rate_servo_container->ModelTag = "rate_servo";
    rate_servo_container->UseData(rate_servo_config);
    rate_servo_container->UseSelfInit(rateServoFullNonlinearSelfInitFunc);
    rate_servo_container->UseUpdate(rateServoFullNonlinearUpdateFunc);
    rate_servo_container->UseReset(rateServoFullNonlinearResetFunc);

    RateCmdMsg_C_subscribe(&rate_servo_config->rateSteeringInMsg,
                           &mrp_steering_config->rateCmdOutMsg);
    AttGuidMsg_C_subscribe(&rate_servo_config->guidInMsg,
                           &sun_safe_point_config->attGuidanceOutMsg);
    VehicleConfigMsg_cpp_subscribe(&rate_servo_config->vehConfigInMsg,
                                   &message_provider->vehConfigOutMsg);

    // Reaction Wheel Motor Torque Mapper
    auto rw_motor_torque_config = new rwMotorTorqueConfig();
    rw_motor_torque_config->bskLogger = &logger;
    setArrayDouble9WithVecDouble9(std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0},
                                  rw_motor_torque_config->controlAxes_B);
    AlgPtr rwMotorTorqueSelfInitFunc = reinterpret_cast<AlgPtr>(SelfInit_rwMotorTorque);
    AlgUpdatePtr rwMotorTorqueUpdateFunc = reinterpret_cast<AlgUpdatePtr>(Update_rwMotorTorque);
    AlgUpdatePtr rwMotorTorqueResetFunc = reinterpret_cast<AlgUpdatePtr>(Reset_rwMotorTorque);
    auto rw_motor_torque_container = new AlgContain();
    rw_motor_torque_container->ModelTag = "rw_motor_torque";
    rw_motor_torque_container->UseData(rw_motor_torque_config);
    rw_motor_torque_container->UseSelfInit(rwMotorTorqueSelfInitFunc);
    rw_motor_torque_container->UseUpdate(rwMotorTorqueUpdateFunc);
    rw_motor_torque_container->UseReset(rwMotorTorqueResetFunc);

    CmdTorqueBodyMsg_C_subscribe(&rw_motor_torque_config->vehControlInMsg,
                                 &rate_servo_config->cmdTorqueOutMsg);
    RWArrayConfigMsg_cpp_subscribe(&rw_motor_torque_config->rwParamsInMsg,
                                   &message_provider->rwConfigOutMsg);

    attitudeControlTask->AddNewObject(message_provider, 11);
    attitudeControlTask->AddNewObject(css_wls_container, 10);
    attitudeControlTask->AddNewObject(sun_safe_point_container, 9);
    attitudeControlTask->AddNewObject(mrp_steering_container, 8);
    attitudeControlTask->AddNewObject(rate_servo_container, 7);
    attitudeControlTask->AddNewObject(rw_motor_torque_container, 6);

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
