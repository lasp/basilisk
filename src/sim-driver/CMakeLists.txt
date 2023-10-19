cmake_minimum_required(VERSION 3.14)

# This is not clean, but as far is is needed for the prototyping effort. Any and all sources needed to compile either
# executable main.cpp and main-mrp-steering-controllers.cpp are set into the SRC variable and then the executable.
set(SRC
#        "main.cpp"
        "main-mrp-steering-controllers.cpp"
        "simulationDriver.h"
        "simulationDriver.cpp"
        "../architecture/system_model/sys_process.h"
        "../architecture/system_model/sys_process.cpp"
        "../architecture/system_model/sim_model.h"
        "../architecture/system_model/sim_model.cpp"
        "../architecture/system_model/sys_model_task.h"
        "../architecture/system_model/sys_model_task.cpp"
        "../fswAlgorithms/opticalNavigation/flybyODuKF/flybyODuKF.h"
        "../fswAlgorithms/attGuidance/flybyPoint/flybyPoint.h"
        "../fswAlgorithms/attDetermination/CSSEst/cssWlsEst.h"
        "../fswAlgorithms/attGuidance/sunSafePoint/sunSafePoint.h"
        "../fswAlgorithms/attControl/mrpSteering/mrpSteering.h"
        "../fswAlgorithms/attControl/rateServoFullNonlinear/rateServoFullNonlinear.h"
        "../fswAlgorithms/effectorInterfaces/rwMotorTorque/rwMotorTorque.h"
        "../architecture/alg_contain/alg_contain.h"
        "../architecture/alg_contain/alg_contain.cpp"
        messageProvider.cpp
        messageProvider.h
        # Added by Kevin for static linking instead of dynamic:
        "../fswAlgorithms/opticalNavigation/flybyODuKF/flybyODuKF.cpp"
        "../fswAlgorithms/attGuidance/flybyPoint/flybyPoint.cpp"
        "../architecture/utilities/bskLogging.cpp"
        "../architecture/_GeneralModuleFiles/sys_model.cpp"
        "../architecture/utilities/linearAlgebra.c"
        "../fswAlgorithms/attGuidance/attTrackingError/attTrackingError.c"
        "../architecture/utilities/rigidBodyKinematics.c"
        "../architecture/utilities/avsEigenSupport.cpp"
        "../architecture/utilities/moduleIdGenerator/moduleIdGenerator.cpp"
        "../fswAlgorithms/attDetermination/CSSEst/cssWlsEst.c"
        "../fswAlgorithms/attGuidance/sunSafePoint/sunSafePoint.c"
        "../fswAlgorithms/attControl/mrpSteering/mrpSteering.c"
        "../fswAlgorithms/attControl/rateServoFullNonlinear/rateServoFullNonlinear.c"
        "../fswAlgorithms/effectorInterfaces/rwMotorTorque/rwMotorTorque.c"
        # Add auto source - autocoded message defs
        "/share/basilisk/dist3/autoSource/cMsgCInterface/AttGuidMsg_C.cpp"
        "/share/basilisk/dist3/autoSource/cMsgCInterface/AttRefMsg_C.cpp"
        "/share/basilisk/dist3/autoSource/cMsgCInterface/NavAttMsg_C.cpp"
        "/share/basilisk/dist3/autoSource/cMsgCInterface/CmdTorqueBodyMsg_C.cpp"
        "/share/basilisk/dist3/autoSource/cMsgCInterface/RWAvailabilityMsg_C.cpp"
        "/share/basilisk/dist3/autoSource/cMsgCInterface/ArrayMotorTorqueMsg_C.cpp"
        "/share/basilisk/dist3/autoSource/cMsgCInterface/SunlineFilterMsg_C.cpp"
        "/share/basilisk/dist3/autoSource/cMsgCInterface/VehicleConfigMsg_C.cpp"
        "/share/basilisk/dist3/autoSource/cMsgCInterface/RWSpeedMsg_C.cpp"
        "/share/basilisk/dist3/autoSource/cMsgCInterface/RateCmdMsg_C.cpp"
        "/share/basilisk/dist3/autoSource/cMsgCInterface/RWArrayConfigMsg_C.cpp"
        "/share/basilisk/dist3/autoSource/cMsgCInterface/CSSArraySensorMsg_C.cpp"
        "/share/basilisk/dist3/autoSource/cMsgCInterface/CSSConfigMsg_C.cpp"
)

add_executable(SimulationDriver ${SRC})
#add_executable(my_program main.cpp)

# Set up eigen for embedded system.
# Disable C11 STD library implementations, they are not available on rv32.
target_compile_definitions(SimulationDriver PUBLIC EIGEN_HAS_CXX11_MATH=0)

# Setup includes (-I flags) for gcc
target_include_directories(SimulationDriver PRIVATE 
  "${CMAKE_SOURCE_DIR}"
  "${CMAKE_SOURCE_DIR}/.."
  "${CMAKE_SOURCE_DIR}/eigen"
  "/share/basilisk/dist3/autoSource")
set_target_properties(SimulationDriver PROPERTIES FOLDER "${DIR_NAME_STRING}")

# Commenting out linked libraries. We going static baby.
# target_link_libraries(SimulationDriver PUBLIC ${ARCHITECTURE_LIBS})
# target_link_libraries(SimulationDriver PUBLIC centerOfBrightness)
# target_link_libraries(SimulationDriver PUBLIC cobConverter)
# target_link_libraries(SimulationDriver PUBLIC flybyODuKF)
# target_link_libraries(SimulationDriver PUBLIC flybyPoint)
# target_link_libraries(SimulationDriver PUBLIC attTrackingError)
# target_link_libraries(SimulationDriver PUBLIC rwMotorTorque)
# target_link_libraries(SimulationDriver PUBLIC cssWlsEst)
# target_link_libraries(SimulationDriver PUBLIC sunSafePoint)
# target_link_libraries(SimulationDriver PUBLIC mrpSteering)
# target_link_libraries(SimulationDriver PUBLIC rateServoFullNonlinear)
# target_link_libraries(SimulationDriver PUBLIC rwMotorTorque)

set(TARGET_NAME "SimulationDriver")

# No open CV for rv32
#include(usingOpenCV)

if(NOT WIN32)
  target_compile_options(SimulationDriver PUBLIC "-fPIC")
endif()

set_target_properties(SimulationDriver PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(SimulationDriver PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(SimulationDriver PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")
