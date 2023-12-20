cmake_minimum_required(VERSION 3.14)

# This is not clean, but as far is is needed for the prototyping effort. Any and all sources needed to compile either
# executable main.cpp and main-mrp-steering-controllers.cpp are set into the SRC variable and then the executable.
set(SRC
#        "main.cpp"
#        "main-mrp-steering-controllers.cpp"
#        "main-sun-safe.cpp"
        simulationDriver.h
        simulationDriver.cpp
        main-centroiding.cpp
        "../architecture/system_model/sys_process.h"
        "../architecture/system_model/sys_process.cpp"
        "../architecture/system_model/sim_model.h"
        "../architecture/system_model/sim_model.cpp"
        "../architecture/system_model/sys_model_task.h"
        "../architecture/system_model/sys_model_task.cpp"
        "../fswAlgorithms/imageProcessing/centerOfBrightness/centerOfBrightness.h"
        "../fswAlgorithms/opticalNavigation/cobConverter/cobConverter.h"
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
        messageProvider.h)

add_executable(SimulationDriver ${SRC})

target_include_directories(SimulationDriver PRIVATE "${CMAKE_SOURCE_DIR}")
set_target_properties(SimulationDriver PROPERTIES FOLDER "${DIR_NAME_STRING}")
target_link_libraries(SimulationDriver PUBLIC ${ARCHITECTURE_LIBS})
target_link_libraries(SimulationDriver PUBLIC attTrackingError)
target_link_libraries(SimulationDriver PUBLIC centerOfBrightness)
target_link_libraries(SimulationDriver PUBLIC cobConverter)
target_link_libraries(SimulationDriver PUBLIC cssWlsEst)
target_link_libraries(SimulationDriver PUBLIC flybyODuKF)
target_link_libraries(SimulationDriver PUBLIC flybyPoint)
target_link_libraries(SimulationDriver PUBLIC mrpPD)
target_link_libraries(SimulationDriver PUBLIC mrpSteering)
target_link_libraries(SimulationDriver PUBLIC rateServoFullNonlinear)
target_link_libraries(SimulationDriver PUBLIC rwMotorTorque)
target_link_libraries(SimulationDriver PUBLIC sunSafePoint)
target_link_libraries(SimulationDriver PUBLIC thrFiringSchmitt)
target_link_libraries(SimulationDriver PUBLIC thrForceMapping)

set(TARGET_NAME "SimulationDriver")
#include(usingOpenCV)

if(NOT WIN32)
  target_compile_options(SimulationDriver PUBLIC "-fPIC")
endif()

set_target_properties(SimulationDriver PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(SimulationDriver PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(SimulationDriver PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")
