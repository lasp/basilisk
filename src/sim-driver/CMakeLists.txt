cmake_minimum_required(VERSION 3.14)

set(SRC "main.cpp"
        "simulationDriver.h"
        "simulationDriver.cpp"
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
        "../architecture/alg_contain/alg_contain.h"
        "../architecture/alg_contain/alg_contain.cpp")

add_executable(SimulationDriver ${SRC})

#string(LENGTH ${CMAKE_SOURCE_DIR} DIR_NAME_START)
#math(EXPR DIR_NAME_START "${DIR_NAME_START} + 1")
#string(SUBSTRING ${CMAKE_CURRENT_SOURCE_DIR} ${DIR_NAME_START} -1 DIR_NAME_STRING)
target_include_directories(SimulationDriver PRIVATE "${CMAKE_SOURCE_DIR}")
set_target_properties(SimulationDriver PROPERTIES FOLDER "${DIR_NAME_STRING}")
target_link_libraries(SimulationDriver PUBLIC ${ARCHITECTURE_LIBS})
target_link_libraries(SimulationDriver PUBLIC centerOfBrightness)
target_link_libraries(SimulationDriver PUBLIC cobConverter)
target_link_libraries(SimulationDriver PUBLIC flybyODuKF)
target_link_libraries(SimulationDriver PUBLIC flybyPoint)
target_link_libraries(SimulationDriver PUBLIC attTrackingError)
target_link_libraries(SimulationDriver PUBLIC attTrackingError)
#target_link_libraries(SimulationDriver PUBLIC algContain)
set(TARGET_NAME "SimulationDriver")
include(usingOpenCV)

if(NOT WIN32)
  target_compile_options(SimulationDriver PUBLIC "-fPIC")
endif()

set_target_properties(SimulationDriver PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(SimulationDriver PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(SimulationDriver PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")
