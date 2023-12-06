cmake_minimum_required(VERSION 3.14)
enable_language(ASM)

# This is not clean, but as far is is needed for the prototyping effort. Any and all sources needed to compile either
# executable main.cpp and main-mrp-steering-controllers.cpp are set into the SRC variable and then the executable.
set(COMMON_SRC
        "main-sun-safe.cpp"
        "../simulationDriver.h"
        "../simulationDriver.cpp"
        "../../architecture/system_model/sys_process.h"
        "../../architecture/system_model/sys_process.cpp"
        "../../architecture/system_model/sim_model.h"
        "../../architecture/system_model/sim_model.cpp"
        "../../architecture/system_model/sys_model_task.h"
        "../../architecture/system_model/sys_model_task.cpp"
        "../../fswAlgorithms/opticalNavigation/flybyODuKF/flybyODuKF.h"
        "../../fswAlgorithms/attGuidance/flybyPoint/flybyPoint.h"
        "../../fswAlgorithms/attDetermination/CSSEst/cssWlsEst.h"
        "../../fswAlgorithms/attGuidance/sunSafePoint/sunSafePoint.h"
        "../../fswAlgorithms/attControl/mrpSteering/mrpSteering.h"
        "../../fswAlgorithms/attControl/rateServoFullNonlinear/rateServoFullNonlinear.h"
        "../../fswAlgorithms/effectorInterfaces/rwMotorTorque/rwMotorTorque.h"
        "../../fswAlgorithms/attControl/mrpPD/mrpPD.h"
        "../../fswAlgorithms/effectorInterfaces/thrForceMapping/thrForceMapping.h"
        "../../fswAlgorithms/effectorInterfaces/thrFiringSchmitt/thrFiringSchmitt.h"
        "../../architecture/alg_contain/alg_contain.h"
        "../../architecture/alg_contain/alg_contain.cpp"
        "../messageProvider.cpp"
        "../messageProvider.h"
        # Added by Kevin for static linking instead of dynamic:
        "../../fswAlgorithms/opticalNavigation/flybyODuKF/flybyODuKF.cpp"
        "../../fswAlgorithms/attGuidance/flybyPoint/flybyPoint.cpp"
        "../../architecture/utilities/bskLogging.cpp"
        "../../architecture/_GeneralModuleFiles/sys_model.cpp"
        "../../architecture/utilities/linearAlgebra.c"
        "../../fswAlgorithms/attGuidance/attTrackingError/attTrackingError.c"
        "../../architecture/utilities/rigidBodyKinematics.c"
        "../../architecture/utilities/avsEigenSupport.cpp"
        "../../architecture/utilities/moduleIdGenerator/moduleIdGenerator.cpp"
        "../../fswAlgorithms/attDetermination/CSSEst/cssWlsEst.c"
        "../../fswAlgorithms/attGuidance/sunSafePoint/sunSafePoint.c"
        "../../fswAlgorithms/attControl/mrpSteering/mrpSteering.c"
        "../../fswAlgorithms/attControl/rateServoFullNonlinear/rateServoFullNonlinear.c"
        "../../fswAlgorithms/effectorInterfaces/rwMotorTorque/rwMotorTorque.c"
        "../../fswAlgorithms/attControl/mrpPD/mrpPD.c"
        "../../fswAlgorithms/effectorInterfaces/thrForceMapping/thrForceMapping.c"
        "../../fswAlgorithms/effectorInterfaces/thrFiringSchmitt/thrFiringSchmitt.c"
        # Add auto source - autocoded message defs
        "../cMsgCInterface/AttGuidMsg_C.cpp"
        "../cMsgCInterface/AttRefMsg_C.cpp"
        "../cMsgCInterface/NavAttMsg_C.cpp"
        "../cMsgCInterface/CmdTorqueBodyMsg_C.cpp"
        "../cMsgCInterface/RWAvailabilityMsg_C.cpp"
        "../cMsgCInterface/ArrayMotorTorqueMsg_C.cpp"
        "../cMsgCInterface/SunlineFilterMsg_C.cpp"
        "../cMsgCInterface/VehicleConfigMsg_C.cpp"
        "../cMsgCInterface/RWSpeedMsg_C.cpp"
        "../cMsgCInterface/RateCmdMsg_C.cpp"
        "../cMsgCInterface/RWArrayConfigMsg_C.cpp"
        "../cMsgCInterface/CSSArraySensorMsg_C.cpp"
        "../cMsgCInterface/CSSConfigMsg_C.cpp"
        "../cMsgCInterface/THRArrayOnTimeCmdMsg_C.cpp"
        "../cMsgCInterface/THRArrayConfigMsg_C.cpp"
        "../cMsgCInterface/THRArrayCmdForceMsg_C.cpp"
)


if(CMAKE_TOOLCHAIN_FILE)
  # if(CMAKE_TOOLCHAIN_FILE MATCHES "riscv.toolchain.cmake")
  #   set(PLATFORM_SRC
  #         # Platform source
  # 
  #         # This is for rocketchip
  #         "/share/ema-fsw-simple-benchmark/RV32imaf/benchPlatform.c"
  #         "/share/ema-fsw-simple-benchmark/RV32imaf/fpga_design_config/fpga_design_config.h"
  #         "/share/ema-fsw-simple-benchmark/RV32imaf/platform/miv_rv32_hal/miv_rv32_hal.h"
  #         "/share/ema-fsw-simple-benchmark/RV32imaf/platform/miv_rv32_hal/miv_rv32_hal.c"
  #         "/share/ema-fsw-simple-benchmark/RV32imaf/platform/miv_rv32_hal/miv_rv32_init.c"
  #         "/share/ema-fsw-simple-benchmark/RV32imaf/platform/miv_rv32_hal/miv_rv32_hal_version.h"
  #         "/share/ema-fsw-simple-benchmark/RV32imaf/platform/miv_rv32_hal/miv_rv32_assert.h"
  #         "/share/ema-fsw-simple-benchmark/RV32imaf/platform/miv_rv32_hal/miv_rv32_plic.h"
  #         "/share/ema-fsw-simple-benchmark/RV32imaf/platform/miv_rv32_hal/miv_rv32_regs.h"
  #         "/share/ema-fsw-simple-benchmark/RV32imaf/platform/miv_rv32_hal/miv_rv32_stubs.c"
  #         "/share/ema-fsw-simple-benchmark/RV32imaf/platform/miv_rv32_hal/miv_rv32_subsys.h"
  #         "/share/ema-fsw-simple-benchmark/RV32imaf/platform/miv_rv32_hal/miv_rv32_syscall.c"
  #         "/share/ema-fsw-simple-benchmark/RV32imaf/platform/miv_rv32_hal/sample_fpga_design_config.h"
  #         "/share/ema-fsw-simple-benchmark/RV32imaf/platform/miv_rv32_hal/miv_rv32_entry.S"
  # 
  #         # This is for microchip
  #         #"/share/ema-fsw-simple-benchmark/RV32imc/benchPlatform.c"
  #         #"/share/ema-fsw-simple-benchmark/RV32imc/fpga_design_config/fpga_design_config.h"
  #         #"/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_hal.h"
  #         #"/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_hal.c"
  #         #"/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_init.c"
  #         #"/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_hal_version.h"
  #         #"/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_assert.h"
  #         #"/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_plic.h"
  #         #"/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_regs.h"
  #         #"/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_stubs.c"
  #         #"/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_subsys.h"
  #         #"/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_syscall.c"
  #         #"/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/sample_fpga_design_config.h"
  #         #"/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_entry.S"
  #   )
else()
  set(PLATFORM_SRC
    ../linux_benchPlatform.c
  )
endif()

# Set the source files as the common source + the platform specific source
set(SRC ${COMMON_SRC} ${PLATFORM_SRC})

add_executable(SimulationDriver ${SRC})
#add_executable(my_program main.cpp)

# Set up eigen for embedded system.
# Disable C11 STD library implementations, they are not available on rv32.
target_compile_definitions(SimulationDriver PUBLIC EIGEN_HAS_CXX11_MATH=0)

# Setup includes (-I flags) for gcc
target_include_directories(SimulationDriver PRIVATE
  "${CMAKE_SOURCE_DIR}"
  "${CMAKE_SOURCE_DIR}/.."
  "${CMAKE_SOURCE_DIR}/../.."
  "${CMAKE_SOURCE_DIR}/../eigen"
  "/share/basilisk/dist3/autoSource"
  "/share/ema-fsw-simple-benchmark/"
  # IMAF
  "/share/ema-fsw-simple-benchmark/RV32imaf"
  "/share/ema-fsw-simple-benchmark/RV32imaf/platform"
  # IMC
  #"/share/ema-fsw-simple-benchmark/RV32imc"
  #"/share/ema-fsw-simple-benchmark/RV32imc/platform"
)

set_target_properties(SimulationDriver PROPERTIES FOLDER "${DIR_NAME_STRING}")

set(TARGET_NAME "SimulationDriver")

if(NOT WIN32)
  target_compile_options(SimulationDriver PUBLIC "-fPIC")
endif()

set_target_properties(SimulationDriver PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(SimulationDriver PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(SimulationDriver PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")
