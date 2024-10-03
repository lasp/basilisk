cmake_minimum_required(VERSION 3.26)

project("basilisk" C CXX)
enable_testing()


################################################################################
## Cache (user-controllable) parameters

set(CMAKE_BUILD_TYPE Debug CACHE STRING
  "The build configuration (Debug, Release, RelWithDebInfo, MinSizeRel")
set(BUILD_OPNAV OFF CACHE BOOL
  "Whether to include the additional OpNav modules in the build.")
set(BUILD_VIZINTERFACE ON CACHE BOOL
  "Whether to include the additional VizInterface modules in the build.")
set(EXTERNAL_MODULES_PATH "" CACHE PATH
  "The path, if any, to an additional set of Basilisk module sources to be built.")
set(GTEST_FORCE_SHARED_CRT ON CACHE BOOL
  "[Windows] Make GTest link the C runtime dynamically.")
set(EXCLUDED_BSK_TARGETS "" CACHE STRING
  "A semicolon-delimited list of targets to exclude from the build.")
set(USE_COVERAGE OFF CACHE BOOL
  "Whether to enable GCOV code coverage analysis")


## Toolchain parameters

# Direct SWIG to emit depfiles to support accurate incremental rebuilds.
set(SWIG_USE_SWIG_DEPENDENCIES ON)
# Set C++ language standard.
set(CMAKE_CXX_STANDARD 17)
# Always generate position-independent code.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(MSVC)
  set(CMAKE_MSVCIDE_RUN_PATH "${CMAKE_BINARY_DIR}/Basilisk")

  add_compile_options(/MP /W3 /wd"4913" /wd"4251" /wd"5208" /wd"4267" /bigobj /EHsc)
  add_compile_definitions(
    _USE_MATH_DEFINES
    _CRT_SECURE_NO_WARNINGS
    _WINSOCK_DEPRECATED_NO_WARNINGS
    _WIN32_WINNT=0x0501 # Windows XP
  )
else()
  # Set the `install_name` of our built shared libraries with `@rpath`.
  set(CMAKE_MACOSX_RPATH ON)
  string(APPEND CMAKE_CXX_FLAGS " -Wall")
  string(APPEND CMAKE_CXX_FLAGS_RELEASE " -O2 -DNDEBUG")
  string(APPEND CMAKE_CXX_FLAGS_DEBUG " -g -gdwarf-3")
endif()

set(CMAKE_SKIP_BUILD_RPATH FALSE) # use, i.e. don't skip the full RPATH for the build tree
if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS YES)
elseif(APPLE)
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) # when building, don't use the install RPATH already (but later on when
                                           # installing)
  set(CMAKE_INSTALL_RPATH "${CMAKE_BINARY_DIR}/Basilisk") # the RPATH to be used when installing

  # don't add the automatically determined parts of the RPATH which point to directories outside the build tree to the
  # install RPATH
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
else()
  set(CMAKE_INSTALL_RPATH "\$ORIGIN/../../")
endif()


################################################################################
## External dependencies

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(SWIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)

include(UseSwig)
include(GoogleTest)
include(FetchContent)

# Construct an import target around the CSPICE library,
# since the `vcpkg` port for CSPICE doesn't seem to do it for us.
block()
  find_path(CSPICE_INCLUDE_DIR cspice/SpiceUsr.h REQUIRED)
  find_library(CSPICE_LIBRARY cspice REQUIRED)

  message(STATUS "${CSPICE_LIBRARY}")
  add_library(cspice::cspice STATIC IMPORTED GLOBAL)
  target_include_directories(cspice::cspice
    INTERFACE "${CSPICE_INCLUDE_DIR}")
  set_target_properties(cspice::cspice PROPERTIES
    # CMake complains vociferously if the config-specific versions
    # of IMPORT_LOCATION aren't given, even though the documentation
    # says the nonspecific version will be used if set.
    IMPORTED_LOCATION "${CSPICE_LIBRARY}"
    IMPORTED_LOCATION_DEBUG "${CSPICE_LIBRARY}"
    IMPORTED_LOCATION_RELEASE "${CSPICE_LIBRARY}")
endblock()

# Hack the Python3 module link options to *not* use `-undefined dynamic_lookup`.
# Instead, pass an explicit allowlist of symbols that may be left undefined.
# This trick is adapted from:
#    https://github.com/wjakob/nanobind/blob/0a6f8f36006dadff4b6279f58ce2188be20c9ada/cmake/nanobind-config.cmake#L33-L42
# The allowlist is obtained from:
#    https://github.com/python/cpython/issues/97524#issuecomment-1460492751
if(APPLE)
  set_target_properties(Python3::Module PROPERTIES
    INTERFACE_LINK_OPTIONS
      "-Wl,@${CMAKE_CURRENT_SOURCE_DIR}/cmake/cpython-exported-symbols"
  )
endif()

################################################################################

# ! create_symlinks : create symlinks to support files in source rather than keeping them in dist The function takes one
# required argument and N optional arguments. \argn: a list of supporting file paths
#
function(create_symlinks destinationPath)
  # Do nothing if building in-source
  if(${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    return()
  endif()

  foreach(filePath ${ARGN})
    get_filename_component(fileName ${filePath} NAME)

    # Get OS dependent path to use in `execute_process`
    file(TO_NATIVE_PATH "${destinationPath}/${fileName}" link)
    file(TO_NATIVE_PATH "${filePath}" target)

    # Delete the files from the dist directory, and instead just link them to the files in source
    file(REMOVE ${destinationPath}/${fileName})

    if(UNIX)
      set(command ln -s ${target} ${link})
    elseif(IS_DIRECTORY ${target})
      message("Creating Directory Symbolic Link")
      set(command cmd.exe /c mklink /D ${link} ${target})
    else()
      # message(target)
      set(command cmd.exe /c mklink ${link} ${target})
    endif()

    execute_process(
      COMMAND ${command}
      RESULT_VARIABLE result
      ERROR_VARIABLE output)

    # If the symlink is unsuccessful, then copy the file directly into dist
    if(NOT ${result} EQUAL 0)
      file(COPY ${target} DESTINATION ${destinationPath})
      message("Could not create symbolic link for: ${target} --> ${output}.  Copied instead")
    endif()

  endforeach(filePath)
endfunction(create_symlinks)

function(add_message_headers)
  file(GLOB CMESSAGE_HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/architecture/msgPayloadDefC/*.h"
       "${EXTERNAL_MODULES_PATH}/msgPayloadDefC/*.h")
  file(GLOB CPPMESSAGE_HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/architecture/msgPayloadDefCpp/*.h"
       "${EXTERNAL_MODULES_PATH}/msgPayloadDefCpp/*.h")
  add_custom_target(msgPayloadDefC SOURCES ${CMESSAGE_HEADER_FILES})
  add_custom_target(msgPayloadDefCpp SOURCES ${CPPMESSAGE_HEADER_FILES})
  set_target_properties(msgPayloadDefC PROPERTIES FOLDER architecture)
  set_target_properties(msgPayloadDefCpp PROPERTIES FOLDER architecture)
endfunction(add_message_headers)

function(sub_dir_list result curdir)
  # Searches through all directories in source, and keeps those called GeneralModuleFiles
  file(
    GLOB_RECURSE children
    LIST_DIRECTORIES true
    RELATIVE ${curdir}
    ${curdir}/*/)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      string(FIND ${curdir}/${child} "_GeneralModuleFiles" FOUND)
      string(FIND ${curdir}/${child} "__pycache__" FOUND_PY)
      string(FIND ${curdir}/${child} "_UnitTest" FOUND_UNIT)
      if(${FOUND} GREATER -1
         AND ${FOUND_PY} EQUAL -1
         AND ${FOUND_UNIT} EQUAL -1)
        list(APPEND dirlist ${child})
      endif()
    endif()
  endforeach()
  set(${result}
      ${dirlist}
      PARENT_SCOPE)
endfunction(sub_dir_list)

function(generate_package_libraries INIT_DIRECTORY AllLibs)
  # Find all _GeneralModuleFiles and put them into library targets so they aren't rewrapped, built with every module.

  # Find all _GeneralModuleFiles directories
  sub_dir_list(LIB_DIRS ${INIT_DIRECTORY})
  foreach(LIB_DIR ${LIB_DIRS})

    # Determine the name of the parent directory of _GeneralModuleFiles (i.e. dynamics, environment, etc)
    get_filename_component(PARENT_DIR ${LIB_DIR} DIRECTORY) # Relative to the source directory (e.g. simulation/power)
    string(FIND ${PARENT_DIR} "/" DIR_NAME_IDX REVERSE)
    math(EXPR DIR_NAME_IDX "${DIR_NAME_IDX} + 1") # "simulation/"
    string(SUBSTRING ${PARENT_DIR} ${DIR_NAME_IDX} -1 PARENT_DIR_NAME) # Should be "power"

    # Set the library name (ex. dynamicsLib, environmentLib, etc)
    set(LIB_NAME "${PARENT_DIR_NAME}Lib")
    # Added to build ExternalModulesLib
    file(
      GLOB LIB_DIR
      RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
      "${INIT_DIRECTORY}/${LIB_DIR}")
    # Grab the library source files
    file(GLOB C_FILES "${LIB_DIR}/*.cpp" "${LIB_DIR}/*.c" "${LIB_DIR}/*.h")
    # Grab the framework source files
    file(GLOB BSK_FWK_FILES "${CMAKE_CURRENT_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.cpp" # Might not be needed
         "${CMAKE_CURRENT_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.h"
         "${CMAKE_CURRENT_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.c")

    # Add Target
    add_library(${LIB_NAME} SHARED ${C_FILES} ${BSK_FWK_FILES})

    # Add to list of library
    list(APPEND AllLibs ${LIB_NAME})

    # Link all necessary libraries
    target_link_libraries(${LIB_NAME} PUBLIC ArchitectureUtilities)
    target_link_libraries(${LIB_NAME} PRIVATE ModuleIdGenerator)
    target_link_libraries(${LIB_NAME} PRIVATE Python3::Module)
    target_link_libraries(${LIB_NAME} PRIVATE Eigen3::Eigen)

    # define build location, IDE generation specifications
    set_target_properties(${LIB_NAME} PROPERTIES FOLDER ${PARENT_DIR})
    set_target_properties(${LIB_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
    set_target_properties(${LIB_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
    set_target_properties(${LIB_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")

    # Define the location for the executable part of the library (.dll) files (Windows Only)
    # https://cmake.org/cmake/help/v3.17/manual/cmake-buildsystem.7.html#output-artifacts for details
    set_target_properties(${LIB_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
    set_target_properties(${LIB_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
    set_target_properties(${LIB_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")

    # Define the location of the import library file (.lib) needed for the .dll (Windows Only)
    set_target_properties(${LIB_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
    set_target_properties(${LIB_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
    set_target_properties(${LIB_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")
  endforeach()
  set(AllLibs
      ${AllLibs}
      PARENT_SCOPE)
endfunction(generate_package_libraries)

function(find_package_targets PKG_DIR ALL_TARGET_LIST)
  file(
    GLOB_RECURSE SWIG_TARGETS
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "${PKG_DIR}/*.i")
  set(${ALL_TARGET_LIST}
      ${SWIG_TARGETS}
      PARENT_SCOPE)
endfunction(find_package_targets)

function(generate_package_targets TARGET_LIST LIB_DEP_LIST MODULE_DIR)
  if(TARGET_LIST)
    string(REPLACE "/" ";" DirList "${TARGET_LIST}")
    list(GET DirList 1 SUBMODULE)

    # It is redundant to compute simulation modules for each target. This should be fixed when modules' directories are
    # restructured.
    file(
      GLOB
      GEN_FILES
      "${CMAKE_CURRENT_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.c"
      "${CMAKE_CURRENT_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.h"
      "${CMAKE_CURRENT_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.i")

    if(${MODULE_DIR} STREQUAL "ExternalModules")
      file(
        GLOB
        GEN_FILES_LOCAL
        "${EXTERNAL_MODULES_PATH}/${MODULE_DIR}/_GeneralModuleFiles/*.c"
        "${EXTERNAL_MODULES_PATH}/${MODULE_DIR}/_GeneralModuleFiles/*.cpp"
        "${EXTERNAL_MODULES_PATH}/${MODULE_DIR}/_GeneralModuleFiles/*.h"
        "${EXTERNAL_MODULES_PATH}/${MODULE_DIR}/_GeneralModuleFiles/*.i")
    else()
      file(GLOB GEN_FILES_LOCAL "${MODULE_DIR}/${SUBMODULE}/_GeneralModuleFiles/*.c"
           "${MODULE_DIR}/${SUBMODULE}/_GeneralModuleFiles/*.cpp" "${MODULE_DIR}/${SUBMODULE}/_GeneralModuleFiles/*.h"
           "${MODULE_DIR}/${SUBMODULE}/_GeneralModuleFiles/*.i")
    endif()
  endif()

  foreach(TARGET_FILE ${TARGET_LIST})
    get_filename_component(TARGET_NAME ${TARGET_FILE} NAME_WE)
    get_filename_component(PARENT_DIR ${TARGET_FILE} DIRECTORY)

    if(${TARGET_NAME} IN_LIST EXCLUDED_BSK_TARGETS)
      message("Skipped Target: ${TARGET_NAME}")
        continue()
    endif()

    # Grab the target source files
    file(
      GLOB
      C_FILES
      "${PARENT_DIR}/*.c"
      "${PARENT_DIR}/*.cpp"
      "${PARENT_DIR}/*.h"
      "${PARENT_DIR}/*.cmake"
      "${PARENT_DIR}/*.rst")
    file(GLOB SWIG_DEP "${PARENT_DIR}/*.c" "${PARENT_DIR}/*.cpp" "${PARENT_DIR}/*.h")

    set_property(SOURCE ${TARGET_FILE} PROPERTY USE_TARGET_INCLUDE_DIRECTORIES TRUE) # Allows for include paths in the
                                                                                     # .i files
    set_property(SOURCE ${TARGET_FILE} PROPERTY CPLUSPLUS ON)

    # Add Target To add dependency so that any change to source file will reswig the interface file
    # https://stackoverflow.com/questions/31007635/is-there-a-way-for-cmake-to-utilize-dependencies-generated-by-swig-mm
    # https://public.kitware.com/pipermail/cmake/2012-December/053045.html
    set(SWIG_MODULE_${TARGET_NAME}_EXTRA_DEPS ${SWIG_DEP} ${GEN_FILES} ${GEN_FILES_LOCAL})
    swig_add_library(
      ${TARGET_NAME}
      LANGUAGE "python"
      TYPE MODULE
      SOURCES ${TARGET_FILE}
              ${C_FILES}
              # TODO: Following commented lines will become default with future imports OUTFILE_DIR
              # "${CMAKE_BINARY_DIR}/Basilisk/${PARENT_DIR}" #_wrap.c/.cxx file OUTPUT_DIR
              # "${CMAKE_BINARY_DIR}/Basilisk/${PARENT_DIR}") # generated .py file
              OUTFILE_DIR
              "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}" # _wrap.c/.cxx file
              OUTPUT_DIR
              "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}") # generated .py file

    # Collect any external dependencies
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${PARENT_DIR}/Custom.cmake")
      message(STATUS "Including custom Custom.cmake for: ${TARGET_FILE}")
      include("${PARENT_DIR}/Custom.cmake")
    endif()

    target_include_directories(
      ${TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/${PARENT_DIR}) # Exposes module .h files to the PYTHON_wrap.c(xx) file
                                                                # (not located in src)
    target_include_directories(${TARGET_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/_GeneralModuleFiles")

    target_include_directories(${TARGET_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/architecture/_GeneralModuleFiles"
    )# Exposes framework files for easy includes
    target_include_directories(${TARGET_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../libs") # Exposes third party library
                                                                                     # include folders

    # Link all necessary libraries
    foreach(LIB ${LIB_DEP_LIST})
      target_link_libraries(${TARGET_NAME} PRIVATE ${LIB})
    endforeach()
    target_link_libraries(${TARGET_NAME} PRIVATE Python3::Module)

    if(${MODULE_DIR} STREQUAL "ExternalModules")
      set_target_properties(${TARGET_NAME} PROPERTIES FOLDER ${MODULE_DIR})
    else()
      set_target_properties(${TARGET_NAME} PROPERTIES FOLDER ${PARENT_DIR}) # define folder in IDE
    endif()
    # Define the location for the .so / .dylib file
    set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
    set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
    set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")

    # Define the location for the executable part of the library (.dll) files (Windows Only)
    # https://cmake.org/cmake/help/v3.17/manual/cmake-buildsystem.7.html#output-artifacts for details
    set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
    set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
    set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")

    # Define the location of the import library file (.lib) needed for the .dll (Windows Only)
    set_target_properties(${TARGET_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
    set_target_properties(${TARGET_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
    set_target_properties(${TARGET_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
  endforeach()
endfunction(generate_package_targets)

################################################################################

if(NOT BUILD_VIZINTERFACE)
  list(APPEND EXCLUDED_BSK_TARGETS
    "vizInterface"
    "cielimInterface"
  )
endif()

if(NOT BUILD_OPNAV)
  list(APPEND EXCLUDED_BSK_TARGETS
    "limbFinding"
    "centerRadiusCNN"
    "houghCircles"
    "camera"
    "centerOfBrightness"
    "opticalFlow"
  )
endif()

if(USE_COVERAGE AND (CMAKE_COMPILER_IS_GNUC OR CMAKE_COMPILER_IS_GNUCXX))
  set(GCOV_FLAGS "-fprofile-arcs -ftest-coverage -O0 -fno-default-inline -fno-inline")

  string(APPEND CMAKE_C_FLAGS " ${GCOV_FLAGS}")
  string(APPEND CMAKE_CXX_FLAGS " ${GCOV_FLAGS}")
  string(APPEND CMAKE_LD_FLAGS " ${GCOV_FLAGS}")
endif()

# Check if the large Spice files are present.  If not, download from JPL site directly
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../supportData/EphemerisData/de430.bsp")
  message(STATUS "File de430.bsp not found, downloading file:")

  # Download it to the build directory (in case the download gets interrupted).
  file(
    DOWNLOAD
      "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de430.bsp"
      "${CMAKE_BINARY_DIR}/data/de430.bsp"
    SHOW_PROGRESS
    STATUS
      DOWNLOAD_RESULT
  )

  list(GET DOWNLOAD_RESULT 0 RESULT_CODE)
  if(NOT RESULT_CODE EQUAL 0)
    list(GET DOWNLOAD_RESULT 1 RESULT_MESSAGE)
    message(FATAL_ERROR "Failed downloading de430.bsp with error code: ${RESULT_CODE} : ${RESULT_MESSAGE}.")
  endif()

  # Move the SPICE file into the appropriate directory.
  file(RENAME "${CMAKE_BINARY_DIR}/data/de430.bsp" "${CMAKE_CURRENT_SOURCE_DIR}/../supportData/EphemerisData/de430.bsp")
else()
  message(STATUS "Found Spice files.")
endif()

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
link_libraries(Eigen3::Eigen)

# Targets that must be built first
add_subdirectory("architecture/utilities")
add_subdirectory("utilities") # has protobuffers included
add_subdirectory("architecture/messaging/")

# TODO: I'd call this generate_libraries(), because it really does find all of them. TODO: I'd like for all generated
# libraries to end up in a dist/Basilisk/lib folder rather than the /dist/Basilisk folder; however haven't found a way
# to use target_link_libraries() in a cross platform way to support this (needs full path and custom extensions
# --.dylib, .lib, .a, etc -- which gets messy in CMake).
set(AllLibs "")
generate_package_libraries("${CMAKE_CURRENT_SOURCE_DIR}" "${AllLibs}") # This finds GeneralModuleFiles and generates a library
                                                               # of the parentDirectory name

set(ARCHITECTURE_LIBS architectureLib ArchitectureUtilities ModuleIdGenerator)

# SIMULATION
# TODO: Move the following commands into a seperate CMakeList.txt s.t. this file is just configuration (problem:
# currently when I do this the targets argument in generate_package_targets gets interpreted differently) TODO: Automate
# this: 1) Look for package directories (dynamics, environment), 2) Find/Link any libraries associated with that package
# (dynamicsLib), 3) Look for custom CMakeLists.txt to link additional libraries.

# Finds relative path of all .i files within the directory
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/simulation/dynamics" DYN_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/simulation/environment" ENV_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/simulation/onboardDataHandling" DATA_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/simulation/deviceInterface" DEVICE_INTERFACE_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/simulation/power" POWER_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/simulation/navigation" NAV_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/simulation/sensors" SENSORS_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/simulation/simSynch" SIM_SYNCH_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/simulation/vizard" VIZ_INTERFACE_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/simulation/thermal" THERMAL_TARGETS)

if(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")
  string(REPLACE "\\" "/" EXTERNAL_MODULES_PATH "${EXTERNAL_MODULES_PATH}")
  include_directories("${EXTERNAL_MODULES_PATH}")
  find_package_targets("${EXTERNAL_MODULES_PATH}" EXTERNAL_MODULE_TARGETS)
  if(EXISTS "${EXTERNAL_MODULES_PATH}/ExternalModules/_GeneralModuleFiles")
    generate_package_libraries("${EXTERNAL_MODULES_PATH}" "${AllLibs}")
  endif()
  generate_package_targets("${EXTERNAL_MODULE_TARGETS}" "${AllLibs}" "ExternalModules")
endif(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")

# The quotations around DYN_TARGETS informs function that it is a list
generate_package_targets("${DYN_TARGETS}" "dynamicsLib;${ARCHITECTURE_LIBS}" "simulation")
generate_package_targets("${ENV_TARGETS}" "environmentLib;${ARCHITECTURE_LIBS};cspice::cspice" "simulation")
generate_package_targets("${DATA_TARGETS}" "onboardDataHandlingLib;${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${DEVICE_INTERFACE_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${POWER_TARGETS}" "powerLib;${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${NAV_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${SENSORS_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${SIM_SYNCH_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${VIZ_INTERFACE_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${THERMAL_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")

# FSW ALGORITHMS
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/_GeneralModuleFiles" GENERAL_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/attControl" ATT_CONTROL_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/attDetermination" ATT_DET_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/attGuidance" ATT_GUID_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/dvGuidance" DV_GUID_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/effectorInterfaces" EFF_INTERFACES_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/formationFlying" FORM_FLYING_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/fswUtilities" FSW_UTIL_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/imageProcessing" IMAGE_PROC_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/opticalNavigation" OPT_NAV_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/pointCloudProcessing" PNT_CLOUD_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/rwConfigData" RW_CONFIG_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/sensorInterfaces" SENSOR_INTERFACE_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/smallBodyNavigation" SMALLBODY_NAV_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/stateEstimation" STATE_EST_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/orbitControl" ORBIT_CONTROL_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/transDetermination" TRANS_DET_TARGETS)
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/fswAlgorithms/vehicleConfigData" VEH_CONFIG_TARGETS)

generate_package_targets("${GENERAL_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${ATT_CONTROL_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${ATT_DET_TARGETS}" "fswAlgorithmsLib;${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${ATT_GUID_TARGETS}" "attGuidanceLib;${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${DV_GUID_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${EFF_INTERFACES_TARGETS}" "effectorInterfacesLib;${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${FORM_FLYING_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${FSW_UTIL_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${IMAGE_PROC_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${OPT_NAV_TARGETS}" "fswAlgorithmsLib;${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${PNT_CLOUD_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${RW_CONFIG_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${SENSOR_INTERFACE_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${SMALLBODY_NAV_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${STATE_EST_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${ORBIT_CONTROL_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${TRANS_DET_TARGETS}" "transDeterminationLib;${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${VEH_CONFIG_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")

# TEMPLATE MODULES
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/moduleTemplates" TEMPLATE_MODULES_TARGETS)

generate_package_targets("${TEMPLATE_MODULES_TARGETS}" "${ARCHITECTURE_LIBS};" "moduleTemplates")

# MESSAGES
add_message_headers()

# ARCHITECTURE
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/architecture" ARCHITECTURE_TARGETS)
generate_package_targets("${ARCHITECTURE_TARGETS}" "${ARCHITECTURE_LIBS};" "architecture")

# Pyswice
find_package_targets("${CMAKE_CURRENT_SOURCE_DIR}/topLevelModules/pyswice" PYSWICE_TARGETS) # Finds relative path of all .i
                                                                                    # files within the directory
generate_package_targets("${PYSWICE_TARGETS}" "${ARCHITECTURE_LIBS};cspice::cspice" "topLevelModules")

# PYTHON PACKAGE CONFIGURATION
# Must make the build directories first, so that cmake can insert empty init files before build (linux specific need)
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/architecture")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/simulation")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/fswAlgorithms")

if(WIN32 AND (Python3_VERSION VERSION_GREATER 3.8))
  file(WRITE "${CMAKE_BINARY_DIR}/Basilisk/__init__.py"
       "#init file written by the build\n" "import sys, os\n" "from Basilisk import __path__\n"
       "bskPath = __path__[0]\n" "os.add_dll_directory(bskPath)\n")
else()
  file(WRITE "${CMAKE_BINARY_DIR}/Basilisk/__init__.py" "#empty init file written by the build")
endif()

# TODO: Iterate through all dist directories and add __init__.py's where they don't exist
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/topLevelModules")
file(
  GLOB_RECURSE DIST_DIRECTORIES
  LIST_DIRECTORIES true
  "${CMAKE_BINARY_DIR}/Basilisk/*")

foreach(DIR ${DIST_DIRECTORIES})
  if(IS_DIRECTORY ${DIR})
    file(
      GLOB DIST_DIR_FILES
      RELATIVE ${DIR}
      "${DIR}/*.py")
    if(DIST_DIR_FILES)
      list(FIND DIST_DIR_FILES "__init__.py" INIT_FOUND)
      if(${INIT_FOUND} EQUAL -1)
        file(WRITE "${DIR}/__init__.py" "")
      endif()
    else()
      file(WRITE "${DIR}/__init__.py" "")
    endif()
  endif()
endforeach()

# symlink utilities into the dist directory TODO: consider making the utilities directory it's own module with
# associated .i file and many `%pythoncode` directives.
file(GLOB pythonModules "${CMAKE_CURRENT_SOURCE_DIR}/utilities/*.py" "${CMAKE_CURRENT_SOURCE_DIR}/utilities/**")
create_symlinks("${CMAKE_BINARY_DIR}/Basilisk/utilities" ${pythonModules})

# symlink into package the supportData files to keep
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/supportData")
file(GLOB dataFiles "${CMAKE_CURRENT_SOURCE_DIR}/../supportData/*")
create_symlinks("${CMAKE_BINARY_DIR}/Basilisk/supportData" ${dataFiles})

# Tests targets
add_subdirectory("architecture/utilities/tests")
