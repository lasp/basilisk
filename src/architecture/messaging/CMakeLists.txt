function(generate_messages searchDir)
  file(
    GLOB_RECURSE message_files
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/"
    "../../${searchDir}/*Payload.i" "${EXTERNAL_MODULES_PATH}/${searchDir}/*Payload.i")

  foreach(msgFile ${message_files})
    get_filename_component(TARGET_NAME ${msgFile} NAME_WE)
    set_property(SOURCE ${msgFile} PROPERTY CPLUSPLUS ON)

    if(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")
      set_property(
        SOURCE ${msgFile}
        PROPERTY SWIG_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}/../" "-I${CMAKE_CURRENT_SOURCE_DIR}/../../"
                 "-I${EXTERNAL_MODULES_PATH}/" "-I${CMAKE_BINARY_DIR}/autoSource/")
      include_directories("${EXTERNAL_MODULES_PATH}/")
    else()
      set_property(
        SOURCE ${msgFile}
        PROPERTY SWIG_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}/../" "-I${CMAKE_CURRENT_SOURCE_DIR}/../../"
                 "-I${CMAKE_BINARY_DIR}/autoSource/")
    endif(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")

    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../")

    swig_add_library(
      ${TARGET_NAME}
      LANGUAGE "python"
      TYPE MODULE
      SOURCES ${msgFile} OUTFILE_DIR "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/" # _wrap.c/.cxx file
              OUTPUT_DIR "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/")

    add_dependencies(${TARGET_NAME} swigtrick)
    set_target_properties(${TARGET_NAME} PROPERTIES SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE)
    set_target_properties(${TARGET_NAME} PROPERTIES FOLDER "architecture/messaging/derivedCode")
    set_target_properties(${SWIG_MODULE_${TARGET_NAME}_REAL_NAME}
                          PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging")
    set_target_properties(
      ${SWIG_MODULE_${TARGET_NAME}_REAL_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG
                                                         "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging")
    set_target_properties(
      ${SWIG_MODULE_${TARGET_NAME}_REAL_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE
                                                         "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging")
    target_link_libraries(${SWIG_MODULE_${TARGET_NAME}_REAL_NAME} PUBLIC architectureLib)
    target_link_libraries(${SWIG_MODULE_${TARGET_NAME}_REAL_NAME} PRIVATE Python3::SABIModule)
  endforeach()
endfunction(generate_messages)

set(message_directory_paths
        "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/"
        "../../../msgPayloadDef/"
)
if(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")
   list(APPEND message_directory_paths
          "${EXTERNAL_MODULES_PATH}/msgPayloadDef/")
endif (NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")

add_custom_command(
        OUTPUT ${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/__init__.py
        COMMAND
        ${Python3_EXECUTABLE} generateMessagePackageInit.py ${message_directory_paths}
        DEPENDS
          msgAutoSource/generateMessagePackageInit.py
          # TODO: add dependency on all message `.i` files
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msgAutoSource
        VERBATIM
)

# Custom target for establishing dependency
add_custom_target(swigtrick DEPENDS ${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/__init__.py)

# Dependency
generate_messages(msgPayloadDef)
