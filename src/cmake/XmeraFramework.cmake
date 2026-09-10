# Root directories to add via `add_subdirectory()`
set(XMERA_MODULE_ROOTS
  "${CMAKE_SOURCE_DIR}/fswAlgorithms/"
  "${CMAKE_SOURCE_DIR}/simulation/"
  CACHE STRING
  "Semicolon-separated roots containing modules to add to the build. Each directory must contain a CMakeLists.txt."
)

set(XMERA_ENABLE_GROUPS ""
  CACHE STRING
  "Semicolon-separated group names or globs to enable."
)

set(XMERA_ENABLE_INTERNAL "NO"
  CACHE STRING
  "Whether to enable modules that are marked as INTERNAL (default NO)"
)

define_property(GLOBAL PROPERTY XMERA_MISSION_PARAMETERS_PROVIDER BRIEF_DOCS
  "This property is the one directory that xmera_provide_mission_parameters() declares. That \
function rejects a second, different directory. Thus this property has a maximum of one value."
)

# Every target that uses a mission-sized array bound must link to this target. This target is the
# one place that gives the mission include directory. A new target that does not link to it does
# not compile.
add_library(xmera_mission_parameters INTERFACE)
add_library(Xmera::MissionParameters ALIAS xmera_mission_parameters)

# Declare that <dir> contains the mission/parameters.h that this build must use. A module root
# calls this function from its own CMakeLists.txt. Thus the module root and its mission parameters
# are one selection and not two.
#
# This function rejects a second, different directory immediately. Thus the include path does not
# get two mission headers at the same time. Two mission headers that do not agree cause an ABI
# mismatch, and the build gives no error message. This mechanism prevents that failure. The check
# is in this function, thus it also applies to a build that does not call
# xmera_resolve_mission_parameters(). A second declaration of the same directory is permitted and
# has no effect.
function(xmera_provide_mission_parameters dir)
  get_filename_component(_dir "${dir}" ABSOLUTE)
  if(NOT EXISTS "${_dir}/mission/parameters.h")
    message(FATAL_ERROR
      "xmera_provide_mission_parameters(): there is no mission/parameters.h in '${_dir}'."
    )
  endif()

  get_property(_existing GLOBAL PROPERTY XMERA_MISSION_PARAMETERS_PROVIDER)
  if(_existing)
    if(NOT _existing STREQUAL _dir)
      message(FATAL_ERROR
        "The build declares two sources of mission/parameters.h:\n  ${_existing}\n  ${_dir}\n"
        "Only one source is permitted. If the build has two sources, different targets compile "
        "against different array bounds. Remove a module root. Or make the two headers into one "
        "mission header."
      )
    endif()
    return()
  endif()

  set_property(GLOBAL PROPERTY XMERA_MISSION_PARAMETERS_PROVIDER "${_dir}")
  target_include_directories(xmera_mission_parameters INTERFACE "${_dir}")
endfunction()

# Call this function one time, after the build adds every module root. At that time there is a
# maximum of one provider. Thus this function only gives the default values when no module root
# declared a directory.
function(xmera_resolve_mission_parameters)
  get_property(_provider GLOBAL PROPERTY XMERA_MISSION_PARAMETERS_PROVIDER)

  if(_provider)
    message(STATUS "Mission parameters: ${_provider}/mission/parameters.h")
  else()
    xmera_provide_mission_parameters("${CMAKE_SOURCE_DIR}/defaults")
    message(STATUS "Mission parameters: Xmera default values "
                   "(${CMAKE_SOURCE_DIR}/defaults/mission/parameters.h)")
  endif()
endfunction()

if(APPLE)
  set(XMERA_RPATH_ORIGIN "@loader_path")
else()
  set(XMERA_RPATH_ORIGIN "$ORIGIN")
endif()

define_property(GLOBAL PROPERTY XMERA_REGISTERED_MESSAGES BRIEF_DOCS
  "An accumulated list of messages registered via xmera_add_swig_message()"
)

function(_xmera_is_prefix prefix value out_var)
  set("${out_var}" OFF PARENT_SCOPE)

  string(LENGTH "${prefix}" _prefix_len)
  string(LENGTH "${value}" _value_len)
  if(_prefix_len LESS_EQUAL _value_len)
    string(SUBSTRING "${value}" 0 ${_prefix_len} _candidate)
    if(_candidate STREQUAL "${prefix}")
      set("${out_var}" ON PARENT_SCOPE)
    endif()
  endif()
endfunction()

function(xmera_is_module_enabled module_path out_var)
  cmake_parse_arguments(PARSE_ARGV 2 arg "INTERNAL" "" "")

  set("${out_var}" OFF PARENT_SCOPE)

  if((NOT arg_INTERNAL) OR XMERA_ENABLE_INTERNAL)
    foreach(_prefix IN LISTS XMERA_ENABLE_GROUPS)
      _xmera_is_prefix("${_prefix}." "${module_path}." _result)
      if(_result)
        set("${out_var}" "${_result}" PARENT_SCOPE)
        return()
      endif()
    endforeach()
  endif()
endfunction()

function(xmera_add_swig_module module)
  # separate "a.b.c" into "a" and "c"
  #  ... yes, ignore intermediate packages. everything just collapses. :(
  # TODO: separate "a.b.c" into "a/b" and "c"
  string(REPLACE "." ";" _package_components "${module}")
  list(POP_BACK _package_components _module_basename)
  list(POP_FRONT _package_components _package_path)
  # list(JOIN _package_components "/" _package_path)

  set(_gen_target_includes "$<TARGET_PROPERTY:${module},INCLUDE_DIRECTORIES>")
  set(_gen_swig_include_flags "$<LIST:TRANSFORM,${_gen_target_includes},PREPEND,-I>")
  add_custom_command(
    COMMENT "Generating SWIG Python/C++ wrapper: ${module}"
    OUTPUT
      "${CMAKE_CURRENT_BINARY_DIR}/${_module_basename}_wrap.cxx"
      "${CMAKE_CURRENT_BINARY_DIR}/${_module_basename}.py"
    COMMAND
      "${SWIG_EXECUTABLE}"
      -python
      -c++
      -MD
      -outcurrentdir
      "${_gen_swig_include_flags}"
      "${CMAKE_CURRENT_SOURCE_DIR}/${_module_basename}.i"
    WORKING_DIRECTORY
      "${CMAKE_CURRENT_BINARY_DIR}"
    MAIN_DEPENDENCY
      "${CMAKE_CURRENT_SOURCE_DIR}/${_module_basename}.i"
    DEPFILE
      "${CMAKE_CURRENT_BINARY_DIR}/${_module_basename}_wrap.d"
    VERBATIM
    COMMAND_EXPAND_LISTS
    DEPENDS_EXPLICIT_ONLY
  )

  add_library("${module}" MODULE
    "${CMAKE_CURRENT_BINARY_DIR}/${_module_basename}_wrap.cxx"
  )

  target_include_directories("${module}" PRIVATE
    # Source includes
    # @TODO: All local includes should ultimately be made relative to CMAKE_SOURCE_DIR.
    "${CMAKE_CURRENT_SOURCE_DIR}"
    # Project-wide includes
    "${CMAKE_SOURCE_DIR}"
    # Generated project-wide includes
    "${CMAKE_BINARY_DIR}"
    # @TODO add architecture/_GeneralModuleFiles to a global interface target or similar
    "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles"
  )

  # The directory is BEFORE the others, thus the compiler always reads the header of the mission
  # and not an in-tree src/mission/. A generator expression is necessary here, because module
  # roots declare their directory after CMake calls this function. The SWIG command line above
  # also uses this directory, because SWIG reads INCLUDE_DIRECTORIES.
  target_include_directories("${module}" BEFORE PRIVATE
    "$<TARGET_PROPERTY:Xmera::MissionParameters,INTERFACE_INCLUDE_DIRECTORIES>"
  )

  target_link_libraries("${module}" PRIVATE
    Xmera::Core
    Python3::Module
  )

  # TODO: once we actually use the full package path to install modules,
  #   we need to use the following logic to ensure that the relative rpath
  #   is correct no matter how deeply nested the module is.
  #
  # # "a.b.c" -> "xmera/a/b/_c.so" -> "../../../lib"
  # string(REGEX MATCHALL [[\.]] _rpath "${module}")
  # # _rpath = ".;."
  # list(APPEND _rpath ".")
  # # _rpath = ".;.;."
  # list(TRANSFORM _rpath PREPEND ".")
  # # _rpath = "..;..;.."
  # list(JOIN _rpath "/" _rpath)
  # # _rpath = "../../.."
  # set(_rpath "${_rpath}/lib")
  # # _rpath = "../../../lib"
  set(_rpath "../../lib")

  set_target_properties("${module}" PROPERTIES
    PREFIX "_"
    OUTPUT_NAME "${_module_basename}"
    INSTALL_RPATH "${XMERA_RPATH_ORIGIN}/${_rpath}"
  )

  install(
    TARGETS "${module}"
    DESTINATION "xmera/${_package_path}"
  )
  install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/${_module_basename}.py"
    DESTINATION "xmera/${_package_path}"
  )
endfunction()

function(xmera_add_swig_message message)
  cmake_parse_arguments(PARSE_ARGV 1 arg "" "TEMPLATE" "")

  if(NOT (DEFINED arg_TEMPLATE))
    set(arg_TEMPLATE "${CMAKE_SOURCE_DIR}/architecture/messaging/msgAutoSource/msgInterfacePy.i.in")
  endif()

  set_property(GLOBAL APPEND PROPERTY XMERA_REGISTERED_MESSAGES "${message}")

  add_custom_command(
    COMMENT "Generating SWIG message interface: ${message}"
    OUTPUT
      "${CMAKE_CURRENT_BINARY_DIR}/${message}.i"
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_SOURCE_DIR}/architecture/messaging/msgAutoSource/generateSWIGModules.py"
      "${CMAKE_CURRENT_BINARY_DIR}/${message}.i"
      "${arg_TEMPLATE}"
      "${message}"
      "${CMAKE_CURRENT_SOURCE_DIR}"
    WORKING_DIRECTORY
      "${CMAKE_SOURCE_DIR}/architecture/messaging/msgAutoSource"
    MAIN_DEPENDENCY
      "${CMAKE_CURRENT_SOURCE_DIR}/${message}.h"
    DEPENDS
      "${CMAKE_SOURCE_DIR}/architecture/messaging/msgAutoSource/generateSWIGModules.py"
      "${arg_TEMPLATE}"
    VERBATIM
    DEPENDS_EXPLICIT_ONLY
  )

  set(_gen_target_includes "$<TARGET_PROPERTY:${message},INCLUDE_DIRECTORIES>")
  set(_gen_swig_include_flags "$<LIST:TRANSFORM,${_gen_target_includes},PREPEND,-I>")
  add_custom_command(
    COMMENT "Generating SWIG Python/C++ wrapper: ${message}"
    OUTPUT
      "${CMAKE_CURRENT_BINARY_DIR}/${message}_wrap.cxx"
      "${CMAKE_CURRENT_BINARY_DIR}/${message}.py"
    COMMAND
      "${SWIG_EXECUTABLE}"
      -python
      -c++
      -MD
      -outcurrentdir
      "${_gen_swig_include_flags}"
      "${CMAKE_CURRENT_BINARY_DIR}/${message}.i"
    WORKING_DIRECTORY
      "${CMAKE_CURRENT_BINARY_DIR}"
    MAIN_DEPENDENCY
      "${CMAKE_CURRENT_BINARY_DIR}/${message}.i"
    DEPFILE
      "${CMAKE_CURRENT_BINARY_DIR}/${message}_wrap.d"
    VERBATIM
    COMMAND_EXPAND_LISTS
    DEPENDS_EXPLICIT_ONLY
  )

  add_library("${message}" MODULE
    "${CMAKE_CURRENT_BINARY_DIR}/${message}_wrap.cxx"
  )

  target_include_directories("${message}" PRIVATE
    # Source includes
    # @TODO: All local includes should ultimately be made relative to CMAKE_SOURCE_DIR.
    "${CMAKE_CURRENT_SOURCE_DIR}"
    # Project-wide includes
    "${CMAKE_SOURCE_DIR}"
  )

  # Refer to the note in xmera_add_swig_module().
  target_include_directories("${message}" BEFORE PRIVATE
    "$<TARGET_PROPERTY:Xmera::MissionParameters,INTERFACE_INCLUDE_DIRECTORIES>"
  )

  target_link_libraries("${message}" PRIVATE
    Python3::Module
    Eigen3::Eigen
    Xmera::Core
  )

  set_target_properties("${message}" PROPERTIES
    PREFIX "_"
    OUTPUT_NAME ${message}
    INSTALL_RPATH "${XMERA_RPATH_ORIGIN}/../../../lib"
  )

  install(
    TARGETS "${message}"
    DESTINATION "xmera/architecture/messaging"
  )
  install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/${message}.py"
    DESTINATION "xmera/architecture/messaging"
  )
endfunction()

function(xmera_generate_messaging_init)
  get_property(_file_contents GLOBAL PROPERTY XMERA_REGISTERED_MESSAGES)
  list(TRANSFORM _file_contents PREPEND "from xmera.architecture.messaging.")
  list(TRANSFORM _file_contents APPEND " import *")
  list(JOIN _file_contents "\n" _file_contents)

  file(GENERATE
    OUTPUT "${CMAKE_BINARY_DIR}/messaging__init__.py"
    CONTENT "${_file_contents}"
  )

  install(
    FILES "${CMAKE_BINARY_DIR}/messaging__init__.py"
    DESTINATION xmera/architecture/messaging/
    RENAME __init__.py
  )
endfunction()
