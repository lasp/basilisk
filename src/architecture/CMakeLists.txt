# Build a linkable core library for C++ modules
add_library(xmera_core SHARED
  _GeneralModuleFiles/sys_model.cpp
  utilities/bskLogging.cpp
)

# TODO: Organize per-artifact includes in a sane way.
# _For now_, making each artifact be explicit abouts its in-tree includes
# keeps things explicit.
target_include_directories(xmera_core PRIVATE
  # Source includes
  # @TODO: All local includes should ultimately be made relative to CMAKE_SOURCE_DIR.
  "${CMAKE_CURRENT_SOURCE_DIR}"
  # Project-wide includes
  "${CMAKE_SOURCE_DIR}"
)

# The link is PUBLIC, thus every target that uses the core also gets the mission include
# directory. Other libraries and test executables do not declare the directory again.
target_link_libraries(xmera_core PUBLIC Xmera::MissionParameters)

install(TARGETS xmera_core DESTINATION lib)

add_library(Xmera::Core ALIAS xmera_core)


add_subdirectory(utilities)
add_subdirectory(system_model)
add_subdirectory(msgPayloadDef)

add_subdirectory(utilitiesSelfCheck)
