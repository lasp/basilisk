set(module "simulation.environment.magneticFieldWMM")
xmera_is_module_enabled("${module}" _is_enabled)
if(NOT _is_enabled)
  return()
endif()

xmera_add_swig_module("${module}")
target_sources("${module}" PRIVATE
  magneticFieldWMM.cpp
  GeomagnetismLibrary.c
)

target_link_libraries("${module}" PRIVATE
  ArchitectureUtilities
  environmentCommon
)

# These are GCC/Clang warning suppressions for the vendored WMM source; MSVC
# rejects them as invalid command-line arguments (D8021).
if(NOT MSVC)
  set_source_files_properties(GeomagnetismLibrary.c PROPERTIES
    COMPILE_OPTIONS "-Wno-unused-but-set-variable;-Wno-misleading-indentation"
  )
endif()
