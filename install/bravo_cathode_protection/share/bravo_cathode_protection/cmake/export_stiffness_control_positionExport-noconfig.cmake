#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "bravo_cathode_protection::stiffness_control_position" for configuration ""
set_property(TARGET bravo_cathode_protection::stiffness_control_position APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(bravo_cathode_protection::stiffness_control_position PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libstiffness_control_position.so"
  IMPORTED_SONAME_NOCONFIG "libstiffness_control_position.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS bravo_cathode_protection::stiffness_control_position )
list(APPEND _IMPORT_CHECK_FILES_FOR_bravo_cathode_protection::stiffness_control_position "${_IMPORT_PREFIX}/lib/libstiffness_control_position.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
