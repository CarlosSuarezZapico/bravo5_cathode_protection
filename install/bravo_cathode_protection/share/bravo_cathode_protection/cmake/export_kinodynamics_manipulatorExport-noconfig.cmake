#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "bravo_cathode_protection::kinodynamics_manipulator" for configuration ""
set_property(TARGET bravo_cathode_protection::kinodynamics_manipulator APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(bravo_cathode_protection::kinodynamics_manipulator PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libkinodynamics_manipulator.so"
  IMPORTED_SONAME_NOCONFIG "libkinodynamics_manipulator.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS bravo_cathode_protection::kinodynamics_manipulator )
list(APPEND _IMPORT_CHECK_FILES_FOR_bravo_cathode_protection::kinodynamics_manipulator "${_IMPORT_PREFIX}/lib/libkinodynamics_manipulator.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
