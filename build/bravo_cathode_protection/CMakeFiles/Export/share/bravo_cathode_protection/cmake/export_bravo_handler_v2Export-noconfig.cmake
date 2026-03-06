#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "bravo_cathode_protection::bravo_handler_v2" for configuration ""
set_property(TARGET bravo_cathode_protection::bravo_handler_v2 APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(bravo_cathode_protection::bravo_handler_v2 PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libbravo_handler_v2.so"
  IMPORTED_SONAME_NOCONFIG "libbravo_handler_v2.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS bravo_cathode_protection::bravo_handler_v2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_bravo_cathode_protection::bravo_handler_v2 "${_IMPORT_PREFIX}/lib/libbravo_handler_v2.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
