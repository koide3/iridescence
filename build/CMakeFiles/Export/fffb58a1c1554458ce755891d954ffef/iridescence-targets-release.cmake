#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Iridescence::iridescence" for configuration "Release"
set_property(TARGET Iridescence::iridescence APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(Iridescence::iridescence PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libiridescence.so.0.1.9"
  IMPORTED_SONAME_RELEASE "libiridescence.so.1"
  )

list(APPEND _cmake_import_check_targets Iridescence::iridescence )
list(APPEND _cmake_import_check_files_for_Iridescence::iridescence "${_IMPORT_PREFIX}/lib/libiridescence.so.0.1.9" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
