#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "kortex_description::kortex_description" for configuration ""
set_property(TARGET kortex_description::kortex_description APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(kortex_description::kortex_description PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libkortex_description.so"
  IMPORTED_SONAME_NOCONFIG "libkortex_description.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS kortex_description::kortex_description )
list(APPEND _IMPORT_CHECK_FILES_FOR_kortex_description::kortex_description "${_IMPORT_PREFIX}/lib/libkortex_description.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
