#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "aidin_hand2::aidin_hand2" for configuration "RelWithDebInfo"
set_property(TARGET aidin_hand2::aidin_hand2 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(aidin_hand2::aidin_hand2 PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libaidin_hand2.so.0.9.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libaidin_hand2.so.0.9"
  )

list(APPEND _cmake_import_check_targets aidin_hand2::aidin_hand2 )
list(APPEND _cmake_import_check_files_for_aidin_hand2::aidin_hand2 "${_IMPORT_PREFIX}/lib/libaidin_hand2.so.0.9.1" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
